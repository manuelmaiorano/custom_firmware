#include "FreeRTOS.h"
#include "task.h"
#include "driver_mpu6050_basic.h"
#include <SEGGER_SYSVIEW.h>
#include <stm32f7xx_hal.h>
#include "estimator.h"
#include "stabilizer_types.h"
#include "gpio_utils.h"

uint8_t res;
uint32_t i;
uint32_t times;
float g[3];
float dps[3];
float degrees;
mpu6050_address_t addr;


TaskHandle_t sensortask_handle;

#define SENSORS_NBR_OF_BIAS_SAMPLES     1024
#define GYRO_VARIANCE_BASE          50
#define GYRO_VARIANCE_THRESHOLD_X   (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y   (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z   (GYRO_VARIANCE_BASE)
#define GYRO_NBR_OF_AXES            3
#define GYRO_MIN_BIAS_TIMEOUT_MS    M2T(1*1000)

#define SENSORS_ACC_SCALE_SAMPLES  200


#define EXTI_LINE_SENS EXTI_LINE_8


typedef struct
{
  Axis3f     bias;
  Axis3f     variance;
  Axis3f     mean;
  bool       isBiasValueFound;
  bool       isBufferFilled;
  Axis3f*  bufHead;
  Axis3f   buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;

static void sensorsBiasObjInit(BiasObj* bias);
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut);
static void sensorsAddBiasValue(BiasObj* bias, float x, float y, float z);
static bool sensorsFindBiasValue(BiasObj* bias);

static bool processAccScale(float ax, float ay, float az);

static bool isInit = false;

static BiasObj gyro_bias;
static float accScaleSum = 0;
static float accScale = 1;



void sensor_task(void* param);


void sensor_task_init() {

    assert_param(xTaskCreate(sensor_task, "sens", 4*configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &sensortask_handle) == pdPASS);
}


void  EXTI9_5_IRQHandler(void)
{
  NVIC_ClearPendingIRQ(EXTI9_5_IRQn);

  uint32_t regval;
  uint32_t maskline;

  /* Compute line mask */
  maskline = (1uL << (EXTI_LINE_SENS & EXTI_PIN_MASK));


  regval = (EXTI->PR & maskline);
  if (regval != 0x00u)
  {  //EXTI_ClearITPendingBit(EXTI_LINE_11);
    EXTI->PR = maskline;
    if (isInit) {
        portBASE_TYPE  xHigherPriorityTaskWoken = pdFALSE;

        // Unlock interrupt handling task
        vTaskNotifyGiveFromISR(sensortask_handle, &xHigherPriorityTaskWoken);

        if(xHigherPriorityTaskWoken) {
        portYIELD();
        }
    }
  }
}

void setup_interrupt() {
    EXTI_ConfigTypeDef exti_config;
    EXTI_HandleTypeDef exti_handle;
    gpio_port_pin_t IRQ_PORT_PIN = {.port = GPIOC, .pin = GPIO_PIN_8};

    exti_config.Line = EXTI_LINE_SENS;
    exti_config.Mode = EXTI_MODE_INTERRUPT;
    exti_config.Trigger = EXTI_TRIGGER_RISING;
    exti_config.GPIOSel = EXTI_GPIOC;
    exti_handle.Line = EXTI_LINE_SENS;
    HAL_EXTI_SetConfigLine(&exti_handle, &exti_config);

    // Init pins
    __GPIOC_CLK_ENABLE();
    pinMode(IRQ_PORT_PIN, INPUT);

    NVIC_SetPriority(EXTI9_5_IRQn, 10);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void sensor_task(void* param) {

    setup_interrupt();

    addr = MPU6050_ADDRESS_AD0_LOW;
    res = mpu6050_basic_init(addr);
    assert_param(res == 0);

    isInit = true;

    measurement_t measurement;
    Axis3f vec;
    sensorsBiasObjInit(&gyro_bias);

    while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        mpu6050_clear_interrupt();
        if (mpu6050_basic_read(g, dps) != 0){
            assert_param(0);
            continue;
            //(void)mpu6050_basic_deinit();
            //assert_param(0);
        }
        //SEGGER_SYSVIEW_PrintfHost("acc: %x, %x, %x", g[0], g[1], g[2]);
        //SEGGER_SYSVIEW_PrintfHost("gyro: %x, %x, %x", dps[0], dps[1], dps[2]);

        sensorsAddBiasValue(&gyro_bias, dps[0], dps[1], dps[2]);
        if (!gyro_bias.isBiasValueFound) {
            sensorsFindBiasValue(&gyro_bias);
        } else {
            processAccScale(g[0], g[1], g[2]);
        }
        
        measurement.type = MeasurementTypeAcceleration;
        vec.x = g[0] / accScale; 
        vec.y = g[1] / accScale;
        vec.z = g[2] / accScale;
        measurement.data.acceleration.acc = vec;
        estimatorEnqueue(&measurement); 

        measurement.type = MeasurementTypeGyroscope;
        vec.x = dps[0] - gyro_bias.bias.x; 
        vec.y = dps[1] - gyro_bias.bias.y;
        vec.z = dps[2] - gyro_bias.bias.z;
        measurement.data.gyroscope.gyro = vec;
        estimatorEnqueue(&measurement); 

    }
}

static bool processAccScale(float ax, float ay, float az)
{
  static bool accBiasFound = false;
  static float accScaleSumCount = 0;

  if (!accBiasFound)
  {
    accScaleSum += sqrtf(powf(ax, 2) + powf(ay, 2) + powf(az, 2));
    accScaleSumCount++;

    if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
    {
      accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
      accBiasFound = true;
    }
  }

  return accBiasFound;
}

static void sensorsBiasObjInit(BiasObj* bias)
{
  bias->isBufferFilled = false;
  bias->bufHead = bias->buffer;
}

/**
 * Calculates the variance and mean for the bias buffer.
 */
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut)
{
  uint32_t i;
  float sum[GYRO_NBR_OF_AXES] = {0};
  float sumSq[GYRO_NBR_OF_AXES] = {0};

  for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
    sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
    sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
    sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
  }


  meanOut->x = (float) sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->y = (float) sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->z = (float) sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;

  varOut->x = sumSq[0] / SENSORS_NBR_OF_BIAS_SAMPLES - meanOut->x * meanOut->x;
  varOut->y = sumSq[1] / SENSORS_NBR_OF_BIAS_SAMPLES - meanOut->y * meanOut->y;
  varOut->z = sumSq[2] / SENSORS_NBR_OF_BIAS_SAMPLES - meanOut->z * meanOut->z;
}


/**
 * Adds a new value to the variance buffer and if it is full
 * replaces the oldest one. Thus a circular buffer.
 */
static void sensorsAddBiasValue(BiasObj* bias, float x, float y, float z)
{
  bias->bufHead->x = x;
  bias->bufHead->y = y;
  bias->bufHead->z = z;
  bias->bufHead++;

  if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES])
  {
    bias->bufHead = bias->buffer;
    bias->isBufferFilled = true;
  }
}

/**
 * Checks if the variances is below the predefined thresholds.
 * The bias value should have been added before calling this.
 * @param bias  The bias object
 */
static bool sensorsFindBiasValue(BiasObj* bias)
{
  static int32_t varianceSampleTime;
  bool foundBias = false;

  if (bias->isBufferFilled)
  {
    sensorsCalculateVarianceAndMean(bias, &bias->variance, &bias->mean);

    if (bias->variance.x < GYRO_VARIANCE_THRESHOLD_X &&
        bias->variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
        bias->variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
        (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount()))
    {
      varianceSampleTime = xTaskGetTickCount();
      bias->bias.x = bias->mean.x;
      bias->bias.y = bias->mean.y;
      bias->bias.z = bias->mean.z;
      foundBias = true;
      bias->isBiasValueFound = true;
    }
  }

  return foundBias;
}
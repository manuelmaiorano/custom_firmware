#include "estimator.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include "config.h"
#include <SEGGER_SYSVIEW.h>
#include <stm32f7xx_hal.h>
#include "locodeck.h"
#include "gpio_utils.h"
#include "kalman_core.h"
#include "axis3fSubSampler.h"

#include "mm_tdoa.h"

#include "physicalConstants.h"



#define MEASUREMENTS_QUEUE_SIZE (21)
static xQueueHandle measurementsQueue;
static StaticQueue_t queue_structure;
static uint8_t queue_array[MEASUREMENTS_QUEUE_SIZE * sizeof(measurement_t)];

static TaskHandle_t task_handle = 0;
static StackType_t estimatortask_Stack[KALMAN_TASK_STACKSIZE];
static StaticTask_t estimatortask_TCB;

static SemaphoreHandle_t runTaskSemaphore;
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;

const uint32_t PREDICTION_UPDATE_INTERVAL_MS = 10;

static kalmanCoreParams_t coreParams;

static state_t taskEstimatorState; // The estimator state produced by the task, copied to the stabilizer when needed.

static kalmanCoreData_t coreData;

static bool isInit = false;

static bool resetEstimation = false;


static Axis3fSubSampler_t accSubSampler;
static Axis3fSubSampler_t gyroSubSampler;
static Axis3f accLatest;
static Axis3f gyroLatest;

static OutlierFilterTdoaState_t outlierFilterTdoaState;
//static OutlierFilterLhState_t sweepOutlierFilterState;

static void kalmanTask(void* parameters);
static void updateQueuedMeasurements(const uint32_t nowMs);

void estimatorKalmanTaskInit() {
  kalmanCoreDefaultParams(&coreParams);

  // Created in the 'empty' state, meaning the semaphore must first be given, that is it will block in the task
  // until released by the stabilizer loop
  runTaskSemaphore = xSemaphoreCreateBinary();
  ASSERT(runTaskSemaphore);

  dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);

  measurementsQueue = xQueueCreateStatic(MEASUREMENTS_QUEUE_SIZE, sizeof(measurement_t), queue_array, &queue_structure);

  xTaskCreateStatic(kalmanTask, KALMAN_TASK_NAME, KALMAN_TASK_STACKSIZE, NULL, KALMAN_TASK_PRI, estimatortask_Stack, &estimatortask_TCB);

}

// Called when this estimator is activated
void estimatorKalmanInit(void)
{
  axis3fSubSamplerInit(&accSubSampler, GRAVITY_MAGNITUDE);
  axis3fSubSamplerInit(&gyroSubSampler, DEG_TO_RAD);

  outlierFilterTdoaReset(&outlierFilterTdoaState);
  //outlierFilterLighthouseReset(&sweepOutlierFilterState, 0);

  uint32_t nowMs = T2M(xTaskGetTickCount());
  kalmanCoreInit(&coreData, &coreParams, nowMs);
}

void kalman_init() {

    // measurementsQueue = xQueueCreateStatic(MEASUREMENTS_QUEUE_SIZE, sizeof(tdoaMeasurement_t), queue_array, &queue_structure);

    // assert_param(xTaskCreate(kalmanTask, KALMAN_TASK_NAME, KALMAN_TASK_STACKSIZE, NULL,
    //                 KALMAN_TASK_PRI, &task_handle) == pdPASS);
    estimatorKalmanTaskInit();

}

void estimatorEnqueue(const measurement_t* measurement) {
    xQueueSend(measurementsQueue, measurement, 0);
}

void estimatorEnqueueTDOA(const tdoaMeasurement_t *tdoa_measurement) {

    measurement_t m;
    m.type = MeasurementTypeTDOA;
    m.data.tdoa = *tdoa_measurement;
    //estimatorEnqueue(&m); 
    xQueueSend(measurementsQueue, &m, 0);

}

bool estimatorDequeue(measurement_t *measurement) {
  return pdTRUE == xQueueReceive(measurementsQueue, measurement, 0);
}

static void kalmanTask(void* parameters) {

    //init_loco_deck();

    //gpio_port_pin_t IRQ_PORT_PIN = {.port=GPIOC, .pin=GPIO_PIN_11 };
    //digitalWrite(IRQ_PORT_PIN, 1);

    uint32_t nowMs = T2M(xTaskGetTickCount());
    uint32_t nextPredictionMs = nowMs;
    bool first = true;
    estimatorKalmanInit();
    while (true) {
        xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);
        // if (resetEstimation) {
        //   estimatorKalmanInit();
        //   resetEstimation = false;
        // }
        nowMs = T2M(xTaskGetTickCount()); 
        if(first) {
          coreData.lastPredictionMs = nowMs;
          coreData.lastProcessNoiseUpdateMs = nowMs;
          first = false;
        }


        if (nowMs >= nextPredictionMs) {
          axis3fSubSamplerFinalize(&accSubSampler);
          axis3fSubSamplerFinalize(&gyroSubSampler);
          
          kalmanCorePredict(&coreData, &accSubSampler.subSample, &gyroSubSampler.subSample, nowMs, false);
     
          nextPredictionMs = nowMs + PREDICTION_UPDATE_INTERVAL_MS;

        }

        kalmanCoreAddProcessNoise(&coreData, &coreParams, nowMs);
        updateQueuedMeasurements(nowMs);

        kalmanCoreFinalize(&coreData);

        xSemaphoreTake(dataMutex, portMAX_DELAY);
        kalmanCoreExternalizeState(&coreData, &taskEstimatorState, &accLatest);
        xSemaphoreGive(dataMutex);


    }

}

void estimatorKalman(state_t *state, const stabilizerStep_t stabilizerStep) {
  // This function is called from the stabilizer loop. It is important that this call returns
  // as quickly as possible. The dataMutex must only be locked short periods by the task.
  xSemaphoreTake(dataMutex, portMAX_DELAY);

  // Copy the latest state, calculated by the task
  memcpy(state, &taskEstimatorState, sizeof(state_t));
  xSemaphoreGive(dataMutex);

  xSemaphoreGive(runTaskSemaphore);
}


static void updateQueuedMeasurements(const uint32_t nowMs) {
    measurement_t m;
    //char buffer[32];
    while (estimatorDequeue(&m)) {
        //
        switch (m.type)
        {
          case MeasurementTypeTDOA:
            kalmanCoreUpdateWithTdoa(&coreData, &m.data.tdoa, nowMs, &outlierFilterTdoaState);
            break;
          case MeasurementTypeGyroscope:
            axis3fSubSamplerAccumulate(&gyroSubSampler, &m.data.gyroscope.gyro);
            gyroLatest = m.data.gyroscope.gyro;
            break;
          case MeasurementTypeAcceleration:
            axis3fSubSamplerAccumulate(&accSubSampler, &m.data.acceleration.acc);
            accLatest = m.data.acceleration.acc;
            break;
          default:
            break;
        }
        

    }

}
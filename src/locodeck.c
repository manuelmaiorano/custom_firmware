
#include <stdint.h>
#include <string.h>
#include <stm32f7xx_hal.h>
#include <stm32f7xx_ll_system.h>


#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

#include <SEGGER_SYSVIEW.h>

#include "config.h"
#include "locodeck.h"
#include "spi_driver.h"
#include "gpio_utils.h"


#include "lpsTdoa2Tag.h"

static void dwm1000Init();

static const gpio_port_pin_t CS_PORT_PIN = {.port=GPIOB, .pin=GPIO_PIN_1};
static const gpio_port_pin_t RESET_PORT_PIN = {.port=GPIOC, .pin=GPIO_PIN_9};
static const gpio_port_pin_t IRQ_PORT_PIN = {.port=GPIOC, .pin=GPIO_PIN_12 };

#define EXTI_LINE EXTI_LINE_12
#define DEFAULT_RX_TIMEOUT 10000

static uwbAlgorithm_t *algorithm = &uwbTdoa2TagAlgorithm;

static bool isInit = false;
static TaskHandle_t uwbTaskHandle;
static StackType_t uwbtask_Stack[LPS_DECK_STACKSIZE];
static StaticTask_t uwbtask_TCB;


static SemaphoreHandle_t algoSemaphore;
static dwDevice_t dwm_device;
static dwDevice_t *dwm = &dwm_device;

static QueueHandle_t lppShortQueue;

static uint32_t timeout;

static EXTI_HandleTypeDef exti_handle;

static void txCallback(dwDevice_t *dev)
{
  timeout = algorithm->onEvent(dev, eventPacketSent);
}

static void rxCallback(dwDevice_t *dev)
{
  timeout = algorithm->onEvent(dev, eventPacketReceived);
}

static void rxTimeoutCallback(dwDevice_t * dev) {
  timeout = algorithm->onEvent(dev, eventReceiveTimeout);
}

static void rxFailedCallback(dwDevice_t * dev) {
  timeout = algorithm->onEvent(dev, eventReceiveFailed);
}

static void uwbTask(void* parameters) {
  lppShortQueue = xQueueCreate(10, sizeof(lpsLppShortPacket_t));

  //algoOptions.currentRangingMode = lpsMode_auto;

  algorithm->init(dwm);
  timeout = algorithm->onEvent(dwm, eventTimeout);
  //systemWaitStart();

  while(1) {
    // xSemaphoreTake(algoSemaphore, portMAX_DELAY);
    // handleModeSwitch();
    // xSemaphoreGive(algoSemaphore);

    if (ulTaskNotifyTake(pdTRUE, timeout / portTICK_PERIOD_MS) > 0) {
      do{
        xSemaphoreTake(algoSemaphore, portMAX_DELAY);
        dwHandleInterrupt(dwm);
        xSemaphoreGive(algoSemaphore);
      } while(HAL_GPIO_ReadPin(IRQ_PORT_PIN.port, IRQ_PORT_PIN.pin) != GPIO_PIN_RESET);
    } else {
      xSemaphoreTake(algoSemaphore, portMAX_DELAY);
      timeout = algorithm->onEvent(dwm, eventTimeout);
      xSemaphoreGive(algoSemaphore);
    }
  }
}

static lpsLppShortPacket_t lppShortPacket;

bool lpsSendLppShort(uint8_t destId, void* data, size_t length)
{
  bool result = false;

  if (isInit)
  {
    lppShortPacket.dest = destId;
    lppShortPacket.length = length;
    memcpy(lppShortPacket.data, data, length);
    result = xQueueSend(lppShortQueue, &lppShortPacket,0) == pdPASS;
  }

  return result;
}

bool lpsGetLppShort(lpsLppShortPacket_t* shortPacket)
{
  return xQueueReceive(lppShortQueue, shortPacket, 0) == pdPASS;
}

static uint8_t spiTxBuffer[196];
static uint8_t spiRxBuffer[196];
static uint16_t spiSpeed = SPI_BAUDRATE_2MHZ;

/************ Low level ops for libdw **********/
static void spiWrite(dwDevice_t* dev, const void *header, size_t headerLength,
                                      const void* data, size_t dataLength)
{
  spiBeginTransaction(spiSpeed);
  HAL_GPIO_WritePin(CS_PORT_PIN.port, CS_PORT_PIN.pin, GPIO_PIN_RESET);
  memcpy(spiTxBuffer, header, headerLength);
  memcpy(spiTxBuffer+headerLength, data, dataLength);
  spiExchange(headerLength+dataLength, spiTxBuffer, spiRxBuffer);
  HAL_GPIO_WritePin(CS_PORT_PIN.port, CS_PORT_PIN.pin, GPIO_PIN_SET);
  spiEndTransaction();
}

static void spiRead(dwDevice_t* dev, const void *header, size_t headerLength,
                                     void* data, size_t dataLength)
{
  spiBeginTransaction(spiSpeed);
  HAL_GPIO_WritePin(CS_PORT_PIN.port, CS_PORT_PIN.pin, GPIO_PIN_RESET);
  memcpy(spiTxBuffer, header, headerLength);
  memset(spiTxBuffer+headerLength, 0, dataLength);
  spiExchange(headerLength+dataLength, spiTxBuffer, spiRxBuffer);
  memcpy(data, spiRxBuffer+headerLength, dataLength);
  HAL_GPIO_WritePin(CS_PORT_PIN.port, CS_PORT_PIN.pin, GPIO_PIN_SET);
  spiEndTransaction();
}

void exti12_callback(void)
{
  if(isInit) {
     portBASE_TYPE  xHigherPriorityTaskWoken = pdFALSE;

    // Unlock interrupt handling task
    vTaskNotifyGiveFromISR(uwbTaskHandle, &xHigherPriorityTaskWoken);

    if(xHigherPriorityTaskWoken) {
      portYIELD();
    }
  }

}

void  EXTI15_10_IRQHandler(void)
{
  SEGGER_SYSVIEW_RecordEnterISR();
  HAL_EXTI_IRQHandler(&exti_handle);
  SEGGER_SYSVIEW_RecordExitISR();
}

static void spiSetSpeed(dwDevice_t* dev, dwSpiSpeed_t speed)
{
  if (speed == dwSpiSpeedLow)
  {
    spiSpeed = SPI_BAUDRATE_2MHZ;
  }
  else if (speed == dwSpiSpeedHigh)
  {
    spiSpeed = SPI_BAUDRATE_21MHZ;
  }
}

static void delayms(dwDevice_t* dev, unsigned int delay)
{
  vTaskDelay(M2T(delay));
}

static dwOps_t dwOps = {
  .spiRead = spiRead,
  .spiWrite = spiWrite,
  .spiSetSpeed = spiSetSpeed,
  .delayms = delayms,
};

/*********** Deck driver initialization ***************/

static void dwm1000Init()
{
  static EXTI_ConfigTypeDef exti_config;
  spiBegin();

  // Set up interrupt

  exti_config.Line = EXTI_LINE;
  exti_config.Mode = EXTI_MODE_INTERRUPT;
  exti_config.Trigger = EXTI_TRIGGER_RISING;
  exti_config.GPIOSel = EXTI_GPIOC;
  exti_handle.Line = EXTI_LINE;
  HAL_EXTI_SetConfigLine(&exti_handle, &exti_config);
  HAL_EXTI_RegisterCallback(&exti_handle, HAL_EXTI_COMMON_CB_ID, exti12_callback);

  // Init pins
  __GPIOC_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef gpio_init_struct;
  gpio_init_struct.Pin = CS_PORT_PIN.pin;
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_struct.Pull = GPIO_NOPULL;
  gpio_init_struct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(CS_PORT_PIN.port, &gpio_init_struct);

  gpio_init_struct.Pin = RESET_PORT_PIN.pin;
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_struct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RESET_PORT_PIN.port, &gpio_init_struct);

  NVIC_SetPriority(EXTI15_10_IRQn, 10);
  NVIC_EnableIRQ(EXTI15_10_IRQn);

  // Reset the DW1000 chip
  HAL_GPIO_WritePin(RESET_PORT_PIN.port, RESET_PORT_PIN.pin, GPIO_PIN_RESET);
  //HAL_Delay(10);
  vTaskDelay(M2T(10));
  HAL_GPIO_WritePin(RESET_PORT_PIN.port, RESET_PORT_PIN.pin, GPIO_PIN_SET);
  vTaskDelay(M2T(10));
  //HAL_Delay(10);

  //digitalWrite(IRQ_PORT_PIN, 1);

  // Initialize the driver
  dwInit(dwm, &dwOps);       // Init libdw

  int result = dwConfigure(dwm);
  if (result != 0) {
    isInit = false;
    return;
  }

  dwEnableAllLeds(dwm);

  dwTime_t delay = {.full = 0};
  dwSetAntenaDelay(dwm, delay);

  dwAttachSentHandler(dwm, txCallback);
  dwAttachReceivedHandler(dwm, rxCallback);
  dwAttachReceiveTimeoutHandler(dwm, rxTimeoutCallback);
  dwAttachReceiveFailedHandler(dwm, rxFailedCallback);

  dwNewConfiguration(dwm);
  dwSetDefaults(dwm);


  dwEnableMode(dwm, MODE_SHORTDATA_FAST_ACCURACY);

  dwSetChannel(dwm, CHANNEL_2);
  dwSetPreambleCode(dwm, PREAMBLE_CODE_64MHZ_9);

  dwUseSmartPower(dwm, true);

  dwSetReceiveWaitTimeout(dwm, DEFAULT_RX_TIMEOUT);

  dwCommitConfiguration(dwm);


  algoSemaphore= xSemaphoreCreateMutex();

  uwbTaskHandle = xTaskCreateStatic(uwbTask, LPS_DECK_TASK_NAME, LPS_DECK_STACKSIZE, NULL, LPS_DECK_TASK_PRI, uwbtask_Stack, &uwbtask_TCB);


  isInit = true;

}


void init_loco_deck() {
  dwm1000Init();
}


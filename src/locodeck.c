#define DEBUG_MODULE "DWM"

#include <stdint.h>
#include <string.h>
#include <stm32f7xx.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

#include "config.h"
#include <stm32f7xx_hal_exti.h>
#include <stm32f7xx_hal_spi.h>
#include <stm32f7xx_hal_gpio.h>
#include <stm32f7xx_ll_system.h>
#include "gpio_utils.h"
#include "locodeck.h"

#include "lpsTdoa2Tag.h"


const gpio_port_pin_t CS_PORT_PIN = {.port=GPIOB, .pin=GPIO_PIN_8 };
const gpio_port_pin_t RESET_PORT_PIN = {.port=GPIOC, .pin=GPIO_PIN_10};
const gpio_port_pin_t IRQ_PORT_PIN = {.port=GPIOC, .pin=GPIO_PIN_11 };


#define DEFAULT_RX_TIMEOUT 10000

// The anchor position can be set using parameters
// As an option you can set a static position in this file and set
// combinedAnchorPositionOk to enable sending the anchor rangings to the Kalman filter

static lpsAlgoOptions_t algoOptions = {
  // .userRequestedMode is the wanted algorithm, available as a parameter
#if defined(CONFIG_DECK_LOCO_ALGORITHM_TDOA2)
  .userRequestedMode = lpsMode_TDoA2,
#elif defined(CONFIG_DECK_LOCO_ALGORITHM_TDOA3)
  .userRequestedMode = lpsMode_TDoA3,
#elif defined(CONFIG_DECK_LOCO_ALGORITHM_TWR)
  .userRequestedMode = lpsMode_TWR,
#else
  .userRequestedMode = lpsMode_auto,
#endif
  // .currentRangingMode is the currently running algorithm, available as a log
  // lpsMode_auto is an impossible mode which forces initialization of the requested mode
  // at startup
  .currentRangingMode = lpsMode_auto,
  .modeAutoSearchActive = true,
  .modeAutoSearchDoInitialize = true,
};

// struct {
//   uwbAlgorithm_t *algorithm;
//   char *name;
// } algorithmsList[LPS_NUMBER_OF_ALGORITHMS + 1] = {
//   [lpsMode_TWR] = {.algorithm = &uwbTwrTagAlgorithm, .name="TWR"},
//   [lpsMode_TDoA2] = {.algorithm = &uwbTdoa2TagAlgorithm, .name="TDoA2"},
//   [lpsMode_TDoA3] = {.algorithm = &uwbTdoa3TagAlgorithm, .name="TDoA3"},
// };

static uwbAlgorithm_t *algorithm = &uwbTdoa2TagAlgorithm;

static bool isInit = false;
static TaskHandle_t uwbTaskHandle = 0;
static SemaphoreHandle_t algoSemaphore;
static dwDevice_t dwm_device;
static dwDevice_t *dwm = &dwm_device;

static QueueHandle_t lppShortQueue;

static uint32_t timeout;


//static void buildAnchorMemList(const uint32_t memAddr, const uint8_t readLen, uint8_t* dest, const uint32_t pageBase_address, const uint8_t anchorCount, const uint8_t unsortedAnchorList[]);

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

// static bool switchToMode(const lpsMode_t newMode) {
//   bool result = false;

//   if (lpsMode_auto != newMode && newMode <= LPS_NUMBER_OF_ALGORITHMS) {
//     algoOptions.currentRangingMode = newMode;
//     algorithm = algorithmsList[algoOptions.currentRangingMode].algorithm;

//     algorithm->init(dwm);
//     timeout = algorithm->onEvent(dwm, eventTimeout);

//     result = true;
//   }

//   return result;
// }

// static void autoModeSearchTryMode(const lpsMode_t newMode, const uint32_t now) {
//   // Set up next time to check
//   algoOptions.nextSwitchTick = now + LPS_AUTO_MODE_SWITCH_PERIOD;
//   switchToMode(newMode);
// }

// static lpsMode_t autoModeSearchGetNextMode() {
//   lpsMode_t newMode = algoOptions.currentRangingMode + 1;
//   if (newMode > LPS_NUMBER_OF_ALGORITHMS) {
//     newMode = lpsMode_TWR;
//   }

//   return newMode;
// }

// static void processAutoModeSwitching() {
//   uint32_t now = xTaskGetTickCount();

//   if (algoOptions.modeAutoSearchActive) {
//     if (algoOptions.modeAutoSearchDoInitialize) {
//       autoModeSearchTryMode(lpsMode_TDoA2, now);
//       algoOptions.modeAutoSearchDoInitialize = false;
//     } else {
//       if (now > algoOptions.nextSwitchTick) {
//         if (algorithm->isRangingOk()) {
//           // We have found an algorithm, stop searching and lock to it.
//           algoOptions.modeAutoSearchActive = false;
//           DEBUG_PRINT("Automatic mode: detected %s\n", algorithmsList[algoOptions.currentRangingMode].name);
//         } else {
//           lpsMode_t newMode = autoModeSearchGetNextMode();
//           autoModeSearchTryMode(newMode, now);
//         }
//       }
//     }
//   }
// }

// static void resetAutoSearchMode() {
//   algoOptions.modeAutoSearchActive = true;
//   algoOptions.modeAutoSearchDoInitialize = true;
// }

// static void handleModeSwitch() {
//   if (algoOptions.userRequestedMode == lpsMode_auto) {
//     processAutoModeSwitching();
//   } else {
//     resetAutoSearchMode();
//     if (algoOptions.userRequestedMode != algoOptions.currentRangingMode) {
//       if (switchToMode(algoOptions.userRequestedMode)) {
//         DEBUG_PRINT("Switching to mode %s\n", algorithmsList[algoOptions.currentRangingMode].name);
//       }
//     }
//   }
// }

static void uwbTask(void* parameters) {
  lppShortQueue = xQueueCreate(10, sizeof(lpsLppShortPacket_t));

  algoOptions.currentRangingMode = lpsMode_auto;

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
      } while(digitalRead(IRQ_PORT_PIN) != 0);
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
static uint16_t spiSpeed = SPI_BAUDRATEPRESCALER_64;

/************ Low level ops for libdw **********/
static void spiWrite(dwDevice_t* dev, const void *header, size_t headerLength,
                                      const void* data, size_t dataLength)
{
  spiBeginTransaction(spiSpeed);
  digitalWrite(CS_PORT_PIN, LOW);
  memcpy(spiTxBuffer, header, headerLength);
  memcpy(spiTxBuffer+headerLength, data, dataLength);
  spiExchange(headerLength+dataLength, spiTxBuffer, spiRxBuffer);
  digitalWrite(CS_PORT_PIN, HIGH);
  spiEndTransaction();
  //STATS_CNT_RATE_EVENT(&spiWriteCount);
}

static void spiRead(dwDevice_t* dev, const void *header, size_t headerLength,
                                     void* data, size_t dataLength)
{
  spiBeginTransaction(spiSpeed);
  digitalWrite(CS_PORT_PIN, LOW);
  memcpy(spiTxBuffer, header, headerLength);
  memset(spiTxBuffer+headerLength, 0, dataLength);
  spiExchange(headerLength+dataLength, spiTxBuffer, spiRxBuffer);
  memcpy(data, spiRxBuffer+headerLength, dataLength);
  digitalWrite(CS_PORT_PIN, HIGH);
  spiEndTransaction();
  //STATS_CNT_RATE_EVENT(&spiReadCount);
}

void __attribute__((used)) EXTI11_Callback(void)
  {
    portBASE_TYPE  xHigherPriorityTaskWoken = pdFALSE;

    // Unlock interrupt handling task
    vTaskNotifyGiveFromISR(uwbTaskHandle, &xHigherPriorityTaskWoken);

    if(xHigherPriorityTaskWoken) {
      portYIELD();
    }
  }

static void spiSetSpeed(dwDevice_t* dev, dwSpiSpeed_t speed)
{
  if (speed == dwSpiSpeedLow)
  {
    spiSpeed = SPI_BAUDRATEPRESCALER_64;
  }
  else if (speed == dwSpiSpeedHigh)
  {
    spiSpeed = SPI_BAUDRATEPRESCALER_4;
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
  EXTI_ConfigTypeDef exti_config;
  EXTI_HandleTypeDef exti_handle;
  spiBegin();

  // Set up interrupt
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE11);
  

  exti_config.Line = EXTI_LINE_11;
  exti_config.Mode = EXTI_MODE_INTERRUPT;
  exti_config.Trigger = EXTI_TRIGGER_RISING;
  exti_handle.Line = EXTI_LINE_11;
  HAL_EXTI_SetConfigLine(&exti_handle, &exti_config);

  // Init pins
  pinMode(CS_PORT_PIN, OUTPUT);
  pinMode(RESET_PORT_PIN, OUTPUT);
  pinMode(IRQ_PORT_PIN, INPUT);

  // Reset the DW1000 chip
  digitalWrite(RESET_PORT_PIN, 0);
  vTaskDelay(M2T(10));
  digitalWrite(RESET_PORT_PIN, 1);
  vTaskDelay(M2T(10));

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

  xTaskCreate(uwbTask, LPS_DECK_TASK_NAME, LPS_DECK_STACKSIZE, NULL,
                    LPS_DECK_TASK_PRI, &uwbTaskHandle);

  isInit = true;
}

uint16_t locoDeckGetRangingState() {
  return algoOptions.rangingState;
}

void locoDeckSetRangingState(const uint16_t newState) {
  algoOptions.rangingState = newState;
}


static bool dwm1000Test()
{
  if (!isInit) {
    DEBUG_PRINT("Error while initializing DWM1000\n");
  }

  return isInit;
}



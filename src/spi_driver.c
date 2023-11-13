
#include <stdbool.h>
#include <stm32f7xx_hal.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "spi_driver.h"


SPI_HandleTypeDef hspi;

// static SemaphoreHandle_t txComplete;
// static SemaphoreHandle_t rxComplete;
static SemaphoreHandle_t txrxComplete;
static SemaphoreHandle_t spiMutex;

static void spiConfigureWithSpeed(uint16_t baudRatePrescaler);

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {

  __GPIOA_CLK_ENABLE();
  __SPI1_CLK_ENABLE();
  __DMA2_CLK_ENABLE();

  GPIO_InitTypeDef Gpio_init_structure;
  Gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  Gpio_init_structure.Speed = GPIO_SPEED_FAST;
  Gpio_init_structure.Pull = GPIO_PULLDOWN;
  Gpio_init_structure.Alternate = GPIO_AF5_SPI1;

  Gpio_init_structure.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOA, &Gpio_init_structure);

  static DMA_HandleTypeDef dma_handle_tx;
  static DMA_HandleTypeDef dma_handle_rx;
  
  dma_handle_tx.Init.Channel = DMA_CHANNEL_3;
  dma_handle_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  dma_handle_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
  dma_handle_tx.Init.MemBurst = DMA_MBURST_SINGLE;
  dma_handle_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  dma_handle_tx.Init.MemInc = DMA_MINC_ENABLE;
  dma_handle_tx.Init.Mode = DMA_NORMAL;
  
  dma_handle_tx.Init.PeriphBurst = DMA_PBURST_SINGLE;
  dma_handle_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  dma_handle_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  dma_handle_tx.Init.Priority = DMA_PRIORITY_HIGH;
  
  // Configure TX DMA
  dma_handle_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  dma_handle_tx.Instance = DMA2_Stream5;

  assert_param(HAL_DMA_Init(&dma_handle_tx) == HAL_OK);

  // Configure RX DMA
  dma_handle_rx.Init = dma_handle_tx.Init;
  dma_handle_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  dma_handle_rx.Instance = DMA2_Stream0;
  assert_param(HAL_DMA_Init(&dma_handle_rx) == HAL_OK);

  __HAL_LINKDMA(hspi, hdmatx, dma_handle_tx);
  __HAL_LINKDMA(hspi, hdmarx, dma_handle_rx);

  // Configure interrupts
  NVIC_SetPriority(DMA2_Stream5_IRQn, 7);
  NVIC_EnableIRQ(DMA2_Stream5_IRQn);

  NVIC_SetPriority(DMA2_Stream0_IRQn, 7);
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);


}


void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi) {
  
  __SPI1_CLK_DISABLE();

  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

  HAL_DMA_DeInit(hspi->hdmarx);
  HAL_DMA_DeInit(hspi->hdmatx);

}

void spiBegin() {
  txrxComplete = xSemaphoreCreateBinary();
  // rxComplete = xSemaphoreCreateBinary();
  spiMutex = xSemaphoreCreateMutex();

  spiConfigureWithSpeed(SPI_BAUDRATE_2MHZ);

}


static void spiConfigureWithSpeed(uint16_t baudRatePrescaler) {
  
  hspi.Init.Direction = SPI_DIRECTION_2LINES;
  hspi.Init.Mode = SPI_MODE_MASTER;
  hspi.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi.Init.NSS = SPI_NSS_SOFT;
  hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi.Init.CRCPolynomial = 0;
  hspi.Init.BaudRatePrescaler = baudRatePrescaler;
  hspi.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi.Init.TIMode = SPI_TIMODE_DISABLE;
  
  hspi.Instance = SPI1;
  HAL_SPI_DeInit(&hspi);
  assert_param(HAL_SPI_Init(&hspi) == HAL_OK);
}

bool spiExchange(size_t length, const uint8_t * data_tx, uint8_t * data_rx) {

  HAL_SPI_TransmitReceive_DMA(&hspi, data_tx, data_rx, (uint16_t) length);
  bool result = (xSemaphoreTake(txrxComplete, portMAX_DELAY) == pdTRUE);

  return result;

}

void spiBeginTransaction(uint16_t baudRatePrescaler)
{
  xSemaphoreTake(spiMutex, portMAX_DELAY);
  spiConfigureWithSpeed(baudRatePrescaler);
}

void spiEndTransaction()
{
  xSemaphoreGive(spiMutex);
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  xSemaphoreGiveFromISR(txrxComplete, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }


}


void DMA2_Stream5_IRQHandler(void) {
  HAL_DMA_IRQHandler(hspi.hdmatx);
}

void DMA2_Stream0_IRQHandler(void) {
  HAL_DMA_IRQHandler(hspi.hdmarx);

}

#include <stdbool.h>
#include <stm32f7xx_hal.h>

#include <dma_spi_utils.h>

#include "FreeRTOS.h"
#include "semphr.h"

#define SPI_BAUDRATE_2MHZ   SPI_BAUDRATEPRESCALER_64

#define SPI_TX_DMA_STREAM       DMA2_Stream5
#define SPI_TX_DMA_IRQ          DMA2_Stream5_IRQn
#define SPI_TX_DMA_IRQHandler   DMA2_Stream5_IRQHandler
#define SPI_TX_DMA_CHANNEL      DMA_Channel_3
#define SPI_TX_DMA_FLAG_TCIF    DMA_FLAG_TCIF1_5

#define SPI_RX_DMA_STREAM       DMA2_Stream0
#define SPI_RX_DMA_IRQ          DMA2_Stream0_IRQn
#define SPI_RX_DMA_IRQHandler   DMA2_Stream0_IRQHandler
#define SPI_RX_DMA_CHANNEL      DMA_Channel_3
#define SPI_RX_DMA_FLAG_TCIF    DMA_FLAG_TCIF0_4


#define SPI_I2S_DMAReq_Tx               ((uint16_t)0x0002)
#define SPI_I2S_DMAReq_Rx               ((uint16_t)0x0001)

static bool isInit = false;

static SemaphoreHandle_t txComplete;
static SemaphoreHandle_t rxComplete;
static SemaphoreHandle_t spiMutex;

static void spiConfigureWithSpeed(uint16_t baudRatePrescaler);
static void spiDMAInit();

void spiBegin() {


    GPIO_InitTypeDef Gpio_init_structure;

    // binary semaphores created using xSemaphoreCreateBinary() are created in a state
    // such that the semaphore must first be 'given' before it can be 'taken'
    txComplete = xSemaphoreCreateBinary();
    rxComplete = xSemaphoreCreateBinary();
    spiMutex = xSemaphoreCreateMutex();

    __SPI1_CLK_ENABLE();

    __GPIOA_CLK_ENABLE();

    __DMA1_CLK_ENABLE();

    Gpio_init_structure.Mode = GPIO_MODE_AF_PP;
    Gpio_init_structure.Speed = GPIO_SPEED_FAST;
    Gpio_init_structure.Pull = GPIO_PULLDOWN;
    Gpio_init_structure.Alternate = GPIO_AF5_SPI1;

    // Gpio_init_structure.Pin = GPIO_PIN_4;
    // HAL_GPIO_Init(GPIOA, &Gpio_init_structure);

    Gpio_init_structure.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOA, &Gpio_init_structure);

    Gpio_init_structure.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOA, &Gpio_init_structure);

    Gpio_init_structure.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &Gpio_init_structure);

    spiDMAInit();

    spiConfigureWithSpeed(SPI_BAUDRATE_2MHZ);

    isInit = true;
}

static void spiDMAInit() {

    DMA_InitTypeDef DMA_init_structure;
    
    DMA_init_structure.FIFOMode = DMA_FIFOMODE_DISABLE;
    DMA_init_structure.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
    DMA_init_structure.MemBurst = DMA_MBURST_SINGLE;
    DMA_init_structure.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    DMA_init_structure.MemInc = DMA_MINC_ENABLE;
    DMA_init_structure.Mode = DMA_NORMAL;
    
    DMA_init_structure.PeriphBurst = DMA_PBURST_SINGLE;
    DMA_init_structure.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    DMA_init_structure.PeriphInc = DMA_PINC_DISABLE;
    DMA_init_structure.Priority = DMA_PRIORITY_HIGH;
    
    // Configure TX DMA
    DMA_init_structure.Channel = DMA_CHANNEL_3;
    DMA_init_structure.Direction = DMA_MEMORY_TO_PERIPH;
    DMA_HandleTypeDef dma_handle_tx;
    dma_handle_tx.Instance = DMA2_Stream5;
    dma_handle_tx.Init = DMA_init_structure;
    assert_param(HAL_DMA_Init(&dma_handle_tx) == HAL_OK);

    // Configure RX DMA
    DMA_init_structure.Channel = DMA_CHANNEL_3;
    DMA_init_structure.Direction = DMA_PERIPH_TO_MEMORY;
    DMA_HandleTypeDef dma_handle_rx;
    dma_handle_rx.Instance = DMA2_Stream0;
    dma_handle_rx.Init = DMA_init_structure;
    assert_param(HAL_DMA_Init(&dma_handle_rx) == HAL_OK);

    // Configure interrupts
    NVIC_SetPriority(DMA2_Stream5_IRQn, 7);
    NVIC_EnableIRQ(DMA2_Stream5_IRQn);

    NVIC_SetPriority(DMA2_Stream0_IRQn, 7);
    NVIC_EnableIRQ(DMA1_Stream0_IRQn);


}

static void spiConfigureWithSpeed(uint16_t baudRatePrescaler) {

    SPI_InitTypeDef spi_init_structure;

    
    SPI_I2S_DeInit(SPI1);
    
    spi_init_structure.Direction = SPI_DIRECTION_2LINES;
    spi_init_structure.Mode = SPI_MODE_MASTER;
    spi_init_structure.DataSize = SPI_DATASIZE_8BIT;
    spi_init_structure.CLKPolarity = SPI_POLARITY_LOW;
    spi_init_structure.CLKPhase = SPI_PHASE_1EDGE;
    spi_init_structure.NSS = SPI_NSS_SOFT;
    spi_init_structure.FirstBit = SPI_FIRSTBIT_MSB;
    spi_init_structure.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi_init_structure.CRCPolynomial = 0;
    spi_init_structure.BaudRatePrescaler = baudRatePrescaler;
    spi_init_structure.NSSPMode = SPI_NSS_PULSE_DISABLE;
    spi_init_structure.TIMode = SPI_TIMODE_DISABLE;
    
    SPI_HandleTypeDef spi_handle;
    spi_handle.Init = spi_init_structure;
    spi_handle.Instance = SPI1;
    assert_param(HAL_SPI_Init(&spi_handle) == HAL_OK);
}

bool spiTest(void)
{
  return isInit;
}

bool spiExchange(size_t length, const uint8_t * data_tx, uint8_t * data_rx) {
  // DMA already configured, just need to set memory addresses
  SPI_TX_DMA_STREAM->M0AR = (uint32_t)data_tx;
  SPI_TX_DMA_STREAM->NDTR = length;
  SPI_TX_DMA_STREAM->PAR = (uint32_t) (&(SPI1->DR)) ;

  SPI_RX_DMA_STREAM->M0AR = (uint32_t)data_rx;
  SPI_RX_DMA_STREAM->NDTR = length;
  SPI_RX_DMA_STREAM->PAR = (uint32_t) (&(SPI1->DR)) ;


  // Enable SPI DMA Interrupts
  DMA_ITConfig(SPI_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_ITConfig(SPI_RX_DMA_STREAM, DMA_IT_TC, ENABLE);

  // Clear DMA Flags
  //DMA_ClearFlag(SPI_TX_DMA_STREAM, DMA_FLAG_FEIF1_5|DMA_FLAG_DMEIF1_5|DMA_FLAG_TEIF1_5|DMA_FLAG_HTIF1_5|DMA_FLAG_TCIF1_5);
  //DMA_ClearFlag(SPI_RX_DMA_STREAM, DMA_FLAG_FEIF0_4|DMA_FLAG_DMEIF0_4|DMA_FLAG_TEIF0_4|DMA_FLAG_HTIF0_4|DMA_FLAG_TCIF0_4);
  clear_flags_start(SPI_TX_DMA_STREAM);
  clear_flags_start(SPI_RX_DMA_STREAM);

  // Enable DMA Streams
  DMA_Cmd(SPI_TX_DMA_STREAM,ENABLE);
  DMA_Cmd(SPI_RX_DMA_STREAM,ENABLE);

  // Enable SPI DMA requests
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);

  // Enable peripheral
  SPI_Cmd(SPI1, ENABLE);

  // Wait for completion
  bool result = (xSemaphoreTake(txComplete, portMAX_DELAY) == pdTRUE)
             && (xSemaphoreTake(rxComplete, portMAX_DELAY) == pdTRUE);

  // Disable peripheral
  SPI_Cmd(SPI1, DISABLE);
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


void __attribute__((used)) DMA2_Stream5_IRQHandler(void) {
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(SPI_TX_DMA_STREAM, DMA_IT_TC, DISABLE);
  //DMA_ClearITPendingBit(SPI_TX_DMA_STREAM, SPI_TX_DMA_FLAG_TCIF);

  // Clear stream flags
  //DMA_ClearFlag(SPI_TX_DMA_STREAM,SPI_TX_DMA_FLAG_TCIF);
  clear_flag_tc(SPI_TX_DMA_STREAM);

  // Disable SPI DMA requests
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);

  // Disable streams
  DMA_Cmd(SPI_TX_DMA_STREAM,DISABLE);

  // Give the semaphore, allowing the SPI transaction to complete
  xSemaphoreGiveFromISR(txComplete, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }
}

void __attribute__((used)) DMA2_Stream0_IRQHandler(void) {
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(SPI_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
  //DMA_ClearITPendingBit(SPI_RX_DMA_STREAM, SPI_RX_DMA_FLAG_TCIF);

  // Clear stream flags
  //DMA_ClearFlag(SPI_RX_DMA_STREAM,SPI_RX_DMA_FLAG_TCIF);
  clear_flag_tc(SPI_RX_DMA_STREAM);

  // Disable SPI DMA requests
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, DISABLE);

  // Disable streams
  DMA_Cmd(SPI_RX_DMA_STREAM,DISABLE);

  // Give the semaphore, allowing the SPI transaction to complete
  xSemaphoreGiveFromISR(rxComplete, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }
}
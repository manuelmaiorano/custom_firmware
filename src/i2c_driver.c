#include <stm32f7xx_hal.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

static SemaphoreHandle_t tx_complete;
static SemaphoreHandle_t rx_complete;


static I2C_HandleTypeDef hi2c;


uint8_t i2c_init(void) {

    tx_complete = xSemaphoreCreateBinary();
    rx_complete = xSemaphoreCreateBinary();

    GPIO_InitTypeDef GPIO_InitStructure;
    // Enable GPIOA clock
    __HAL_RCC_GPIOF_CLK_ENABLE();

    // Enable I2C_SENSORS clock
    __HAL_RCC_I2C2_CLK_ENABLE();

    // Configure I2C_SENSORS pins to unlock bus.
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Alternate = GPIO_AF4_I2C2;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.Pin = GPIO_PIN_1; // SCL
    HAL_GPIO_Init(GPIOF, &GPIO_InitStructure);
    GPIO_InitStructure.Pin =  GPIO_PIN_0; // SDA
    HAL_GPIO_Init(GPIOF, &GPIO_InitStructure);


    static DMA_HandleTypeDef dma_handle_rx;
    static DMA_HandleTypeDef dma_handle_tx;

    __HAL_RCC_DMA1_CLK_ENABLE();

    dma_handle_rx.Init.Channel = DMA_CHANNEL_7;
    //dma_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    dma_handle_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    dma_handle_rx.Init.MemInc = DMA_MINC_ENABLE;
    dma_handle_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dma_handle_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    dma_handle_rx.Init.Mode = DMA_NORMAL;
    dma_handle_rx.Init.Priority = DMA_PRIORITY_HIGH;
    dma_handle_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    dma_handle_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
    dma_handle_rx.Init.MemBurst = DMA_MBURST_SINGLE;
    dma_handle_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;
    //dma_handle.Instance = DMA1_Stream2;

     // Configure rx DMA
    dma_handle_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    dma_handle_rx.Instance = DMA1_Stream2;
    assert_param(HAL_DMA_Init(&dma_handle_rx) == HAL_OK);

    // Configure tx DMA
    dma_handle_tx.Init = dma_handle_rx.Init;
    dma_handle_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    dma_handle_tx.Instance = DMA1_Stream7;
    assert_param(HAL_DMA_Init(&dma_handle_tx) == HAL_OK);

    //hi2c.hdmarx = &dma_handle_rx;
    //hi2c.hdmatx = &dma_handle_tx;
    __HAL_LINKDMA(&hi2c, hdmatx, dma_handle_tx);
    __HAL_LINKDMA(&hi2c, hdmarx, dma_handle_rx);


    NVIC_SetPriority(DMA1_Stream2_IRQn, 7);
    NVIC_EnableIRQ(DMA1_Stream2_IRQn);

    NVIC_SetPriority(DMA1_Stream7_IRQn, 7);
    NVIC_EnableIRQ(DMA1_Stream7_IRQn);

    hi2c.Init.Timing = 0xB0420F13;
    hi2c.Init.OwnAddress1 = 0x00;
    hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c.Init.OwnAddress2 = 0x00;
    hi2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c.Init.NoStretchMode =  I2C_NOSTRETCH_DISABLE;
    hi2c.Instance = I2C2; 

    HAL_I2C_Init(&hi2c);

    NVIC_SetPriority(I2C2_EV_IRQn, 7);
    NVIC_EnableIRQ(I2C2_EV_IRQn);

    NVIC_SetPriority(I2C2_ER_IRQn, 7);
    NVIC_EnableIRQ(I2C2_ER_IRQn);

    assert_param(HAL_I2C_IsDeviceReady(&hi2c, 0xD0, 3, 10) == HAL_OK);
    return 0;

}

uint8_t i2c_deinit() {
    __I2C2_FORCE_RESET();
    __I2C2_RELEASE_RESET();

    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_1);
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_0);

    HAL_DMA_DeInit(hi2c.hdmarx);
    HAL_DMA_DeInit(hi2c.hdmatx);

    NVIC_DisableIRQ(DMA1_Stream2_IRQn);
    NVIC_DisableIRQ(DMA1_Stream7_IRQn);

    NVIC_DisableIRQ(I2C2_EV_IRQn);
    NVIC_DisableIRQ(I2C2_ER_IRQn);

    assert_param(HAL_I2C_DeInit(&hi2c) == HAL_OK);
    return 0;
}

uint8_t i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {

    assert_param(HAL_I2C_Mem_Read_DMA(&hi2c, (uint16_t) addr, (uint16_t) reg, 1, buf, len) == HAL_OK);
    if (xSemaphoreTake(rx_complete, 1000) == pdTRUE) {
        return 0;
    }
    return 1;

    // assert_param(HAL_I2C_Mem_Read(&hi2c, (uint16_t) addr, (uint16_t) reg, 1, buf, len, 1000) == HAL_OK);
    // return 0;

}

uint8_t i2c_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {

    assert_param(HAL_I2C_Mem_Write_DMA(&hi2c, (uint16_t) addr, (uint16_t) reg, 1, buf, len) == HAL_OK);
    if (xSemaphoreTake(tx_complete, 1000) == pdTRUE) {
        return 0;
    }
    return 1;

    // assert_param(HAL_I2C_Mem_Write(&hi2c, (uint16_t) addr, (uint16_t) reg, 1, buf, len, 1000) == HAL_OK);
    // return 0;

}


void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c) {

    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(tx_complete, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken){
        portYIELD();
    }
  
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c) {

    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(rx_complete, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken){
        portYIELD();
    }
}


void I2C2_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&hi2c);
}

void I2C2_ER_IRQHandler(void)
{
  HAL_I2C_ER_IRQHandler(&hi2c);
}

void DMA1_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hi2c.hdmarx);
}

void DMA1_Stream7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hi2c.hdmatx);
}

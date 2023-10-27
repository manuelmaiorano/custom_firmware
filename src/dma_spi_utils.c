#include "dma_spi_utils.h"

typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;

#define TRANSFER_IT_ENABLE_MASK (uint32_t)(DMA_SxCR_TCIE | DMA_SxCR_HTIE | \
                                           DMA_SxCR_TEIE | DMA_SxCR_DMEIE)
#define TRANSFER_IT_MASK        (uint32_t)0x0F3C0F3C
#define HIGH_ISR_MASK           (uint32_t)0x20000000
#define RESERVED_MASK           (uint32_t)0x0F7D0F7D  

static uint32_t calc_base_address(DMA_Stream_TypeDef* Instance);
static uint32_t calc_stream_index(DMA_Stream_TypeDef* Instance);

void SPI_I2S_DMACmd(SPI_TypeDef* SPIx, uint16_t SPI_I2S_DMAReq, FunctionalState NewState)
{

  if (NewState != DISABLE)
  {
    /* Enable the selected SPI DMA requests */
    SPIx->CR2 |= SPI_I2S_DMAReq;
  }
  else
  {
    /* Disable the selected SPI DMA requests */
    SPIx->CR2 &= (uint16_t)~SPI_I2S_DMAReq;
  }
}

void DMA_Cmd(DMA_Stream_TypeDef* DMAy_Streamx, FunctionalState NewState)
{

  if (NewState != DISABLE)
  {
    /* Enable the selected DMAy Streamx by setting EN bit */
    DMAy_Streamx->CR |= (uint32_t)DMA_SxCR_EN;
  }
  else
  {
    /* Disable the selected DMAy Streamx by clearing EN bit */
    DMAy_Streamx->CR &= ~(uint32_t)DMA_SxCR_EN;
  }
}

void DMA_ClearFlag(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_FLAG)
{
  DMA_TypeDef* DMAy;

  /* Determine the DMA to which belongs the stream */
  if (DMAy_Streamx < DMA2_Stream0)
  {
    /* DMAy_Streamx belongs to DMA1 */
    DMAy = DMA1; 
  } 
  else 
  {
    /* DMAy_Streamx belongs to DMA2 */
    DMAy = DMA2; 
  }

  /* Check if LIFCR or HIFCR register is targeted */
  if ((DMA_FLAG & HIGH_ISR_MASK) != (uint32_t)RESET)
  {
    /* Set DMAy HIFCR register clear flag bits */
    DMAy->HIFCR = (uint32_t)(DMA_FLAG & RESERVED_MASK);
  }
  else 
  {
    /* Set DMAy LIFCR register clear flag bits */
    DMAy->LIFCR = (uint32_t)(DMA_FLAG & RESERVED_MASK);
  }    
}

void DMA_ClearITPendingBit(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_IT)
{
  DMA_TypeDef* DMAy;

  /* Determine the DMA to which belongs the stream */
  if (DMAy_Streamx < DMA2_Stream0)
  {
    /* DMAy_Streamx belongs to DMA1 */
    DMAy = DMA1; 
  } 
  else 
  {
    /* DMAy_Streamx belongs to DMA2 */
    DMAy = DMA2; 
  }

  /* Check if LIFCR or HIFCR register is targeted */
  if ((DMA_IT & HIGH_ISR_MASK) != (uint32_t)RESET)
  {
    /* Set DMAy HIFCR register clear interrupt bits */
    DMAy->HIFCR = (uint32_t)(DMA_IT & RESERVED_MASK);
  }
  else 
  {
    /* Set DMAy LIFCR register clear interrupt bits */
    DMAy->LIFCR = (uint32_t)(DMA_IT & RESERVED_MASK);
  }   
}

void DMA_ITConfig(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_IT, FunctionalState NewState)
{

  /* Check if the DMA_IT parameter contains a FIFO interrupt */
  if ((DMA_IT & DMA_IT_FE) != 0)
  {
    if (NewState != DISABLE)
    {
      /* Enable the selected DMA FIFO interrupts */
      DMAy_Streamx->FCR |= (uint32_t)DMA_IT_FE;
    }    
    else 
    {
      /* Disable the selected DMA FIFO interrupts */
      DMAy_Streamx->FCR &= ~(uint32_t)DMA_IT_FE;  
    }
  }

  /* Check if the DMA_IT parameter contains a Transfer interrupt */
  if (DMA_IT != DMA_IT_FE)
  {
    if (NewState != DISABLE)
    {
      /* Enable the selected DMA transfer interrupts */
      DMAy_Streamx->CR |= (uint32_t)(DMA_IT  & TRANSFER_IT_ENABLE_MASK);
    }
    else
    {
      /* Disable the selected DMA transfer interrupts */
      DMAy_Streamx->CR &= ~(uint32_t)(DMA_IT & TRANSFER_IT_ENABLE_MASK);
    }    
  }
}

void SPI_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    /* Enable the selected SPI peripheral */
    SPIx->CR1 |= SPI_CR1_SPE;
  }
  else
  {
    /* Disable the selected SPI peripheral */
    SPIx->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE);
  }
}

void SPI_I2S_DeInit(SPI_TypeDef* SPIx){

  if (SPIx == SPI1) {
    /* Enable SPI1 reset state */
    __HAL_RCC_SPI1_FORCE_RESET();
    /* Release SPI1 from reset state */
    __HAL_RCC_SPI1_RELEASE_RESET();
  }
}

void clear_flags_start(DMA_Stream_TypeDef* Instance) {
  
  uint32_t base_address = calc_base_address(Instance);
  DMA_Base_Registers *regs = (DMA_Base_Registers *) base_address;
  regs->IFCR = 0x3FU << calc_stream_index(Instance);

}

void clear_flag_tc(DMA_Stream_TypeDef* Instance) {
  uint32_t base_address = calc_base_address(Instance);
  DMA_Base_Registers *regs = (DMA_Base_Registers *) base_address;
  regs->IFCR = DMA_FLAG_TCIF0_4 << calc_stream_index(Instance);

}


static uint32_t calc_base_address(DMA_Stream_TypeDef* Instance)
{
  uint32_t stream_number = (((uint32_t)Instance & 0xFFU) - 16U) / 24U;
  uint32_t StreamBaseAddress;
  
  if (stream_number > 3U)
  {
    /* return pointer to HISR and HIFCR */
    StreamBaseAddress = (((uint32_t)Instance & (uint32_t)(~0x3FFU)) + 4U);
  }
  else
  {
    /* return pointer to LISR and LIFCR */
    StreamBaseAddress = ((uint32_t)Instance & (uint32_t)(~0x3FFU));
  }
  
  return StreamBaseAddress;
}

static uint32_t calc_stream_index(DMA_Stream_TypeDef* Instance) {
  uint32_t stream_number = (((uint32_t)Instance & 0xFFU) - 16U) / 24U;
  static const uint8_t flagBitshiftOffset[8U] = {0U, 6U, 16U, 22U, 0U, 6U, 16U, 22U};
  return flagBitshiftOffset[stream_number];
}

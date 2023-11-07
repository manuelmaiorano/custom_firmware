
#ifndef __CUSTOM_SPI_DRIVER_H__
#define __CUSTOM_SPI_DRIVER_H__

void spiBegin();
bool spiExchange(size_t length, const uint8_t * data_tx, uint8_t * data_rx);
void spiBeginTransaction(uint16_t baudRatePrescaler);
void spiEndTransaction();

// Based on 108MHz peripheral clock
#define SPI_BAUDRATE_21MHZ  SPI_BAUDRATEPRESCALER_8 // 27MHz
#define SPI_BAUDRATE_12MHZ  SPI_BAUDRATEPRESCALER_8     // 13.5MHz
#define SPI_BAUDRATE_6MHZ   SPI_BAUDRATEPRESCALER_16    // 6.75MHz
#define SPI_BAUDRATE_3MHZ   SPI_BAUDRATEPRESCALER_32    // 3.375MHz
#define SPI_BAUDRATE_2MHZ   SPI_BAUDRATEPRESCALER_64 // 1.6875MHz

#endif
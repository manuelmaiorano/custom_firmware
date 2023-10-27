
#ifndef __CUSTOM_SPI_DRIVER_H__
#define __CUSTOM_SPI_DRIVER_H__

void spiBegin();
bool spiExchange(size_t length, const uint8_t * data_tx, uint8_t * data_rx);
void spiBeginTransaction(uint16_t baudRatePrescaler);
void spiEndTransaction();

#endif
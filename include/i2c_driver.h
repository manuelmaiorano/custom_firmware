#include <stm32f7xx_hal.h>

void i2c_init(void);

void i2c_deinit(void);

void i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

void i2c_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

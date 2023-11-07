#include <stm32f7xx_hal.h>

uint8_t i2c_init(void);

uint8_t i2c_deinit(void);

uint8_t i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

uint8_t i2c_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

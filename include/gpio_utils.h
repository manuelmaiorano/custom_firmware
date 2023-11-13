#ifndef __DECK_UTILS_H__
#define __DECK_UTILS_H__

#include <stdint.h>
#include <stm32f7xx_hal.h>

#define LOW 0x0
#define HIGH 0x1

typedef const struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} gpio_port_pin_t;

void digitalWrite(const gpio_port_pin_t port_pin,  const uint32_t val);

int digitalRead(const gpio_port_pin_t port_pin);

#endif

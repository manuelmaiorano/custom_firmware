#ifndef __DECK_UTILS_H__
#define __DECK_UTILS_H__

#include <stdint.h>
#include <stm32f7xx_hal_gpio.h>

#define LOW 0x0
#define HIGH 0x1

#define INPUT           0x0
#define OUTPUT          0x1
#define INPUT_PULLUP    0x2
#define INPUT_PULLDOWN  0x3

typedef const struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} gpio_port_pin_t;

void pinMode(const gpio_port_pin_t port_pin, const uint32_t mode);

void digitalWrite(const gpio_port_pin_t port_pin,  const uint32_t val);

int digitalRead(const gpio_port_pin_t port_pin);

#endif

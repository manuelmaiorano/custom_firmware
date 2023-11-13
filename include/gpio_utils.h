#ifndef __DECK_UTILS_H__
#define __DECK_UTILS_H__

typedef const struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} gpio_port_pin_t;

#endif

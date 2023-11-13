#include "gpio_utils.h"
#include <stm32f7xx_hal_gpio.h>

void digitalWrite(const gpio_port_pin_t port_pin, const uint32_t val)
{
  GPIO_PinState action = GPIO_PIN_RESET;
  if (val) {
    action = GPIO_PIN_SET;
  }
  HAL_GPIO_WritePin(port_pin.port, port_pin.pin, action);

}

int digitalRead(const gpio_port_pin_t port_pin)
{
  int val = HAL_GPIO_ReadPin(port_pin.port, port_pin.pin);
  return (val==GPIO_PIN_SET)?HIGH:LOW;
}

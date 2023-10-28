#include "gpio_utils.h"
#include <stm32f7xx_hal_gpio.h>


void pinMode(const gpio_port_pin_t port_pin,  const uint32_t mode)
{
  //RCC_AHB1PeriphClockCmd(deckGPIOMapping[pin.id].periph, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure = {0};

  GPIO_InitStructure.Pin = port_pin.pin;
  GPIO_InitStructure.Mode = (mode == OUTPUT) ? GPIO_MODE_OUTPUT_PP:GPIO_MODE_INPUT;
  if (mode == INPUT_PULLUP) GPIO_InitStructure.Pull = GPIO_PULLUP;
  if (mode == INPUT_PULLDOWN) GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(port_pin.port, &GPIO_InitStructure);
}

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

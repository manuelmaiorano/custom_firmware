
#include "exti_utils.h"

ITStatus EXTI_GetITStatus(uint32_t EXTI_Line)
{
  FlagStatus bitstatus = RESET;
  
  if ((EXTI->PR & EXTI_Line) != (uint32_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
  
}
BIN = output                     # Name of the output binary
SDK_DIR = ./mcu-vendor         # Target-specific library

INC = -I$/include
INC += -I$/vendor/FreeRTOS-Kernel/include
INC += -I$/vendor/FreeRTOS-Kernel/portable/GCC/ARM_CM7/r0p1
INC += -I$/vendor/libdw1000/inc
INC += -I$/Drivers/STM32F7xx_HAL_Driver/Inc
INC += -I$/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy
INC += -I$/Drivers/CMSIS/Include
INC += -I$/Drivers/CMSIS/Device/ST/STM32F7xx
INC += -I$/Drivers/CMSIS/Device/ST/STM32F7xx/Include

SRC_ASM = ./startup/startup_stm32f767xx.s

SCR_C = src/main.c
SRC_C += src/estimator.c
SRC_C += src/tdoaEngine.c
SRC_C += src/locodeck.c
SRC_C += src/clockCorrectionEngine.c
SRC_C += src/dma_spi_utils.c
SRC_C += src/spi_driver.c
SRC_C += src/lpsTdoa2Tag.c
SRC_C += src/tdoaStorage.c
SRC_C += src/tdoaEngineInstance.c
SRC_C += src/gpio_utils.c
# FreeRTOS
SCR_C += ./vendor/FreeRTOS-Kernel/event_groups.c
SCR_C += ./vendor/FreeRTOS-Kernel/list.c
SCR_C += ./vendor/FreeRTOS-Kernel/portable/GCC/ARM_CM7/port.c
SCR_C += ./vendor/FreeRTOS-Kernel/portable/MemMang/heap_4.c
SCR_C += ./vendor/FreeRTOS-Kernel/queue.c
SCR_C += ./vendor/FreeRTOS-Kernel/tasks.c
SCR_C += ./vendor/FreeRTOS-Kernel/timers.c
SCR_C += ./vendor/FreeRTOS-Kernel/stream_buffer.c
# libdw1000
SCR_C += ./vendor/libdw1000/src/libdw1000.c
SCR_C += ./vendor/libdw1000/src/libdw1000Spi.c
# HAL
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart_ex.c
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash.c
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd.c
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim_ex.c
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim.c
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd_ex.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_cortex.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_spi_ex.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc_ex.c
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c_ex.c
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_eth.c
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_gpio.c
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash_ex.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc.c
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart.c
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr_ex.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_exti.c
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_usb.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_spi.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma_ex.c


SRC_LD = ./STM32F767ZI_FLASH.ld

ARCHFLAGS = -mcpu=cortex-m7
FPFLAGS = -mfloat-abi=hard -mfpu=fpv5-d16 


OPENOCD = openocd -f interface/stlink-v2-1.cfg -f target/stm32f7x.cfg

include cm-makefile/config.mk
include cm-makefile/openocd.mk  # For flash, debug and gdb targets
include cm-makefile/rules.mk
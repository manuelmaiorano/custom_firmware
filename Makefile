BIN = output# Name of the output binary
V = 1

INC = -I$(SRC_DIR)/include
INC += -I$(SRC_DIR)/Third_Party/FreeRTOS/Source/include
INC += -I$(SRC_DIR)/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1
INC += -I$(SRC_DIR)/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
INC += -I$(SRC_DIR)/vendor/libdw1000/inc
INC += -I$(SRC_DIR)/Drivers/STM32F7xx_HAL_Driver/Inc
INC += -I$(SRC_DIR)/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy
INC += -I$(SRC_DIR)/Drivers/CMSIS/Include
INC += -I$(SRC_DIR)/Drivers/CMSIS/Device/ST/STM32F7xx
INC += -I$(SRC_DIR)/Drivers/CMSIS/Device/ST/STM32F7xx/Include
INC += -I$(SRC_DIR)/Third_Party/SEGGER
INC += -I$(SRC_DIR)/CMSIS-DSP/Include
INC += -I$(SRC_DIR)/CMSIS-DSP/PrivateInclude

SRC_ASM = startup/startup_stm32f767xx.s

SRC_C = src/main.c
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
SRC_C += src/system_stm32f7xx.c
SRC_C += src/stm32f7xx_it.c
SRC_C += src/syscalls.c
SRC_C += src/stm32f7xx_hal_msp.c
SRC_C += src/exti_utils.c
# FreeRTOS
SRC_C += Third_Party/FreeRTOS/Source/event_groups.c
SRC_C += Third_Party/FreeRTOS/Source/list.c
SRC_C += Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/port.c
SRC_C += Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
SRC_C += Third_Party/FreeRTOS/Source/queue.c
SRC_C += Third_Party/FreeRTOS/Source/tasks.c
SRC_C += Third_Party/FreeRTOS/Source/timers.c
SRC_C += Third_Party/FreeRTOS/Source/stream_buffer.c
SRC_C += Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
#SEGGER

# libdw1000
SRC_C += vendor/libdw1000/src/libdw1000.c
SRC_C += vendor/libdw1000/src/libdw1000Spi.c
# HAL
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart_ex.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash.c
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd.c
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim_ex.c
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim.c
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd_ex.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_cortex.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_spi_ex.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc_ex.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c_ex.c
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_eth.c
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_gpio.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash_ex.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc.c
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr_ex.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_exti.c
#SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_usb.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_spi.c
SRC_C += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma_ex.c

SRC_ASM += Third_Party/SEGGER/SEGGER_RTT_ASM_ARMv7M.S
SRC_C += Third_Party/SEGGER/SEGGER_RTT_Syscalls_IAR.c
SRC_C += Third_Party/SEGGER/SEGGER_SYSVIEW.c
SRC_C += Third_Party/SEGGER/SEGGER_SYSVIEW_Config_FreeRTOS.c
SRC_C += Third_Party/SEGGER/SEGGER_RTT_printf.c
SRC_C += Third_Party/SEGGER/SEGGER_RTT.c
SRC_C += Third_Party/SEGGER/SEGGER_RTT_Syscalls_SES.c
SRC_C += Third_Party/SEGGER/SEGGER_RTT_Syscalls_KEIL.c
SRC_C += Third_Party/SEGGER/SEGGER_RTT_Syscalls_GCC.c
SRC_C += Third_Party/SEGGER/SEGGER_SYSVIEW_FreeRTOS.c

SRC_C += CMSIS-DSP/Source/BasicMathFunctions/BasicMathFunctions.c
SRC_C += CMSIS-DSP/Source/BasicMathFunctions/BasicMathFunctionsF16.c
SRC_C += CMSIS-DSP/Source/MatrixFunctions/MatrixFunctions.c
SRC_C += CMSIS-DSP/Source/MatrixFunctions/MatrixFunctionsF16.c

SRC_LD = STM32F767ZI_FLASH.ld

ARCHFLAGS = -mcpu=cortex-m7
FPFLAGS = -mfloat-abi=hard -mfpu=fpv5-d16 
DEF = -DSTM32F767xx -DUSE_HAL_DRIVER -DUSE_FULL_ASSERT=1

OPENOCD = openocd -f interface/stlink-v2-1.cfg -f target/stm32f7x.cfg


include cm-makefile/config.mk
include cm-makefile/openocd.mk  # For flash, debug and gdb targets
include cm-makefile/rules.mk
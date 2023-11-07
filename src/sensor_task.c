#include "FreeRTOS.h"
#include "task.h"
#include "driver_mpu6050_basic.h"
#include <stm32f7xx_hal.h>

uint8_t res;
uint32_t i;
uint32_t times;
float g[3];
float dps[3];
float degrees;
mpu6050_address_t addr;


TaskHandle_t sensortask_handle;


void sensor_task(void* param);


void sensor_task_init() {

    assert_param(xTaskCreate(sensor_task, "sens", 4*configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &sensortask_handle) == pdPASS);
}

void sensor_task(void* param) {

    addr = MPU6050_ADDRESS_AD0_LOW;
    res = mpu6050_basic_init(addr);
    assert_param(res == 0);

    while(1) {
        vTaskDelay(100);
        if (mpu6050_basic_read(g, dps) != 0){
            (void)mpu6050_basic_deinit();

            assert_param(0);
        }

    }
}
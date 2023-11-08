#include "FreeRTOS.h"
#include "task.h"
#include "driver_mpu6050_basic.h"
#include <SEGGER_SYSVIEW.h>
#include <stm32f7xx_hal.h>
#include "estimator.h"
#include "stabilizer_types.h"

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

    measurement_t measurement;
    Axis3f vec;

    while(1) {
        vTaskDelay(100);
        if (mpu6050_basic_read(g, dps) != 0){
            (void)mpu6050_basic_deinit();

            assert_param(0);
        }
        //SEGGER_SYSVIEW_PrintfHost("acc: %x, %x, %x", g[0], g[1], g[2]);
        //SEGGER_SYSVIEW_PrintfHost("gyro: %x, %x, %x", dps[0], dps[1], dps[2]);
        
        measurement.type = MeasurementTypeAcceleration;
        vec.x = g[0]; 
        vec.y = g[1];
        vec.z = g[2];
        measurement.data.acceleration.acc = vec;
        estimatorEnqueue(&measurement); 

        measurement.type = MeasurementTypeGyroscope;
        vec.x = dps[0]; 
        vec.y = dps[1];
        vec.z = dps[2];
        measurement.data.gyroscope.gyro = vec;
        estimatorEnqueue(&measurement); 

    }
}
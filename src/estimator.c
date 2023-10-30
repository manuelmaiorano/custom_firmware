#include "estimator.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "config.h"
#include <SEGGER_SYSVIEW.h>
#include <stm32f7xx_hal.h>
#include "locodeck.h"
#include "gpio_utils.h"



#define MEASUREMENTS_QUEUE_SIZE (21)
static xQueueHandle measurementsQueue;
static StaticQueue_t queue_structure;
static tdoaMeasurement_t queue_array[MEASUREMENTS_QUEUE_SIZE];

static TaskHandle_t task_handle = 0;

static void kalmanTask(void* parameters);
static void updateQueuedMeasurements(const uint32_t nowMs);

void kalman_init() {

    measurementsQueue = xQueueCreateStatic(MEASUREMENTS_QUEUE_SIZE, sizeof(tdoaMeasurement_t), queue_array, &queue_structure);

    assert_param(xTaskCreate(kalmanTask, KALMAN_TASK_NAME, KALMAN_TASK_STACKSIZE, NULL,
                    KALMAN_TASK_PRI, &task_handle) == pdPASS);

}
void estimatorEnqueueTDOA(const tdoaMeasurement_t *measurement) {


    portBASE_TYPE result = xQueueSend(measurementsQueue, measurement, 0);

}

bool estimatorDequeue(tdoaMeasurement_t *measurement) {
  return pdTRUE == xQueueReceive(measurementsQueue, measurement, portMAX_DELAY);
}

static void kalmanTask(void* parameters) {

    init_loco_deck();

    //gpio_port_pin_t IRQ_PORT_PIN = {.port=GPIOC, .pin=GPIO_PIN_11 };
    //digitalWrite(IRQ_PORT_PIN, 1);

    uint32_t nowMs = T2M(xTaskGetTickCount());
    uint32_t nextPredictionMs = nowMs;

    while (true) {
        nowMs = T2M(xTaskGetTickCount()); 

        updateQueuedMeasurements(nowMs);

    }



}

static void updateQueuedMeasurements(const uint32_t nowMs) {
    tdoaMeasurement_t m;
    while (estimatorDequeue(&m)) {
        //
        SEGGER_SYSVIEW_PrintfHost("%f", m.distanceDiff);
    }

}
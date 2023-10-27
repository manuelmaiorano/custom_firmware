#include "estimator.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "config.h"
#include <SEGGER_SYSVIEW.h>


#define MEASUREMENTS_QUEUE_SIZE (20)
static xQueueHandle measurementsQueue;
static StaticQueue_t queue_structure;
static tdoaMeasurement_t queue_array[MEASUREMENTS_QUEUE_SIZE];

static TaskHandle_t task_handle = 0;

static void kalmanTask(void* parameters);
static void updateQueuedMeasurements(const uint32_t nowMs);

void kalman_init() {

    measurementsQueue = xQueueCreateStatic(MEASUREMENTS_QUEUE_SIZE, sizeof(tdoaMeasurement_t), queue_array, &queue_structure);

    xTaskCreate(kalmanTask, KALMAN_TASK_NAME, KALMAN_TASK_STACKSIZE, NULL,
                    KALMAN_TASK_PRI, &task_handle);

}
void estimatorEnqueueTDOA(const tdoaMeasurement_t *measurement) {


    portBASE_TYPE result = xQueueSend(measurementsQueue, measurement, 0);

}

bool estimatorDequeue(tdoaMeasurement_t *measurement) {
  return pdTRUE == xQueueReceive(measurementsQueue, measurement, 0);
}

static void kalmanTask(void* parameters) {

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
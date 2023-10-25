#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION		1
#define configUSE_IDLE_HOOK			1
#define configUSE_TICK_HOOK			0
#define configCPU_CLOCK_HZ			( ( unsigned long ) FREERTOS_MCU_CLOCK_HZ )
#define configTICK_RATE_HZ_RAW  1000
#define configTICK_RATE_HZ			( ( portTickType ) configTICK_RATE_HZ_RAW )
#define configMINIMAL_STACK_SIZE	( ( unsigned short ) FREERTOS_MIN_STACK_SIZE )
#define configTOTAL_HEAP_SIZE		( ( size_t ) ( FREERTOS_HEAP_SIZE ) )
#define configMAX_TASK_NAME_LEN		( 10 )
#define configUSE_16_BIT_TICKS		0
#define configIDLE_SHOULD_YIELD		0
#define configUSE_CO_ROUTINES 		0
#define configCHECK_FOR_STACK_OVERFLOW      1
#define configUSE_TASK_NOTIFICATIONS 1
#define configUSE_TIMERS          1
#define configTIMER_TASK_PRIORITY 1
#define configTIMER_QUEUE_LENGTH  20
#define configUSE_MALLOC_FAILED_HOOK 1
#define configTIMER_TASK_STACK_DEPTH (configMINIMAL_STACK_SIZE * 4)

#define configMAX_PRIORITIES		( 6 )
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet		1
#define INCLUDE_uxTaskPriorityGet		1
#define INCLUDE_vTaskDelete				1
#define INCLUDE_vTaskCleanUpResources	1
#define INCLUDE_vTaskSuspend			1
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay				1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_xTaskGetIdleTaskHandle 1
#define INCLUDE_xTimerPendFunctionCall 1

#define configUSE_MUTEXES 1

#define configKERNEL_INTERRUPT_PRIORITY     255
//#define configMAX_SYSCALL_INTERRUPT_PRIORITY 1
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 0x5F /* equivalent to 0x05, or priority 5. */

//Map the port handler to the crt0 interruptions handlers
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler tickFreeRTOS
#define vPortSVCHandler SVC_Handler

//Milliseconds to OS Ticks
#if configTICK_RATE_HZ_RAW != 1000
  #error "Please review the use of M2T and T2M if there is not a 1 to 1 mapping between ticks and milliseconds"
#endif
#define M2T(X) ((unsigned int)(X))
#define F2T(X) ((unsigned int)((configTICK_RATE_HZ/(X))))
#define T2M(X) ((unsigned int)(X))

// Seconds to OS ticks
#define S2T(X) ((portTickType)((X) * configTICK_RATE_HZ))
#define T2S(X) ((X) / (float)configTICK_RATE_HZ)


// DEBUG SECTION
#define configUSE_APPLICATION_TASK_TAG  1
#define configQUEUE_REGISTRY_SIZE       10

#ifdef DEBUG
#define configRECORD_STACK_HIGH_ADDRESS 1
#endif

#define TASK_LED_ID_NBR         1
#define TASK_RADIO_ID_NBR       2
#define TASK_STABILIZER_ID_NBR  3
#define TASK_ADC_ID_NBR         4
#define TASK_PM_ID_NBR          5
#define TASK_PROXIMITY_ID_NBR   6

#define configASSERT( x )  if( ( x ) == 0 ) assertFail(#x, __FILE__, __LINE__ )

/*
#define traceTASK_SWITCHED_IN() \
  { \
    extern void debugSendTraceInfo(unsigned int taskNbr); \
    debugSendTraceInfo((int)pxCurrentTCB->pxTaskTag); \
  }
*/

#define configSUPPORT_STATIC_ALLOCATION 1

// Queue monitoring
#ifdef CONFIG_DEBUG_QUEUE_MONITOR
    #undef traceQUEUE_SEND
    #undef traceQUEUE_SEND_FAILED
    #define traceQUEUE_SEND(xQueue) qm_traceQUEUE_SEND(xQueue)
    void qm_traceQUEUE_SEND(void* xQueue);
    #define traceQUEUE_SEND_FAILED(xQueue) qm_traceQUEUE_SEND_FAILED(xQueue)
    void qm_traceQUEUE_SEND_FAILED(void* xQueue);
#endif // CONFIG_DEBUG_QUEUE_MONITOR

#endif /* FREERTOS_CONFIG_H */

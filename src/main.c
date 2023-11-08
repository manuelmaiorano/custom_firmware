#include <FreeRTOS.h>
#include <task.h>
#include <stm32f7xx_hal.h>
#include <SEGGER_SYSVIEW.h>
#include "gpio_utils.h"
#include "sensor_task.h"



#include "locodeck.h"
#include "estimator.h"

TaskHandle_t ledtask_handle;
TaskHandle_t controllertask_handle;


void led_task(void* param);
void mock_controller_task(void* param);

int main(void) {

    HAL_Init();
    SystemClock_Config();

	SEGGER_SYSVIEW_Conf();
    NVIC_SetPriorityGrouping(0);


    kalman_init();
	sensor_task_init();

	assert_param(xTaskCreate(led_task, "ledtask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &ledtask_handle) == pdPASS);

	assert_param(xTaskCreate(mock_controller_task, "controller", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &controllertask_handle) == pdPASS);


    //start the scheduler - shouldn't return unless there's a problem
	vTaskStartScheduler();

	//if you've wound up here, there is likely an issue with overrunning the freeRTOS heap
	while(1)
	{
	}

}

void mock_controller_task(void* param) {

	uint32_t tick_period = 10;
	state_t state;

	while (1)
	{
		vTaskDelay(tick_period);
		estimatorKalman(&state, 0);
	}
	
}

void led_task(void* param) {

	__GPIOB_CLK_ENABLE();
	const gpio_port_pin_t ledgpio = {.port=GPIOB, .pin=GPIO_PIN_0};
	EXTI_HandleTypeDef handle;
	handle.Line = EXTI_LINE_11;
	pinMode(ledgpio, OUTPUT);

	while (1)
	{
		SEGGER_SYSVIEW_PrintfHost("LedTaskRunning\n");
		digitalWrite(ledgpio, 1);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		digitalWrite(ledgpio, 0);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		//HAL_EXTI_GenerateSWI(&handle);
	}
	
}


void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Configure LSE Drive Capability
	*/
	HAL_PWR_EnableBkUpAccess();
	/** Configure the main internal regulator output voltage
	*/
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 216;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 9;
	RCC_OscInitStruct.PLL.PLLR = 2;	//NOTE: this line was not supplied by HAL - it simply
									//sets the struct to match MCU defaults
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	*/
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
	{
		Error_Handler();
	}
	//PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
	//PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	// PeriphClkInitStruct.PeriphClockSelection =	RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48|
	// 											RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_UART4;
	// PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
	// PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	// PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_SYSCLK;
	// PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
	// if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	// {
	// 	Error_Handler();
	// }
}


void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
	SEGGER_SYSVIEW_PrintfHost("Assertion Failed:file %s \
                            on line %d\r\n", file, line);
  	while(1);
}
#endif
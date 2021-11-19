#include "main.h"
#include "board_api.h"

#define I2C_ADDRESS				0x7f
#define I2C_TIMING				0x00201D2C

#define  PULSE1_VALUE			(uint32_t)(PERIOD_VALUE/2)        /* Capture Compare 1 Value  */
#define  PULSE2_VALUE			(uint32_t)(PERIOD_VALUE*37.5/100) /* Capture Compare 2 Value  */
#define  PULSE3_VALUE			(uint32_t)(PERIOD_VALUE/4)        /* Capture Compare 3 Value  */
#define  PULSE4_VALUE			(uint32_t)(PERIOD_VALUE*12.5/100) /* Capture Compare 4 Value  */

I2C_HandleTypeDef  I2cHandle;

TIM_HandleTypeDef  TimHandle;
TIM_OC_InitTypeDef sConfig;

GPIO_InitTypeDef  GPIO_InitStruct;

uint32_t uhPrescalerValue = 0;

/*****************************************************************************/

uint8_t rxBuff[BOARD_TRANSFER_CHUNK];
uint8_t txBuff[BOARD_TRANSFER_CHUNK];

/*****************************************************************************/
void SystemClock_Config(void);

int main(void) {
	HAL_Init();

	/* Configure the system clock to 72 MHz */
	SystemClock_Config();

	/* Configure LED3, LED4, LED5 and LED6 */
	BSP_LED_Init(LED3);
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
	BSP_LED_Init(LED6);

	/*************************************************************************/

	I2cHandle.Instance             = I2Cx;

	I2cHandle.Init.Timing          = I2C_TIMING;
	I2cHandle.Init.OwnAddress1     = I2C_ADDRESS;
	I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2cHandle.Init.OwnAddress2     = 0xFF;
	I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_ENABLE;

	if(HAL_I2C_Init(&I2cHandle) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	HAL_I2CEx_ConfigAnalogFilter(&I2cHandle,I2C_ANALOGFILTER_ENABLE);

	/*************************************************************************/

	uhPrescalerValue = (uint32_t)(SystemCoreClock / 24000000) - 1;

	TimHandle.Instance = TIMx;

	TimHandle.Init.Prescaler         = uhPrescalerValue;
	TimHandle.Init.Period            = PERIOD_VALUE;
	TimHandle.Init.ClockDivision     = 0;
	TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	TimHandle.Init.RepetitionCounter = 0;
	TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	/* Common configuration for all channels */
	sConfig.OCMode       = TIM_OCMODE_PWM1;
	sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
	sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
	sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;

	/*************************************************************************/

	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitStruct.Pin = (GPIO_PIN_8
						   | GPIO_PIN_9
						   | GPIO_PIN_14
						   | GPIO_PIN_15 );

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	
	/*************************************************************************/

	mc_init();

	/*************************************************************************/

	if(HAL_I2C_Slave_Receive_IT(&I2cHandle, rxBuff, BOARD_TRANSFER_CHUNK) != HAL_OK) {
		Error_Handler();
	}

	while(1) {
		if (HAL_I2C_GetState(&I2cHandle) == HAL_I2C_STATE_READY) {
			if(HAL_I2C_Slave_Receive_IT(&I2cHandle, rxBuff, BOARD_TRANSFER_CHUNK) != HAL_OK) {
				Error_Handler();
			}
			mc_push_command(rxBuff);
		} else {
			mc_work();
			HAL_Delay(5);
		}
	}

	return 0;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV                     = 1
  *            PLLMUL                         = RCC_PLL_MUL9 (9)
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void) {
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK) {
		/* Initialization Error */
		while(1);
	}

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	   clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)!= HAL_OK) {
		/* Initialization Error */
		while(1);
	}
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void) {
	/* Turn LED6 on */
	BSP_LED_On(LED6);
	while(1) {
	}
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line) {
	Error_Handler();
}
#endif

void led_on() {
	BSP_LED_On(LED6);
}

void led_off() {
	BSP_LED_Off(LED6);
}


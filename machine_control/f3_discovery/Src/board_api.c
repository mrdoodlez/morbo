#include "board_api.h"
#include "main.h"

#define MC_GPIO_PORT				GPIOA

#define MC_CH0_CW					GPIO_PIN_8
#define MC_CH0_CCW					GPIO_PIN_9

#define MC_CH1_CW					GPIO_PIN_14
#define MC_CH1_CCW					GPIO_PIN_15

#define TIM4_GPIO_PORT_CHANNEL1		GPIOD
#define TIM4_GPIO_PORT_CHANNEL2		GPIOD

#define TIM4_GPIO_PIN_CHANNEL1		GPIO_PIN_12
#define TIM4_GPIO_PIN_CHANNEL2		GPIO_PIN_13

#define GPIO_AF2_TIM4				((uint8_t)0x02U)

#define SERVO_PERIOD				0xB800;

void board_delay(unsigned int ms) {
	HAL_Delay(ms);
}

void board_led_on() {
	BSP_LED_On(LED6);
}

void board_led_off() {
	BSP_LED_Off(LED6);
}

void board_led_toggle() {
	BSP_LED_Toggle(LED6);
}

uint32_t board_get_time() {
	return HAL_GetTick();
}

void board_set_pwm_period (board_pwm_channel_t ch, float ratio) {
	sConfig.Pulse = ratio * PERIOD_VALUE;

	uint32_t channel = (ch == BOARD_PWM_CH_0) ? TIM_CHANNEL_3 : TIM_CHANNEL_4;

	if (HAL_TIM_PWM_Stop(&mcTimHandle, channel) != HAL_OK) {
		/* PWM generation Error */
		Error_Handler();
	}

	if (HAL_TIM_PWM_ConfigChannel(&mcTimHandle, &sConfig, channel) != HAL_OK) {
		/* Configuration Error */
		Error_Handler();
	}

	if (HAL_TIM_PWM_Start(&mcTimHandle, channel) != HAL_OK) {
		/* PWM generation Error */
		Error_Handler();
	}
}

void board_set_pwm_direction (board_pwm_channel_t ch, board_pwm_direction_t dir) {
	if (dir == BOARD_PWM_DIR_CW) {
		HAL_GPIO_WritePin(MC_GPIO_PORT, (ch == BOARD_PWM_CH_0)
						  ? MC_CH0_CCW : MC_CH1_CCW, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MC_GPIO_PORT, (ch == BOARD_PWM_CH_0)
						  ? MC_CH0_CW : MC_CH1_CW, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(MC_GPIO_PORT, (ch == BOARD_PWM_CH_0)
						  ? MC_CH0_CW : MC_CH1_CW, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MC_GPIO_PORT, (ch == BOARD_PWM_CH_0)
						  ? MC_CH0_CCW : MC_CH1_CCW, GPIO_PIN_SET);
	}
}

void board_set_pwm_enable (board_pwm_channel_t ch, board_pwm_enable_t en) {
	if (en) {
		if (HAL_TIM_PWM_Start(&mcTimHandle, (ch == BOARD_PWM_CH_0)
							  ? TIM_CHANNEL_3 : TIM_CHANNEL_4) != HAL_OK) {
			/* PWM generation Error */
			Error_Handler();
		}
	} else {
		if (HAL_TIM_PWM_Stop(&mcTimHandle, (ch == BOARD_PWM_CH_0)
							 ? TIM_CHANNEL_3 : TIM_CHANNEL_4) != HAL_OK) {
			/* PWM generation Error */
			Error_Handler();
		}
		HAL_GPIO_WritePin(MC_GPIO_PORT, (ch == BOARD_PWM_CH_0)
						  ? MC_CH0_CW : MC_CH1_CW, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MC_GPIO_PORT, (ch == BOARD_PWM_CH_0)
						  ? MC_CH0_CCW : MC_CH1_CCW, GPIO_PIN_RESET);
	}
}

void board_enable_servos (void)
{
	uint32_t uhPrescalerValue = (uint32_t)(SystemCoreClock / 2400000) - 1;

	svTimHandle.Instance = TIM4;

	uint16_t period = SERVO_PERIOD;

	svTimHandle.Init.Prescaler         = uhPrescalerValue;
	svTimHandle.Init.Period            = period;
	svTimHandle.Init.ClockDivision     = 0;
	svTimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	svTimHandle.Init.RepetitionCounter = 0;
	svTimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_PWM_Init(&svTimHandle) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	GPIO_InitTypeDef   GPIO_InitStruct;

	/* TIM2 Peripheral clock enable */
	__HAL_RCC_TIM4_CLK_ENABLE();

	TIM_Base_SetConfig(TIM4, &svTimHandle.Init);

	/* Enable all GPIO Channels Clock requested */
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/* Common configuration for all channels */
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	GPIO_InitStruct.Pin = TIM4_GPIO_PIN_CHANNEL1;
	HAL_GPIO_Init(TIM4_GPIO_PORT_CHANNEL1, &GPIO_InitStruct);

	GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	GPIO_InitStruct.Pin = TIM4_GPIO_PIN_CHANNEL2;
	HAL_GPIO_Init(TIM4_GPIO_PORT_CHANNEL2, &GPIO_InitStruct);
}

void board_set_servo_pos (board_servo_channel_t ch, float angle) {
	float ratio = 0.05 + angle / 180.0 * 0.05;

	sConfig.Pulse = ratio * SERVO_PERIOD;

	uint32_t channel = (ch == BOARD_SERVO_CH_0) ? TIM_CHANNEL_1 : TIM_CHANNEL_2;

	if (HAL_TIM_PWM_Stop(&svTimHandle, channel) != HAL_OK) {
		/* PWM generation Error */
		Error_Handler();
	}

	if (HAL_TIM_PWM_ConfigChannel(&svTimHandle, &sConfig, channel) != HAL_OK) {
		/* Configuration Error */
		Error_Handler();
	}

	if (HAL_TIM_PWM_Start(&svTimHandle, channel) != HAL_OK) {
		/* PWM generation Error */
		Error_Handler();
	}
}

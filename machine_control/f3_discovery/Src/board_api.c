#include "board_api.h"
#include "main.h"

#define MC_GPIO_PORT    GPIOA

#define MC_CH0_CW       GPIO_PIN_8
#define MC_CH0_CCW      GPIO_PIN_9

#define MC_CH1_CW       GPIO_PIN_14
#define MC_CH1_CCW      GPIO_PIN_15

void board_delay(unsigned int ms) {
	HAL_Delay(ms);
}

void board_led_toggle() {
	BSP_LED_Toggle(LED6);
}

uint32_t board_get_time() {
	return HAL_GetTick();
}

void board_set_pwm_period (board_pwm_channel_t ch, uint8_t period) {
	sConfig.Pulse = period;
	sConfig.Pulse *= PERIOD_VALUE;
	sConfig.Pulse >>= 8;

	uint32_t channel = (ch == BOARD_PWM_CH_0) ? TIM_CHANNEL_3 : TIM_CHANNEL_4;

	if (HAL_TIM_PWM_Stop(&TimHandle, channel) != HAL_OK) {
		/* PWM generation Error */
		Error_Handler();
	}

	if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, channel) != HAL_OK) {
		/* Configuration Error */
		Error_Handler();
	}

	if (HAL_TIM_PWM_Start(&TimHandle, channel) != HAL_OK) {
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
		if (HAL_TIM_PWM_Start(&TimHandle, (ch == BOARD_PWM_CH_0)
							  ? TIM_CHANNEL_3 : TIM_CHANNEL_4) != HAL_OK) {
			/* PWM generation Error */
			Error_Handler();
		}
	} else {
		if (HAL_TIM_PWM_Stop(&TimHandle, (ch == BOARD_PWM_CH_0)
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


#ifndef _BOARD_API_H_
#define _BOARD_API_H_

#include <stdint.h>

#define BOARD_TRANSFER_CHUNK	16

typedef enum {
	BOARD_PWM_CH_0,
	BOARD_PWM_CH_1
} board_pwm_channel_t;

typedef enum {
	BOARD_PWM_DIR_CW,
	BOARD_PWM_DIR_CCW
} board_pwm_direction_t;

typedef enum {
	BOARD_PWM_CH_DIS,
	BOARD_PWM_CH_EN
} board_pwm_enable_t;

void board_delay(unsigned int ms);

void board_led_toggle();
void board_led_on();
void board_led_off();

uint32_t board_get_time();

void board_set_pwm_period    (board_pwm_channel_t ch, uint8_t period);
void board_set_pwm_direction (board_pwm_channel_t ch, board_pwm_direction_t dir);
void board_set_pwm_enable    (board_pwm_channel_t ch, board_pwm_enable_t en);

#endif //_BOARD_API_H_


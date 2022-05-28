#include "mc_control.h"
#include "mc_proto.h"
#include "board_api.h"
#include <string.h>
#include <math.h>

#define MOTOR_CONTROL_MOVE_PERIOD_MS	100

#define MOTOR_CONTROL_TASK_ID			0
#define TURRET_CONTROL_TASK_ID			1
#define LED_CONTROL_TASK_ID				2

#define MC_NUM_TASKS					3

typedef struct {
	uint32_t scheduler_ts_ms;
	uint32_t is_active;
	void(*task_function)(void*);
} mc_control_task_desc_t;

/*****************************************************************************/

static struct {
	mc_control_cmd_t last_cmd;
	uint32_t new_cmd;
	mc_control_task_desc_t tasks[MC_NUM_TASKS];

	uint32_t motion_begin_ts;
	float vl;
	float vr;

	mc_control_encoders_t encoders;
} _machine_state;

/*****************************************************************************/

mc_control_cmd_t encoders_reply = {
	.m = 'm',
	.b = 'b',
	.code = MC_RC_CODE_GET_ENCODERS,
};

mc_reply_desc_t _transfer_state;

/*****************************************************************************/

static void _mc_work_speeds(mc_control_speeds_t* speeds);
static void _mc_work_stop();

static void _motor_control_task(void *arg);
static void _turret_control_task(void *arg);
static void _led_control_task(void *arg);

/*****************************************************************************/

static uint8_t _cmd_sniff[256];
static uint8_t _cmd_sniff_pos = 0;

/*****************************************************************************/

void mc_init() {
	memset(&_machine_state, 0, sizeof(_machine_state));
	memset(&_transfer_state, 0, sizeof(_transfer_state));
	_machine_state.tasks[MOTOR_CONTROL_TASK_ID].task_function
		= _motor_control_task;
	_machine_state.tasks[TURRET_CONTROL_TASK_ID].task_function
		= _turret_control_task;
	_machine_state.tasks[LED_CONTROL_TASK_ID].task_function
		= _led_control_task;
}

void mc_work() {
	uint32_t curr = board_get_time();

	for (uint32_t task_id = 0; task_id < MC_NUM_TASKS; task_id++) {
		_machine_state.tasks[task_id].task_function(0);
		_machine_state.tasks[task_id].scheduler_ts_ms = curr;
	}

	_machine_state.new_cmd = 0;
}

void mc_push_command(uint8_t* cmd_buff) {
	if ((cmd_buff[0] == 'm') && (cmd_buff[1] == 'b')) {
		board_led_toggle();
		memcpy(&_machine_state.last_cmd, cmd_buff, BOARD_TRANSFER_CHUNK);
		_machine_state.new_cmd = 1;
		for (uint32_t i = 0; i < BOARD_TRANSFER_CHUNK; i++)
			_cmd_sniff[_cmd_sniff_pos++] = cmd_buff[i];

		mc_work();
	}
}

mc_reply_desc_t* mc_get_reply() {
	return &_transfer_state;
}

void mc_update_encoder(mc_encoder_ch_t ch) {
	if (ch == MC_ENCODER_CH_L) {
		if (_machine_state.vl > 0) {
			_machine_state.encoders.pulses_l++;
		} else {
			_machine_state.encoders.pulses_l--;
		}
	} else {
		if (_machine_state.vr > 0) {
			_machine_state.encoders.pulses_r++;
		} else {
			_machine_state.encoders.pulses_r--;
		}
	}
}

/*****************************************************************************/

static void _motor_control_task(void *arg) {
	uint32_t curr = board_get_time();

	if (_machine_state.new_cmd) {
		if (_machine_state.last_cmd.code == MC_RC_CODE_SET_PWM) {
			mc_control_speeds_t *speeds = _machine_state.last_cmd.payload;
			_mc_work_speeds(speeds);
			_machine_state.vl = speeds->pwm_l;
			_machine_state.vr = speeds->pwm_r;
			_machine_state.motion_begin_ts = curr;
			_transfer_state.transfer_len = 0;
		} else if (_machine_state.last_cmd.code == MC_RC_CODE_GET_ENCODERS) {
			*(mc_control_encoders_t*)(encoders_reply.payload) = _machine_state.encoders;
			_transfer_state.transfer_len = sizeof(encoders_reply);
			_transfer_state.transfer_buff = (uint8_t*)&encoders_reply;
		}
	} else {
		if ((curr - _machine_state.motion_begin_ts) > MOTOR_CONTROL_MOVE_PERIOD_MS) {
			if ((_machine_state.vl != 0) || (_machine_state.vr != 0)) {
				_mc_work_stop();
				_machine_state.vl = 0;
				_machine_state.vr = 0;
			}
		}
	}
}

static void _turret_control_task(void *arg) {
	static struct {
		float angle_v;
		float angle_h;
	} curr_state = {
		90.0,
		90.0
	};

	if (_machine_state.new_cmd) {
		if (_machine_state.last_cmd.code == MC_RC_CODE_SET_TURRET) {
			mc_control_turret_t *turret = _machine_state.last_cmd.payload;
			if (curr_state.angle_v != turret->angle_v)
				board_set_servo_pos (BOARD_SERVO_CH_0, turret->angle_v);
			if (curr_state.angle_h != turret->angle_h)
				board_set_servo_pos (BOARD_SERVO_CH_1, turret->angle_h);

			curr_state.angle_v = turret->angle_v;
			curr_state.angle_h = turret->angle_h;
		}
	}
}

static void _led_control_task(void *arg) {
	static struct {
		enum {
			LED_COLOR_RED		= 0,
			LED_COLOR_BLUE		= 1,
		} color;
		uint32_t on_ts;
	} state = {
		.color = LED_COLOR_RED,
	};

	uint32_t curr = board_get_time();

	if ((curr - state.on_ts) > 500) {
		state.color = 1 - state.color;
		state.on_ts = curr;

		if (state.color == LED_COLOR_RED) {
			board_blue_on(0);
			board_red_on(1);
		} else if (state.color == LED_COLOR_BLUE) {
			board_blue_on(1);
			board_red_on(0);
		}
	}
}

static void _mc_work_speeds(mc_control_speeds_t* speeds) {
	board_set_pwm_direction(BOARD_PWM_CH_0, speeds->pwm_l > 0
							? BOARD_PWM_DIR_CCW
							: BOARD_PWM_DIR_CW);
	board_set_pwm_period(BOARD_PWM_CH_0, fabs(speeds->pwm_l));

	board_set_pwm_direction(BOARD_PWM_CH_1, speeds->pwm_r > 0
							? BOARD_PWM_DIR_CCW
							: BOARD_PWM_DIR_CW);
	board_set_pwm_period(BOARD_PWM_CH_1, fabs(speeds->pwm_r));
}

static void _mc_work_stop() {
	board_set_pwm_enable(BOARD_PWM_CH_0, BOARD_PWM_CH_DIS);
	board_set_pwm_enable(BOARD_PWM_CH_1, BOARD_PWM_CH_DIS);
}

#include "mc_control.h"
#include "mc_proto.h"
#include "board_api.h"
#include <string.h>

#define MOTOR_CONTROL_MOVE_PERIOD_MS	100

#define MC_NUM_TASKS					1

#define MOTOR_CONTROL_TASK_ID			0

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
	uint8_t vl;
	uint8_t vr;

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

/*****************************************************************************/

static uint8_t _cmd_sniff[256];
static uint8_t _cmd_sniff_pos = 0;

/*****************************************************************************/

void mc_init() {
	memset(&_machine_state, 0, sizeof(_machine_state));
	memset(&_transfer_state, 0, sizeof(_transfer_state));
	_machine_state.tasks[MOTOR_CONTROL_TASK_ID].task_function
		= _motor_control_task;
}

void mc_work() {
	uint32_t curr = board_get_time();
	if (_machine_state.new_cmd) {
		if ((_machine_state.last_cmd.code == MC_RC_CODE_SET_PWM)
			|| (_machine_state.last_cmd.code == MC_RC_CODE_GET_ENCODERS))
		{
			_machine_state.tasks[MOTOR_CONTROL_TASK_ID].task_function(0);
			_machine_state.tasks[MOTOR_CONTROL_TASK_ID].scheduler_ts_ms = curr;
		}
		_machine_state.new_cmd = 0;
	}

	for (uint32_t task_id = 0; task_id < MC_NUM_TASKS; task_id++) {
		_machine_state.tasks[task_id].task_function(0);
		_machine_state.tasks[task_id].scheduler_ts_ms = curr;
	}
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
		if (_machine_state.vl < 0x80) {
			_machine_state.encoders.pulses_l++;
		} else {
			_machine_state.encoders.pulses_l--;
		}
	} else {
		if (_machine_state.vr < 0x80) {
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
			_machine_state.vl = speeds->speed_l;
			_machine_state.vr = speeds->speed_r;
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

static void _mc_work_speeds(mc_control_speeds_t* speeds) {
	board_set_pwm_direction(BOARD_PWM_CH_0, speeds->speed_l < 0x80
							? BOARD_PWM_DIR_CCW
							: BOARD_PWM_DIR_CW);
	board_set_pwm_period(BOARD_PWM_CH_0, speeds->speed_l << 1);

	board_set_pwm_direction(BOARD_PWM_CH_1, speeds->speed_r < 0x80
							? BOARD_PWM_DIR_CCW
							: BOARD_PWM_DIR_CW);
	board_set_pwm_period(BOARD_PWM_CH_1, speeds->speed_r << 1);
}

static void _mc_work_stop() {
	board_set_pwm_enable(BOARD_PWM_CH_0, BOARD_PWM_CH_DIS);
	board_set_pwm_enable(BOARD_PWM_CH_1, BOARD_PWM_CH_DIS);
}

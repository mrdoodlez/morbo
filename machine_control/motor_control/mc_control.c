#include "mc_control.h"
#include "board_api.h"
#include <string.h>

#define SCHEDULER_PERIOD_MS				300

#define MC_NUM_TASKS					1

#define MOTOR_CONTROL_TASK_ID			0

#define MC_RC_CODE_ACK					0
#define MC_RC_CODE_NACK					1

#define MC_RC_CODE_PING					2

#define MC_RC_CODE_STOP					3
#define MC_RC_CODE_SPEEDS				4

#define MOTOR_CONTROL_MOVE_PERIOD_MS	500

typedef struct {
	uint8_t speed_l;
	uint8_t speed_r;
} __attribute__((packed)) mc_control_speeds_t;

typedef struct {
	uint32_t scheduler_ts_ms;
	uint32_t is_active;
	void(*task_function)(void*);
} mc_control_task_desc_t;

/*****************************************************************************/

typedef struct {
	uint8_t code;
	uint8_t payload[BOARD_TRANSFER_CHUNK -1];
} __attribute__((packed)) mc_control_cmd_t;

/*****************************************************************************/

static struct {
	mc_control_cmd_t last_cmd;
	uint32_t new_cmd;
	mc_control_task_desc_t tasks[MC_NUM_TASKS];
} _machine_state;

/*****************************************************************************/

static void _mc_work_speeds(mc_control_speeds_t* speeds);
static void _mc_work_stop();

static void _motor_control_task(void *arg);

/*****************************************************************************/

void mc_init() {
	memset(&_machine_state, 0, sizeof(_machine_state));
	_machine_state.tasks[MOTOR_CONTROL_TASK_ID].task_function
		= _motor_control_task;
}

void mc_work() {
	uint32_t curr = board_get_time();
	if (_machine_state.new_cmd) {
		if (_machine_state.last_cmd.code == MC_RC_CODE_SPEEDS)
		{
			_machine_state.tasks[MOTOR_CONTROL_TASK_ID].task_function(0);
			_machine_state.tasks[MOTOR_CONTROL_TASK_ID].scheduler_ts_ms = curr;
		}
		_machine_state.new_cmd = 0;
	}

	for (uint32_t task_id = 0; task_id < MC_NUM_TASKS; task_id++) {
		if ((curr - _machine_state.tasks[task_id].scheduler_ts_ms)
				> SCHEDULER_PERIOD_MS) {
			_machine_state.tasks[task_id].task_function(0);
			_machine_state.tasks[task_id].scheduler_ts_ms = curr;
		}
	}
}

void mc_push_command(uint8_t* cmd_buff) {
	memcpy(&_machine_state.last_cmd, cmd_buff, BOARD_TRANSFER_CHUNK);
	_machine_state.new_cmd = 1;
}

/*****************************************************************************/

static void _motor_control_task(void *arg) {
	static struct {
		uint32_t motion_begin_ts;
		uint8_t vl;
		uint8_t vr;
	} task_state = {0, 0, 0};

	uint32_t curr = board_get_time();

	if (_machine_state.new_cmd
			&& _machine_state.last_cmd.code == MC_RC_CODE_SPEEDS) {
		mc_control_speeds_t *speeds = _machine_state.last_cmd.payload;
		_mc_work_speeds(speeds);
		task_state.vl = speeds->speed_l;
		task_state.vr = speeds->speed_r;
		task_state.motion_begin_ts = curr;
		board_led_on();
	} else if ((curr - task_state.motion_begin_ts)
			> MOTOR_CONTROL_MOVE_PERIOD_MS) {
		if ((task_state.vl != 0) || (task_state.vr != 0)) {
			_mc_work_stop();
			task_state.vl = 0;
			task_state.vr = 0;
			board_led_off();
		}
	}
}

static void _mc_work_speeds(mc_control_speeds_t* speeds) {
	board_set_pwm_direction(BOARD_PWM_CH_0, speeds->speed_l < 0x80
							? BOARD_PWM_DIR_CW
							: BOARD_PWM_DIR_CCW);
	board_set_pwm_period(BOARD_PWM_CH_0, speeds->speed_l << 1);

	board_set_pwm_direction(BOARD_PWM_CH_1, speeds->speed_r < 0x80
							? BOARD_PWM_DIR_CW
							: BOARD_PWM_DIR_CCW);
	board_set_pwm_period(BOARD_PWM_CH_1, speeds->speed_r << 1);

}

static void _mc_work_stop() {
	board_set_pwm_enable(BOARD_PWM_CH_0, BOARD_PWM_CH_DIS);
	board_set_pwm_enable(BOARD_PWM_CH_1, BOARD_PWM_CH_DIS);
}

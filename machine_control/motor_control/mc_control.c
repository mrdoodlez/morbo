#include "mc_control.h"
#include "board_api.h"
#include <string.h>

#define MC_DATA_SIZE        256
#define MC_WQE_LEN          8

#define MC_RC_CODE_ACK      0
#define MC_RC_CODE_NACK     1

#define MC_RC_CODE_PING     2

#define MC_RC_CODE_STOP     3
#define MC_RC_CODE_SPEEDS   4

typedef struct {
	uint8_t speed_l;
	uint8_t speed_r;
} __attribute__((packed)) mc_rc_speeds_t;

/*****************************************************************************/

typedef struct {
	uint16_t seq_num;
	uint8_t  code;
	uint8_t  payload_len;
} rcdev_common_header_t;

typedef struct {
	rcdev_common_header_t hdr;
	uint8_t *payload;
} mc_wqe_desc_t;

static struct {
	uint8_t data[MC_DATA_SIZE];
	uint8_t pos;
} _cmd_data;

static struct {
	uint32_t head;
	uint32_t tail;
	mc_wqe_desc_t q[MC_WQE_LEN];
} _work_queue;

static struct {
	uint32_t is_moving;
	uint32_t last_cmd_ts;
} _machine_state;

/*****************************************************************************/

static void _mc_work_speeds(mc_rc_speeds_t* speeds);
static void _mc_work_stop();

/*****************************************************************************/

void mc_init() {
	memset(&_cmd_data, 0, sizeof(_cmd_data));
	memset(&_work_queue, 0, sizeof(_work_queue));
	memset(&_machine_state, 0, sizeof(_machine_state));
}

void mc_work() {
	uint32_t cmd_rcvd = 0;
	if (_work_queue.tail != _work_queue.head) {
		cmd_rcvd = 1;
		mc_wqe_desc_t *wqe = &(_work_queue.q[_work_queue.tail]);
		switch(wqe->hdr.code) {
		case MC_RC_CODE_STOP:
			_mc_work_stop();
			break;
		case MC_RC_CODE_SPEEDS:
			_mc_work_speeds((mc_rc_speeds_t*)(wqe->payload));
			break;
		case MC_RC_CODE_PING:
			break;
		default:
			cmd_rcvd = 0;
			break;
		}
		_work_queue.tail++;
		_work_queue.tail %= MC_WQE_LEN;
	}
	if (cmd_rcvd) {
		_machine_state.last_cmd_ts = board_get_time();
	} else {
		if (_machine_state.is_moving) {
			if ((board_get_time() - _machine_state.last_cmd_ts) > 3000) {
				_mc_work_stop();
			}
		}
	}
}

void mc_push_command(uint8_t* cmd_buff) {
	rcdev_common_header_t* cmd = cmd_buff;
	_work_queue.q[_work_queue.head].hdr = *cmd;
	if(cmd->payload_len > 0) {
		if ((_cmd_data.pos + cmd->payload_len) > MC_DATA_SIZE) {
			_cmd_data.pos = 0;
		}
		uint8_t *data = &(_cmd_data.data[_cmd_data.pos]);
		memcpy(data, (uint8_t*)cmd + sizeof(rcdev_common_header_t), cmd->payload_len);
		_cmd_data.pos += cmd->payload_len;
		_work_queue.q[_work_queue.head].payload = data;
	}
	_work_queue.head++;
	_work_queue.head %= MC_WQE_LEN;
}

/*****************************************************************************/

static void _mc_work_speeds(mc_rc_speeds_t* speeds) {
	board_set_pwm_direction(BOARD_PWM_CH_0, speeds->speed_l < 0x80
							? BOARD_PWM_DIR_CW
							: BOARD_PWM_DIR_CCW);
	board_set_pwm_period(BOARD_PWM_CH_0, speeds->speed_l << 1);

	board_set_pwm_direction(BOARD_PWM_CH_1, speeds->speed_r < 0x80
							? BOARD_PWM_DIR_CW
							: BOARD_PWM_DIR_CCW);
	board_set_pwm_period(BOARD_PWM_CH_1, speeds->speed_r << 1);

	_machine_state.is_moving
		= (speeds->speed_l != 0) || (speeds->speed_r != 0);
}

static void _mc_work_stop() {
	board_set_pwm_enable(BOARD_PWM_CH_0, BOARD_PWM_CH_DIS);
	board_set_pwm_enable(BOARD_PWM_CH_1, BOARD_PWM_CH_DIS);
	_machine_state.is_moving = 0;
}

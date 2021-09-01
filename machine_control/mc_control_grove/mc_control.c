#include "mc_control.h"
#include "grove_driver.h"
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

/*****************************************************************************/

extern void delay(unsigned int ms);
extern void led_toggle();

/*****************************************************************************/

static void _mc_work_speeds(mc_rc_speeds_t* speeds);

/*****************************************************************************/

void mc_task_run() {
    memset(&_cmd_data, 0, sizeof(_cmd_data));
    memset(&_work_queue, 0, sizeof(_work_queue));

    grove_begin(0x0F);    
    
    while(1) {
        if (_work_queue.tail != _work_queue.head) {
            mc_wqe_desc_t *wqe = &(_work_queue.q[_work_queue.tail]);
            switch(wqe->hdr.code) {
                case MC_RC_CODE_STOP:  
                    break;
                case MC_RC_CODE_SPEEDS:
                    _mc_work_speeds((mc_rc_speeds_t*)(wqe->payload));
                    break;
                case MC_RC_CODE_PING:
                default:
                    break;
            }
            _work_queue.tail++;
            _work_queue.tail %= MC_WQE_LEN;
        }
        delay(50);
    }
}

void mc_push(rcdev_common_header_t* cmd) {
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
    grove_speed(MOTOR1, speeds->speed_l);
    grove_speed(MOTOR2, speeds->speed_r);
}

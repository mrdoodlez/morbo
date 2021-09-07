#include "rc_device.h"
#include "mc_control.h"

#define RC_SYNC_1   'm'
#define RC_SYNC_2   'b'

#define RC_CS_LEN   2

extern void led_toggle();
extern void led_on();
extern void led_off();

typedef enum {
    RCDEV_PROTO_STATE_OUT_OF_SYNC,
    RCDEV_PROTO_STATE_IN_SYNC,
    RCDEV_PROTO_STATE_FETCH_HEADER,
    RCDEV_PROTO_STATE_FETCH_PAYLOAD,
    RCDEV_PROTO_STATE_FETCH_CS,
} rcdev_proto_state_t;

static struct {
    uint32_t pos;
    rcdev_proto_state_t pstate;
    uint8_t buff[256];
    uint8_t dbg_buff[256];
    uint8_t dbg_pos;
} _state = {
    .pos = 0,
    .pstate = RCDEV_PROTO_STATE_OUT_OF_SYNC
};

void rcdev_on_new_data(uint8_t data) {
    _state.dbg_buff[_state.dbg_pos++] = data;
    switch(_state.pstate) {
        case RCDEV_PROTO_STATE_IN_SYNC:
            if (data == RC_SYNC_2) {
                _state.pstate = RCDEV_PROTO_STATE_FETCH_HEADER;
            }
            break;
        case RCDEV_PROTO_STATE_FETCH_HEADER:
            _state.buff[_state.pos++] = data;
            if (_state.pos == sizeof(rcdev_common_header_t)) {
                if (((rcdev_common_header_t*)(_state.buff))->payload_len > 0) {
                    _state.pstate = RCDEV_PROTO_STATE_FETCH_PAYLOAD;
                } else {
                    _state.pstate = RCDEV_PROTO_STATE_FETCH_CS;
                }
            }
            break;
        case RCDEV_PROTO_STATE_FETCH_PAYLOAD:
            _state.buff[_state.pos++] = data;
            if (_state.pos == (sizeof(rcdev_common_header_t)
                    + ((rcdev_common_header_t*)(_state.buff))->payload_len)) {
                _state.pstate = RCDEV_PROTO_STATE_FETCH_CS;
            }
            break;
        case RCDEV_PROTO_STATE_FETCH_CS:
            _state.buff[_state.pos++] = data;
            if (_state.pos == (sizeof(rcdev_common_header_t)
                    + ((rcdev_common_header_t*)(_state.buff))->payload_len)
                    + RC_CS_LEN) {
                mc_push((rcdev_common_header_t*)(_state.buff));
                led_off();
                rcdev_proto_reset();
            }
            break;
        case RCDEV_PROTO_STATE_OUT_OF_SYNC:
        default:
            if (data == RC_SYNC_1) {
                led_on();
                _state.pstate = RCDEV_PROTO_STATE_IN_SYNC;
            }
            break;
    }
}

void rcdev_proto_reset() {
    _state.pos = 0;
    _state.pstate = RCDEV_PROTO_STATE_OUT_OF_SYNC;
}

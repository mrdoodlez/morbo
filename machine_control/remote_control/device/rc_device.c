#include "rc_device.h"

#define RC_SYNC_1   'm'
#define RC_SYNC_2   'b'

#define RC_CS_LEN   2


typedef struct {
    uint16_t seq_num;
    uint8_t  code;
    uint8_t  payload_len;
} rcdev_common_header_t;

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


static void _process(rcdev_common_header_t* hdr, uint8_t *payload)  {
    _state.pos = 0;
}

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
                _state.pstate = RCDEV_PROTO_STATE_FETCH_PAYLOAD;
            }
            break;
        case RCDEV_PROTO_STATE_FETCH_CS:
            _state.buff[_state.pos++] = data;
            if (_state.pos == (sizeof(rcdev_common_header_t)
                    + ((rcdev_common_header_t*)(_state.buff))->payload_len)
                    + RC_CS_LEN) {
                _process(_state.buff,
                        _state.buff + sizeof(rcdev_common_header_t));
                _state.pos = 0;
                _state.pstate = RCDEV_PROTO_STATE_OUT_OF_SYNC;
            }
            break;
        case RCDEV_PROTO_STATE_OUT_OF_SYNC:
        default:
            if (data == RC_SYNC_1) {
                _state.pstate = RCDEV_PROTO_STATE_IN_SYNC;
            }
            break;
    }
}


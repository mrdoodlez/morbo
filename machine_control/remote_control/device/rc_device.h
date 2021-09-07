#ifndef _RC_DEVICE_H_
#define _RC_DEVICE_H_

#include <stdint.h>

typedef struct {
    uint16_t seq_num;
    uint8_t  code;
    uint8_t  payload_len;
} rcdev_common_header_t;

void rcdev_on_new_data(uint8_t data);
void rcdev_proto_reset();

#endif //_RC_DEVICE_H_

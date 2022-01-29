#ifndef _MC_CONTROL_H_
#define _MC_CONTROL_H_

#include <stdint.h>
#include <stddef.h>

typedef struct {
	size_t transfer_len;
	uint8_t *transfer_buff;
} mc_reply_desc_t;

typedef enum {
	MC_ENCODER_CH_L,
	MC_ENCODER_CH_R
} mc_encoder_ch_t;

void mc_init();
void mc_push_command(uint8_t* cmd_buff);
void mc_work();
mc_reply_desc_t* mc_get_reply();

void mc_update_encoder(mc_encoder_ch_t ch);

#endif //_MC_CONTROL_H_

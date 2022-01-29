#ifndef _MC_CONTROL_H_
#define _MC_CONTROL_H_

#include <stdint.h>
#include <stddef.h>

void mc_init();
void mc_push_command(uint8_t* cmd_buff);
void mc_work();
void mc_get_reply(uint8_t* msg_buff, size_t* msg_len);

#endif //_MC_CONTROL_H_

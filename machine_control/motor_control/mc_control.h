#ifndef _MC_CONTROL_H_
#define _MC_CONTROL_H_

#include <stdint.h>

void mc_init();
void mc_push_command(uint8_t* cmd_buff);
void mc_work();

#endif //_MC_CONTROL_H_

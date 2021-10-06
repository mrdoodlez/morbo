#ifndef _MC_CONTROL_H_
#define _MC_CONTROL_H_

#include "rc_device.h"

void mc_task_run();
void mc_push(rcdev_common_header_t* cmd);

#endif //_MC_CONTROL_H_

#ifndef MESSAGE_TASK_H
#define MESSAGE_TASK_H
#include "struct_typedef.h"
#include "can_bsp.h"
#include "pid.h"
#include "remote_control.h"
//包的大小
#define MESSAGE_TASK_TIME						3
#define MESSAGE_TASK_INIT_TIME		500
extern void message_task(void const *pvParameters);

#endif

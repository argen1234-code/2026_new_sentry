#ifndef __UI_TASK_H
#define __UI_TASK_H

#include "cmsis_os.h"
#include "stdint.h"
#include "main.h"

#include "ui_interface.h"
#include "ui_types.h"

#define UI_TIME 100 //任务周期是50ms

extern void UI_task(void const * argument);

#endif





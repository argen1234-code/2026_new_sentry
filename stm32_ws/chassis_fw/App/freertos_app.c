/**
  * @file       freertos_app.c
  * @brief      FreeRTOS 任务创建
  * @details    只创建两个任务：
  *             - comm：排空 USB 接收缓冲、解析旧协议、周期回发状态；
  *             - chassis：全向轮解算与 FDCAN1 电流下发（1 kHz）。
  */

#include "FreeRTOS.h"
#include "cmsis_os2.h"

#include "chassis_task.h"
#include "legacy_bridge.h"
#include "usb_cdc_port.h"

static void comm_task(void *argument);

void MX_FREERTOS_Init(void);

static const osThreadAttr_t comm_attributes = {
    .name = "comm",
    .stack_size = 512 * 4,
    .priority = osPriorityNormal,
};

static const osThreadAttr_t chassis_attributes = {
    .name = "chassis",
    .stack_size = 768 * 4,
    .priority = osPriorityHigh,
};

static void comm_task(void *argument)
{
    (void)argument;

    for (;;)
    {
        usb_cdc_port_poll_rx();
        legacy_bridge_tick();
        osDelay(1);
    }
}

void MX_FREERTOS_Init(void)
{
    (void)osThreadNew(comm_task, NULL, &comm_attributes);
    (void)osThreadNew(chassis_task, NULL, &chassis_attributes);
}

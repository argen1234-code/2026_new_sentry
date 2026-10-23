/**
 ******************************************************************************
 * @file    task_main.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   应用任务启动入口声明。
 *
 * @note    本文件只负责声明 app 启动编排入口，不承载控制逻辑、设备协议
 *          或状态估计。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_TASKS_TASK_MAIN_H
#define PRINTK_TASKS_TASK_MAIN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 按当前 Robot target 启动所有启用的 Apps。
 *
 * 启动顺序由 target 的 `ROBOT_APP_ENABLE_*` 宏裁剪，失败时立即停止，
 * 由上层入口进入 `Error_Handler()`。
 *
 * @return 全部启动成功返回 1，否则返回 0。
 */
uint8_t task_main_init(void);

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_TASKS_TASK_MAIN_H */

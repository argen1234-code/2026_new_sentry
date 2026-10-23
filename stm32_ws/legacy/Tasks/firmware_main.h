/**
 ******************************************************************************
 * @file    main.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   固件主入口声明。
 *
 * @note    本文件承载当前工程自己的 `main()` 入口，不使用 CubeMX 生成的
 *          `Boards/Core/Src/main.c` 或 `main.cpp` 作为最终入口。
 * @note    `TASK_MAIN_TARGET` 用于在编译期选择当前固件要展开的具体 target
 *          外设初始化路径，默认跟随 `ROBOT_TARGET`。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_TASKS_MAIN_H
#define PRINTK_TASKS_MAIN_H

#include "robot.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 当前主入口展开的机器人 target。
 *
 * 工程宏可以显式覆盖该值；未覆盖时使用 `Robot/robot.h` 选择的
 * `ROBOT_TARGET`。该宏只负责主入口外设初始化分支，不承担运行时切换。
 */
#ifndef TASK_MAIN_TARGET
#define TASK_MAIN_TARGET ROBOT_TARGET
#endif

/**
 * @brief 应用程序主入口。
 *
 * 该函数完成 HAL、系统时钟、CubeMX 外设、BSP 和 RTOS 的启动。
 *
 * @return 正常情况下不会返回。
 */
int main(void);

/**
 * @brief 按当前 target 初始化 CubeMX 外设。
 *
 * 该函数只调用 `MX_*_Init()`，具体设备抽象和协议启动仍由 BSP、Devices
 * 和 Apps 各自负责。
 */
void task_main_periph_init(void);

/**
 * @brief 配置系统主时钟。
 *
 * 时钟参数来自当前 DM02 STM32H723 CubeMX 配置。
 */
void SystemClock_Config(void);

/**
 * @brief 配置外设公共时钟。
 *
 * 当前主要配置 ADC 和 FDCAN 使用的 PLL2 时钟。
 */
void PeriphCommonClock_Config(void);

/**
 * @brief 不可恢复错误处理入口。
 *
 * 发生 HAL/BSP/任务初始化失败时关闭中断并停留在该函数内。
 */
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_TASKS_MAIN_H */

#ifndef TASK_PROFILER_SAMPLING_H
#define TASK_PROFILER_SAMPLING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "tim.h" /* 如果你的 TIM_HandleTypeDef 定义在别处请调整或改为前向声明 */

/*
 * 初始化并启动采样型任务分析器（会创建一个后台任务）
 *
 * 参数:
 *   htim               - 指向已经由 MX 配置好的定时器句柄（例如 &htim2）
 *   sample_hz          - 采样频率（Hz），建议 500 ~ 2000（高采样率更精确但中断开销更大）
 *   report_interval_ms - 打印/报告间隔（ms），例如 2000 表示每 2s 打印一次
 *   stack_words        - profiler 任务栈大小（words，32-bit word）。例如 512/4=128 words
 *   task_priority      - profiler 任务优先级（FreeRTOS 风格，例如 tskIDLE_PRIORITY + 1）
 *
 * 返回:
 *   0 = 成功创建并启动；非 0 = 失败（常见原因：内存不足）
 *
 * 使用说明:
 *  1) 在 main 或某个 init 任务里调用 task_profiler_start(...)；
 *  2) 在中断向量/IRQ handler 或 HAL 回调里调用 task_profiler_tim_irqhandler(htim) 来让模块采样；
 *     - 示例：在 TIMx_IRQHandler 或 HAL_TIM_PeriodElapsedCallback 中调用；
 *  3) 模块会在后台周期性打印统计结果（通过 printf；用户可修改输出函数）。
 */
int task_profiler_start(TIM_HandleTypeDef *htim,
                        uint32_t sample_hz,
                        uint32_t report_interval_ms,
                        uint16_t stack_words,
                        uint32_t task_priority);

/*
 * 停止并删除 profiler 任务（会停止定时器的中断）
 */
void task_profiler_stop(void);

/*
 * 中断时调用：把采样逻辑放在这里（函数体很短、适合在 ISR 中直接调用）
 * 注意：只应在 TIM 中断或等价高精度中断里调用。
 */
void task_profiler_tim_irqhandler(TIM_HandleTypeDef *htim);

/*
 * 立即获取快照（非阻塞）：把结果拷到 caller 提供的缓冲区
 * 返回当前登记的任务数（<= max_tasks）
 *
 * names：数组，元素为 char name_buf[NAME_LEN] 的指针（或等价）；
 * counts：对应的 uint32_t 数组，接收样本计数
 * max_tasks：names/counts 能容纳的元素数
 *
 * 如果只想让模块在后台打印，不需要调用此函数。
 */
size_t task_profiler_get_snapshot(char **names, uint32_t *counts, size_t max_tasks, size_t name_buf_len);

/* 返回累计采样总数（当前窗口） */
uint32_t task_profiler_get_total_samples(void);

/* 重置计数（清零） */
void task_profiler_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* TASK_PROFILER_SAMPLING_H */

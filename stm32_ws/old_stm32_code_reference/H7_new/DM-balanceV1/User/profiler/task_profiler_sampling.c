/*
 * task_profiler_sampling.c
 *
 * 采样型任务分析器（sampling profiler）实现文件
 *  - 不依赖 FreeRTOS 运行时统计
 *  - 使用硬件定时器中断做采样（在中断中快速记录当前线程 id）
 *  - 后台任务定期打印统计结果
 *
 * 请把该文件加入工程，确保在 main 中或 init 任务里调用 task_profiler_start(...)
 *
 * 注意（重要）：
 *  - 在 ISR 中获取当前线程 ID（osThreadGetId / xTaskGetCurrentTaskHandle）可能依赖于 RTOS 实现。
 *    在大多数 CMSIS-RTOS2 / RTX 环境，osThreadGetId() 在 ISR 中返回当前线程（在某些移植/RTOS 版本上也工作）。
 *    如果在你的平台中 ISR 中的 osThreadGetId() 返回 NULL 或行为异常，请改为在 TIM IRQ 中读取 RTOS 内部“当前 TCB”指针（非公开 API，风险较高），或改用采样任务（但精度较差）。
 *
 *  - 本实现尽量在 ISR 中只做非常短的操作（查找/登记 ID 并 ++ 计数）。
 *
 *  - 输出使用 printf，请确保 printf 已被重定向（UART/RTT）。
 */

#include "task_profiler_sampling.h"

#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h" /* 修改为你工程对应的 HAL 头（如 stm32f4xx_hal.h） */

#define MAX_TRACKED_TASKS 64
#define NAME_LEN 24

/* Internal arrays */
static void *tracked_ids[MAX_TRACKED_TASKS]; /* store osThreadId_t (pointer) */
static char tracked_names[MAX_TRACKED_TASKS][NAME_LEN];
static volatile uint32_t tracked_counts[MAX_TRACKED_TASKS];
static volatile uint32_t total_samples = 0;

/* Timers / task handle */
static TIM_HandleTypeDef *prof_htim = NULL;
static osThreadId profiler_task_id = NULL;

/* Config */
static uint32_t profiler_report_ms = 2000;
static uint32_t profiler_sample_hz = 1000;

/* Helper: find or register id; returns index or -1 */
static int find_or_register_id(void *id)
{
    if (id == NULL) return -1;
    /* First try to find existing */
    for (int i = 0; i < MAX_TRACKED_TASKS; ++i) {
        if (tracked_ids[i] == id) return i;
    }
    /* Then register in first empty slot */
    for (int i = 0; i < MAX_TRACKED_TASKS; ++i) {
        if (tracked_ids[i] == NULL) {
            tracked_ids[i] = id;
            const char *name = osThreadGetName((osThreadId)id);
            if (name && name[0]) {
                strncpy(tracked_names[i], name, NAME_LEN - 1);
                tracked_names[i][NAME_LEN - 1] = '\0';
            } else {
                /* fallback name (pointer-based) */
                snprintf(tracked_names[i], NAME_LEN, "T%p", id);
            }
            tracked_counts[i] = 0;
            return i;
        }
    }
    /* no slot */
    return -1;
}

/* This function is intended to be called from TIM IRQ or HAL_TIM_PeriodElapsedCallback:
 * - it must be short and safe in ISR context */
void task_profiler_tim_irqhandler(TIM_HandleTypeDef *htim)
{
    if (htim == NULL || prof_htim == NULL) return;
    if (htim != prof_htim) return;

    /* Minimal check and clear handled by caller (HAL IRQ) - we assume caller calls this after HAL clears flags */
    /* Get current thread id. Note: behaviour may vary by RTOS/port. */
    void *cur = (void*)osThreadGetId(); /* CMSIS-RTOS2 API; returns current thread id */

    /* If osThreadGetId returns NULL in ISR on your port, consider replacing with platform-specific method */
    if (cur == NULL) {
        total_samples++;
        return;
    }

    int idx = find_or_register_id(cur);
    if (idx >= 0) {
        /* atomic increment */
        tracked_counts[idx]++;
    }
    total_samples++;
}

/* Helper: stop timer IRQ */
static void stop_profiler_timer(void)
{
    if (prof_htim) {
        HAL_TIM_Base_Stop_IT(prof_htim);
    }
}

/* Helper: start timer IRQ (assumes prof_htim configured in MX to tick at sample_hz or caller configures prescaler/period accordingly) */
static void start_profiler_timer(void)
{
    if (prof_htim) {
        /* If MX did not configure frequency, try to configure here based on profiler_sample_hz.
         * But safer approach: user configures TIM in CubeMX.
         * We attempt to start interrupt regardless.
         */
        HAL_TIM_Base_Start_IT(prof_htim);
    }
}

/* Print helper - you can replace with RTT/uart implementation */
static void profiler_print(const char *s)
{
    printf("%s", s);
}

/* Task entry: report periodically */
void task_profiler_task(void *arg)
{
    uint32_t interval = profiler_report_ms;
    if (arg) interval = (uint32_t)(uintptr_t)arg;

    profiler_report_ms = interval;

    /* Ensure timer started */
    start_profiler_timer();

    for (;;) {
        /* Wait for report interval */
        osDelay(interval);

        /* Copy snapshot atomically: disable IRQ to avoid races with ISR */
        taskENTER_CRITICAL();
        uint32_t sample_total = total_samples;
        /* copy tracked arrays to local buffers */
        int n = 0;
        for (int i = 0; i < MAX_TRACKED_TASKS; ++i) {
            if (tracked_ids[i] != NULL) n++;
        }

        char local_names[MAX_TRACKED_TASKS][NAME_LEN];
        uint32_t local_counts[MAX_TRACKED_TASKS];
        int local_n = 0;
        for (int i = 0; i < MAX_TRACKED_TASKS; ++i) {
            if (tracked_ids[i] != NULL) {
                strncpy(local_names[local_n], tracked_names[i], NAME_LEN);
                local_counts[local_n] = tracked_counts[i];
                local_n++;
            }
        }
        /* clear counters for next window */
        for (int i = 0; i < MAX_TRACKED_TASKS; ++i) {
            tracked_counts[i] = 0;
        }
        total_samples = 0;
        taskEXIT_CRITICAL();

        /* Format print */
        char *buf = pvPortMalloc(1024 + local_n * 128);
        if (buf == NULL) {
            /* fallback simple print */
            profiler_print("Profiler: malloc failed for report buffer\n");
            continue;
        }
        size_t off = 0;
        off += snprintf(buf + off, 1024 + local_n * 128 - off, "=== Profiler Report (samples=%lu) ===\r\n", (unsigned long)sample_total);
        for (int i = 0; i < local_n; ++i) {
            float pct = 0.0f;
            if (sample_total > 0) pct = ((float)local_counts[i] * 100.0f) / (float)sample_total;
            off += snprintf(buf + off, 1024 + local_n * 128 - off, "%2d) %-20s : %8lu samples, %6.2f %%\r\n",
                            i + 1, local_names[i], (unsigned long)local_counts[i], pct);
            if (off >= 1024 + local_n * 128 - 100) break;
        }
        profiler_print(buf);
        vPortFree(buf);
    }
}

/* Start API: creates the profiler task and stores timer handle */
int task_profiler_start(TIM_HandleTypeDef *htim,
                        uint32_t sample_hz,
                        uint32_t report_interval_ms,
                        uint16_t stack_words,
                        uint32_t task_priority)
{
    if (profiler_task_id != NULL) return 0; /* already started */

    if (htim == NULL) return -1;
    prof_htim = htim;
    profiler_sample_hz = sample_hz;
    profiler_report_ms = report_interval_ms;

    /* Start the HAL timer in interrupt mode (MX typically configured) */
    if (HAL_TIM_Base_Start_IT(prof_htim) != HAL_OK) {
        /* If can't start, still try to create task; but sampling may not happen */
    }

    /* Create task using CMSIS-RTOS2 or FreeRTOS API */
    osThreadAttr attr;
    memset(&attr, 0, sizeof(attr));
    attr.name = "profiler";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = (uint32_t)stack_words * 4U; /* convert words to bytes if user passed words */
    attr.priority = (osPriority_t)task_priority;

    /* osThreadNew requires osPriority enum; if using FreeRTOS CMSIS wrapper this works.
       If osThreadNew not available in your environment, fallback to xTaskCreate. */
#if defined(osThreadNew)
    profiler_task_id = osThreadNew((osThreadFunc_t)task_profiler_task, (void*)(uintptr_t)profiler_report_ms, &attr);
    if (profiler_task_id == NULL) {
        /* fallback to FreeRTOS xTaskCreate */
        BaseType_t res = xTaskCreate((TaskFunction_t)task_profiler_task, "profiler", (uint16_t)stack_words, (void*)(uintptr_t)profiler_report_ms, (UBaseType_t)task_priority, (TaskHandle_t*)&profiler_task_id);
        if (res != pdPASS) return -2;
    }
#else
    BaseType_t res = xTaskCreate((TaskFunction_t)task_profiler_task, "profiler", (uint16_t)stack_words, (void*)(uintptr_t)profiler_report_ms, (UBaseType_t)task_priority, (TaskHandle_t*)&profiler_task_id);
    if (res != pdPASS) return -2;
#endif

    return 0;
}

void task_profiler_stop(void)
{
    if (profiler_task_id) {
        osThreadTerminate(profiler_task_id);
        profiler_task_id = NULL;
    }
    stop_profiler_timer();
}

/* Snapshot API */
size_t task_profiler_get_snapshot(char **names, uint32_t *counts, size_t max_tasks, size_t name_buf_len)
{
    size_t n = 0;
    taskENTER_CRITICAL();
    for (int i = 0; i < MAX_TRACKED_TASKS && n < max_tasks; ++i) {
        if (tracked_ids[i] != NULL) {
            if (names != NULL && name_buf_len > 0) {
                strncpy(names[n], tracked_names[i], name_buf_len - 1);
                names[n][name_buf_len - 1] = '\0';
            }
            if (counts != NULL) counts[n] = tracked_counts[i];
            n++;
        }
    }
    taskEXIT_CRITICAL();
    return n;
}

uint32_t task_profiler_get_total_samples(void)
{
    uint32_t t;
    taskENTER_CRITICAL();
    t = total_samples;
    taskEXIT_CRITICAL();
    return t;
}

void task_profiler_reset(void)
{
    taskENTER_CRITICAL();
    for (int i = 0; i < MAX_TRACKED_TASKS; ++i) {
        tracked_ids[i] = NULL;
        tracked_names[i][0] = '\0';
        tracked_counts[i] = 0;
    }
    total_samples = 0;
    taskEXIT_CRITICAL();
}

/**
 ******************************************************************************
 * @file    bsp_dwt.c
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   DWT CYCCNT 时间基、短延时和运行时间线实现。
 *
 * @note    本文件基于旧版 DWT BSP 重构命名和接口，并保留短周期
 *          dt、短阻塞延时、运行时间读取等核心行为。
 * @note    32 位 CYCCNT 溢出轮数依赖周期性读取维护；两次读取间隔
 *          不应超过一次完整 CYCCNT 回绕周期。
 * @note    64 位时间线使用 2^32 为单轮周期，修正旧版使用
 *          UINT32_MAX 导致每次溢出少计 1 cycle 的问题。
 * @note    本新版不再暴露旧版全局 SysTime/CYCCNT64，外部应通过
 *          bsp_dwt_get_time()/bsp_dwt_get_timeline_*() 获取时间。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "bsp_dwt.h"

#include <stddef.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置板级外设封装需要的私有宏、局部表和轻量状态，不承载机器人业务语义。
 * ========================================================================== */

/* ==========================================================================
 * 状态缓存与工作区
 * --------------------------------------------------------------------------
 * BSP 层只缓存外设句柄、DMA 缓冲和硬件状态，具体设备用途由 Robot 配置决定。
 * ========================================================================== */

/**
 * @brief CPU 主频，单位为 Hz。
 */
static uint32_t dwt_cpu_freq_hz;

/**
 * @brief CPU 主频对应的每毫秒周期数。
 */
static uint32_t dwt_cpu_freq_hz_ms;

/**
 * @brief CPU 主频对应的每微秒周期数。
 */
static uint32_t dwt_cpu_freq_hz_us;

/**
 * @brief DWT `CYCCNT` 溢出轮转次数。
 */
static uint32_t dwt_cyccnt_round_count;

/**
 * @brief 上一次记录的 DWT `CYCCNT` 值。
 */
static uint32_t dwt_cyccnt_last;

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责索引、句柄、参数和 HAL 状态检查，避免上层直接碰 CubeMX 句柄细节。
 * ========================================================================== */

/**
 * @brief 使用已读取的 DWT `CYCCNT` 值更新溢出轮转计数。
 *
 * 通过比较当前计数值与上次记录值判断 32 位周期计数器是否发生溢出。
 *
 * @param cnt_now 已读取的当前 DWT `CYCCNT` 值。
 */
static void update_counter(uint32_t cnt_now)
{
    if (cnt_now < dwt_cyccnt_last) {
        dwt_cyccnt_round_count++;
    }

    dwt_cyccnt_last = cnt_now;
}

/**
 * @brief 获取当前 64 位 DWT 周期计数。
 *
 * @return 当前累计的 64 位周期计数值。
 */
static uint64_t get_cycle_count64(void)
{
    uint32_t cnt_now = DWT->CYCCNT;

    update_counter(cnt_now);

    /* @diff: 旧版使用 UINT32_MAX 作为单轮周期会少计 1 cycle；此处按 2^32 组合 64 位计数。 */
    return (((uint64_t)dwt_cyccnt_round_count) << 32) | (uint64_t)cnt_now;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只暴露板级能力和安全钳位，Devices/Apps 通过这些接口绑定硬件。
 * ========================================================================== */
void bsp_dwt_init(uint32_t cpu_freq_mhz)
{
    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = 0U;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    dwt_cpu_freq_hz = cpu_freq_mhz * 1000000U;
    dwt_cpu_freq_hz_ms = dwt_cpu_freq_hz / 1000U;
    dwt_cpu_freq_hz_us = dwt_cpu_freq_hz / 1000000U;
    dwt_cyccnt_round_count = 0U;

    /* @diff: 旧版没有对该值进行置零，此处置零修复重初始化后的误判溢出风险。 */
    dwt_cyccnt_last = 0U;
}

float bsp_dwt_get_delta_t(uint32_t *cnt_last)
{
    uint32_t cnt_now;
    float dt;

    if ((cnt_last == NULL) || (dwt_cpu_freq_hz == 0U)) {
        /* @diff: 旧版未检查空指针和未初始化主频，新版直接返回 0。 */
        return 0.0f;
    }

    cnt_now = DWT->CYCCNT;
    dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)dwt_cpu_freq_hz);
    *cnt_last = cnt_now;

    update_counter(cnt_now);

    return dt;
}

double bsp_dwt_get_delta_t64(uint32_t *cnt_last)
{
    uint32_t cnt_now;
    double dt;

    if ((cnt_last == NULL) || (dwt_cpu_freq_hz == 0U)) {
        /* @diff: 旧版未检查空指针和未初始化主频，新版直接返回 0。 */
        return 0.0;
    }

    cnt_now = DWT->CYCCNT;
    dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)dwt_cpu_freq_hz);
    *cnt_last = cnt_now;

    update_counter(cnt_now);

    return dt;
}

void bsp_dwt_get_time(dwt_time_t *out_time)
{
    uint64_t cycle_count64;
    uint64_t seconds;
    uint64_t remain_cycles;
    uint64_t remain_cycles_after_ms;

    if ((out_time == NULL) ||
        (dwt_cpu_freq_hz == 0U) ||
        (dwt_cpu_freq_hz_ms == 0U) ||
        (dwt_cpu_freq_hz_us == 0U)) {
        /* @diff: 旧版通过全局 SysTime/CYCCNT64 输出，新版改为指针输出并保护非法入参。 */
        return;
    }

    cycle_count64 = get_cycle_count64();
    seconds = cycle_count64 / dwt_cpu_freq_hz;
    remain_cycles = cycle_count64 - (seconds * dwt_cpu_freq_hz);

    out_time->s = (uint32_t)seconds;
    out_time->ms = (uint16_t)(remain_cycles / dwt_cpu_freq_hz_ms);

    remain_cycles_after_ms = remain_cycles - ((uint64_t)out_time->ms * dwt_cpu_freq_hz_ms);
    out_time->us = (uint16_t)(remain_cycles_after_ms / dwt_cpu_freq_hz_us);
}

float bsp_dwt_get_timeline_s(void)
{
    dwt_time_t time_now = {0U};

    bsp_dwt_get_time(&time_now);

    return (float)time_now.s + ((float)time_now.ms * 0.001f) + ((float)time_now.us * 0.000001f);
}

float bsp_dwt_get_timeline_ms(void)
{
    dwt_time_t time_now = {0U};

    bsp_dwt_get_time(&time_now);

    return ((float)time_now.s * 1000.0f) + (float)time_now.ms + ((float)time_now.us * 0.001f);
}

uint64_t bsp_dwt_get_timeline_us(void)
{
    dwt_time_t time_now = {0U};

    bsp_dwt_get_time(&time_now);

    return ((uint64_t)time_now.s * 1000000ULL) + ((uint64_t)time_now.ms * 1000ULL) + (uint64_t)time_now.us;
}

void bsp_dwt_delay(float delay_s)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait_cycles_f;
    uint32_t wait_cycles;

    if (!(delay_s > 0.0f) || (dwt_cpu_freq_hz == 0U)) {
        /* @diff: 旧版未检查非正延时、NaN 和未初始化主频，新版直接返回。 */
        return;
    }

    wait_cycles_f = delay_s * (float)dwt_cpu_freq_hz;
    if (wait_cycles_f >= (float)UINT32_MAX) {
        /* @diff: 超过单轮 CYCCNT 的延时不保证严格等价，此处避免 float 到 uint32_t 的越界转换。 */
        wait_cycles = UINT32_MAX;
    } else {
        wait_cycles = (uint32_t)wait_cycles_f;
    }

    if (wait_cycles == 0U) {
        /* @diff: 正延时小于 1 cycle 时至少等待 1 cycle，更接近旧版 float 比较行为。 */
        wait_cycles = 1U;
    }

    while ((DWT->CYCCNT - tickstart) < wait_cycles) {
    }
}

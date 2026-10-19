/**
 ******************************************************************************
 * @file    bsp_dwt.h
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   DWT CYCCNT 时间基、短延时和运行时间线接口。
 *
 * @note    与旧版接口相比，本文件采用 bsp_dwt_* 命名和 dwt_time_t
 *          类型，不再导出 SysTime/CYCCNT64 全局变量。
 * @note    短周期 delta time 和短延时行为与旧版常规路径接近；边界条件
 *          增加了空指针、未初始化主频、非正延时等防御式处理。
 * @note    64 位时间线依赖调用方定期读取 DWT 状态，读取间隔不应超过
 *          一次 32 位 CYCCNT 回绕周期。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_BSP_DWT_H
#define PRINTK_BSP_DWT_H

#include "main.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * BSP 头文件描述板级外设能力、ID 和轻量配置，不描述设备用途或机器人装配语义。
 * ========================================================================== */

/**
 * @brief DWT 时间拆分结构体。
 *
 * 该结构体用于描述当前时间线的秒、毫秒和微秒部分。
 */
typedef struct
{
    uint32_t s;     /**< 秒部分。 */
    uint16_t ms;    /**< 当前秒内的毫秒部分，范围为 0~999。 */
    uint16_t us;    /**< 当前毫秒内的微秒部分，范围为 0~999。 */
} dwt_time_t;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按初始化、查询、收发和回调辅助排列，上层不应直接访问 CubeMX 句柄。
 * ========================================================================== */

/**
 * @brief 初始化 DWT 周期计数器时间基。
 *
 * 该函数会使能 Cortex-M 的 DWT `CYCCNT` 计数器，并根据输入主频建立
 * 秒、毫秒、微秒时间换算关系。
 *
 * @note 差异: 旧版未重置上一次 CYCCNT 记录值；新版初始化时会同步清零，
 *       避免运行中重初始化后误判溢出。
 *
 * @param cpu_freq_mhz CPU 主频，单位为 MHz。
 */
void bsp_dwt_init(uint32_t cpu_freq_mhz);

/**
 * @brief 计算自上次记录以来的时间差。
 *
 * @note 差异: 旧版未检查 cnt_last 和初始化状态；新版入参非法或主频未初始化时返回 0。
 *
 * @param cnt_last 输入上次记录的 `CYCCNT` 值，函数返回前会更新为当前值。
 * @return 两次采样之间的时间差，单位为秒。
 */
float bsp_dwt_get_delta_t(uint32_t *cnt_last);

/**
 * @brief 计算自上次记录以来的时间差，返回双精度结果。
 *
 * @note 差异: 旧版未检查 cnt_last 和初始化状态；新版入参非法或主频未初始化时返回 0。
 *
 * @param cnt_last 输入上次记录的 `CYCCNT` 值，函数返回前会更新为当前值。
 * @return 两次采样之间的时间差，单位为秒。
 */
double bsp_dwt_get_delta_t64(uint32_t *cnt_last);

/**
 * @brief 获取当前时间线，单位为秒。
 *
 * @return 当前时间线，单位为秒。
 */
float bsp_dwt_get_timeline_s(void);

/**
 * @brief 获取当前时间线，单位为毫秒。
 *
 * @return 当前时间线，单位为毫秒。
 */
float bsp_dwt_get_timeline_ms(void);

/**
 * @brief 获取当前时间线，单位为微秒。
 *
 * @return 当前时间线，单位为微秒。
 */
uint64_t bsp_dwt_get_timeline_us(void);

/**
 * @brief 获取当前拆分时间。
 *
 * @note 差异: 旧版通过全局 SysTime/CYCCNT64 暴露状态；新版通过 out_time 显式输出。
 *
 * @param out_time 输出的拆分时间结构体指针。
 */
void bsp_dwt_get_time(dwt_time_t *out_time);

/**
 * @brief 基于 DWT 周期计数器执行阻塞延时。
 *
 * @note 差异: 旧版未检查非正延时、NaN 和初始化状态；新版遇到非法延时或主频未初始化时返回。
 *
 * @param delay_s 延时时长，单位为秒。
 */
void bsp_dwt_delay(float delay_s);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_BSP_DWT_H

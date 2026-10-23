/**
 ******************************************************************************
 * @file    app_board_link.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   板间通信应用层接口。
 *
 * @note    本模块只负责从各 App 快照组装板间状态，并缓存接收到的底盘板
 *          状态；不读取旧全局状态，也不绑定具体 UART。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_BOARD_LINK_H
#define PRINTK_APPS_BOARD_LINK_H

#include <stdint.h>

#include "communication.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 状态与诊断类型
 * --------------------------------------------------------------------------
 * App 头文件只暴露快照、诊断和必要的注入类型，让下游按稳定接口读取状态。
 * ========================================================================== */
/**
 * @brief BoardLink 应用层诊断。
 */
typedef struct
{
    uint32_t loop_count;        /**< 任务循环次数。 */
    uint32_t state_send_count;  /**< 云台板状态发送成功次数。 */
    uint32_t state_fail_count;  /**< 云台板状态发送失败次数。 */
    uint32_t chassis_rx_count;  /**< 底盘板状态更新次数。 */
    uint32_t last_time_us;      /**< 最近一次周期时间戳低 32 位。 */
    uint32_t last_chassis_seq;  /**< 最近一次底盘状态 seq。 */
    CommunicationStatus last_status; /**< 最近一次通信状态。 */
    uint8_t ready;             /**< 非 0 表示任务已启动。 */
} AppBoardLinkDiag;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 公开接口按初始化、快照读取和外部注入排列；旧 task/global API 不再作为入口。
 * ========================================================================== */
/**
 * @brief 初始化并启动 BoardLink 任务。
 *
 * @return 成功返回 1，失败返回 0。
 */
uint8_t app_board_link_init(void);

/**
 * @brief 读取最新底盘板状态缓存。
 *
 * @param out 底盘板状态输出。
 * @return 成功返回 1，参数无效或暂无状态返回 0。
 */
uint8_t app_board_link_get_chassis(CommunicationChassisState *out);

/**
 * @brief 读取 BoardLink 诊断。
 *
 * @param out 诊断输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_board_link_get_diag(AppBoardLinkDiag *out);

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_BOARD_LINK_H */

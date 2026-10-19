/**
 ******************************************************************************
 * @file    app_magazine.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   弹舱应用层接口。
 *
 * @note    本模块负责把 Input 输出的弹舱动作命令应用到 Devices/Magazine。
 * @note    本模块不读取遥控器、不实现发射控制，也不恢复旧 shoot 任务内的弹舱调用。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_MAGAZINE_H
#define PRINTK_APPS_MAGAZINE_H

#include <stdint.h>

#include "input.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 状态与诊断类型
 * --------------------------------------------------------------------------
 * App 头文件只暴露快照、诊断和必要的注入类型，让下游按稳定接口读取状态。
 * ========================================================================== */
/**
 * @brief 弹舱应用层状态。
 */
typedef enum
{
    AppMagazineClosed = 0, /**< 弹舱关闭。 */
    AppMagazineOpen = 1,   /**< 弹舱打开。 */
    AppMagazineUnknown = 2 /**< 弹舱状态未知。 */
} AppMagazineState;

/**
 * @brief 弹舱应用层诊断。
 */
typedef struct
{
    uint32_t loop_count;      /**< 周期执行次数。 */
    uint32_t cmd_count;       /**< 接收到有效动作命令次数。 */
    uint32_t open_count;      /**< 执行打开动作成功次数。 */
    uint32_t close_count;     /**< 执行关闭动作成功次数。 */
    uint32_t io_fail_count;   /**< 设备层执行失败次数。 */
    uint32_t hold_count;      /**< 无有效动作时保持状态次数。 */
    uint32_t last_tick_ms;    /**< 最近一次周期时间戳，单位 ms。 */
    InputMagAction last_action; /**< 最近一次动作命令。 */
    AppMagazineState state;   /**< 当前弹舱状态。 */
    uint8_t input_online;     /**< 最近一次命令源在线标志。 */
    uint8_t cmd_valid;        /**< 最近一次命令有效标志。 */
    uint8_t io_ok;            /**< 最近一次设备层执行是否成功。 */
} AppMagazineDiag;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 公开接口按初始化、快照读取和外部注入排列；旧 task/global API 不再作为入口。
 * ========================================================================== */
/**
 * @brief 初始化并启动弹舱任务。
 *
 * @return 成功返回 1，失败返回 0。
 */
uint8_t app_magazine_init(void);

/**
 * @brief 应用一条弹舱命令。
 *
 * 命令无效、离线或动作为保持时不会改变弹舱状态。
 *
 * @param cmd 弹舱命令。
 * @return 执行动作成功或无需动作返回 1，设备层失败返回 0。
 */
uint8_t app_magazine_set_cmd(const InputMagCmd *cmd);

/**
 * @brief 获取当前弹舱状态快照。
 *
 * @param out 状态输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_magazine_get_state(AppMagazineState *out);

/**
 * @brief 获取弹舱应用层诊断快照。
 *
 * @param out 诊断输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_magazine_get_diag(AppMagazineDiag *out);

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_MAGAZINE_H */

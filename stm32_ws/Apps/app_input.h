/**
 ******************************************************************************
 * @file    app_input.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Input 应用层接口。
 *
 * @note    本模块负责把 RC 原始协议解析状态、视觉输入和观测状态注入
 *          Component/Input，并缓存机构命令快照。
 * @note    本模块不保留旧遥控全局状态、旧遥控 App 或旧输入映射接口。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_INPUT_H
#define PRINTK_APPS_INPUT_H

#include <stdint.h>

#include "app_communication.h"
#include "input.h"
#include "rc_dbus.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 状态与诊断类型
 * --------------------------------------------------------------------------
 * App 头文件只暴露快照、诊断和必要的注入类型，让下游按稳定接口读取状态。
 * ========================================================================== */
/**
 * @brief Input 应用层诊断信息。
 */
typedef struct
{
    uint32_t loop_count;       /**< 周期执行次数。 */
    uint32_t rc_sync_count;    /**< RC 新帧同步成功次数。 */
    uint32_t rc_lost_count;    /**< RC 超时离线次数。 */
    uint32_t rc_parse_fail;    /**< RC DBUS 解析失败次数。 */
    uint32_t input_fail_count; /**< Component/Input 返回错误次数。 */
    uint32_t vision_count;     /**< 本周期读取到有效视觉输入的累计次数。 */
    uint32_t last_tick_ms;     /**< 最近一次周期时间戳，单位 ms。 */
    uint32_t last_frame_seq;   /**< 最近一次处理的 RC 原始帧序号。 */
    RcStatus rc_status;        /**< 最近一次 DBUS 解析状态。 */
    InputStatus input_status;  /**< 最近一次 Input 状态。 */
    CommunicationStatus vision_status; /**< 最近一次视觉读取状态。 */
    uint8_t rc_online;         /**< 非 0 表示 RC 缓存在线。 */
    uint8_t vision_online;     /**< 非 0 表示视觉输入在线。 */
    uint8_t gimbal_online;     /**< 最近一次云台命令在线标志。 */
    uint8_t chassis_online;    /**< 最近一次底盘命令在线标志。 */
    uint8_t shoot_online;      /**< 最近一次发射命令在线标志。 */
    uint8_t mag_online;        /**< 最近一次弹舱命令源在线标志。 */
    uint8_t mag_valid;         /**< 最近一次弹舱动作有效标志。 */
    InputMagAction mag_action; /**< 最近一次弹舱动作。 */
    uint8_t cali_online;       /**< 最近一次校准命令源在线标志。 */
    uint8_t cali_valid;        /**< 最近一次校准动作有效标志。 */
    InputCaliAction cali_action; /**< 最近一次校准动作。 */
} AppInputDiag;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 公开接口按初始化、快照读取和外部注入排列；旧 task/global API 不再作为入口。
 * ========================================================================== */
/**
 * @brief 初始化并启动 Input 任务。
 *
 * @return 成功返回 1，失败返回 0。
 */
uint8_t app_input_init(void);

/**
 * @brief 获取当前 Input 输出快照。
 *
 * @param out 输出快照。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_input_get_output(InputOutput *out);

/**
 * @brief 获取当前统一输入参考快照。
 *
 * @param out 统一输入参考输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_input_get_ref(InputRef *out);

/**
 * @brief 获取当前云台命令快照。
 *
 * @param out 云台命令输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_input_get_gimbal_cmd(GimbalCmd *out);

/**
 * @brief 获取当前底盘命令快照。
 *
 * @param out 底盘命令输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_input_get_chassis_cmd(ChassisCmd *out);

/**
 * @brief 获取当前发射命令快照。
 *
 * @param out 发射命令输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_input_get_shoot_cmd(ShootCmd *out);

/**
 * @brief 获取当前弹舱命令快照。
 *
 * @param out 弹舱命令输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_input_get_magazine_cmd(InputMagCmd *out);

/**
 * @brief 获取当前校准命令快照。
 *
 * @param out 校准命令输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_input_get_cali_cmd(InputCaliCmd *out);

/**
 * @brief 获取 Input 应用层诊断快照。
 *
 * @param out 诊断输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_input_get_diag(AppInputDiag *out);

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_INPUT_H */

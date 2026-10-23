/**
 ******************************************************************************
 * @file    input_gimbal_rc.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   RC 到云台参考的映射接口。
 *
 * @note    本文件把遥控器物理输入解释为云台人工参考，解释规则由映射配置决定。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_INPUT_GIMBAL_RC_H
#define PRINTK_COMPONENT_INPUT_GIMBAL_RC_H

#include "input_common.h"
#include "rc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */

/**
 * @brief RC 云台映射配置。
 */
typedef struct
{
    int8_t yaw_ch;             /**< yaw 摇杆通道索引，范围 0~4，负数表示禁用。 */
    int8_t pitch_ch;           /**< pitch 摇杆通道索引，范围 0~4，负数表示禁用。 */
    int8_t mode_sw;            /**< 云台模式拨杆索引，范围 0~1，负数表示禁用。 */
    int8_t override_sw;        /**< 键盘覆盖模式要求的拨杆索引，负数表示不限制。 */
    int16_t deadband;          /**< 摇杆死区。 */
    float32_t yaw_scale;       /**< yaw 摇杆比例。 */
    float32_t pitch_scale;     /**< pitch 摇杆比例。 */
    float32_t mouse_yaw_scale; /**< 鼠标 X 到 yaw 的比例。 */
    float32_t mouse_pitch_scale; /**< 鼠标 Y 到 pitch 的比例。 */
    GimbalMode sw_up_mode;     /**< 拨杆上档对应模式。 */
    GimbalMode sw_mid_mode;    /**< 拨杆中档对应模式。 */
    GimbalMode sw_down_mode;   /**< 拨杆下档对应模式。 */
    uint16_t zero_key;         /**< 切换无力模式的键位掩码，0 表示禁用。 */
    uint16_t gyro_key;         /**< 切换陀螺仪绝对角模式的键位掩码，0 表示禁用。 */
    uint16_t encode_key;       /**< 切换编码器相对角模式的键位掩码，0 表示禁用。 */
    uint16_t raw_key;          /**< 切换原始模式的键位掩码，0 表示禁用。 */
    uint16_t turn_key;         /**< 180 度转向键位掩码，0 表示禁用。 */
    float32_t turn_speed;      /**< 180 度转向期间每周期附加 yaw 增量，单位 rad。 */
    float32_t turn_done_err;   /**< 180 度转向完成角度误差，单位 rad。 */
    uint8_t priority;          /**< 输出参考优先级。 */
} InputGimbalRcMap;

/**
 * @brief RC 云台映射上下文。
 */
typedef struct
{
    float32_t turn_target; /**< 180 度转向目标 yaw，单位 rad。 */
    uint16_t last_key;     /**< 上一次键盘按键位图，用于边沿检测。 */
    uint8_t turn_active;   /**< 非 0 表示正在执行 180 度转向参考生成。 */
} InputGimbalRcCtx;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 初始化 RC 云台映射上下文。
 *
 * @param ctx 映射上下文。
 */
void input_gimbal_rc_init(InputGimbalRcCtx *ctx);

/**
 * @brief 将 RC 物理输入映射为云台参考。
 *
 * @param ctx 映射上下文。
 * @param rc DBUS 风格物理输入状态。
 * @param map 映射配置。
 * @param yaw 当前云台 yaw 角，单位 rad。
 * @param yaw_valid 非 0 表示 yaw 可用于 180 度转向参考。
 * @param out 云台参考输出。
 * @return 执行状态。
 */
InputStatus input_gimbal_rc_map(InputGimbalRcCtx *ctx,
                                const RcDbusState *rc,
                                const InputGimbalRcMap *map,
                                float32_t yaw,
                                uint8_t yaw_valid,
                                InputGimbalRef *out);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_INPUT_GIMBAL_RC_H

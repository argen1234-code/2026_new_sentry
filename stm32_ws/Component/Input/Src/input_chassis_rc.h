/**
 ******************************************************************************
 * @file    input_chassis_rc.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   RC 到底盘参考的映射接口。
 *
 * @note    本文件把遥控器物理输入解释为底盘人工参考，解释规则由映射配置决定。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_INPUT_CHASSIS_RC_H
#define PRINTK_COMPONENT_INPUT_CHASSIS_RC_H

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
 * @brief RC 底盘映射配置。
 */
typedef struct
{
    int8_t vx_ch;             /**< vx 摇杆通道索引，范围 0~4，负数表示禁用。 */
    int8_t vy_ch;             /**< vy 摇杆通道索引，范围 0~4，负数表示禁用。 */
    int8_t wz_ch;             /**< wz 摇杆通道索引，范围 0~4，负数表示禁用。 */
    int8_t mode_sw;           /**< 底盘模式拨杆索引，范围 0~1，负数表示禁用。 */
    int8_t follow_sw;         /**< 底盘跟随拨杆索引，范围 0~1，负数表示禁用。 */
    int8_t override_sw;       /**< 键盘覆盖模式要求的拨杆索引，负数表示不限制。 */
    int16_t deadband;         /**< 摇杆死区。 */
    float32_t vx_scale;       /**< vx 摇杆比例。 */
    float32_t vy_scale;       /**< vy 摇杆比例。 */
    float32_t wz_scale;       /**< wz 摇杆比例。 */
    float32_t key_vx_max;     /**< 键盘前进速度参考。 */
    float32_t key_vx_min;     /**< 键盘后退速度参考。 */
    float32_t key_vy_max;     /**< 键盘左移速度参考。 */
    float32_t key_vy_min;     /**< 键盘右移速度参考。 */
    float32_t key_wz;         /**< 键盘旋转速度参考幅值。 */
    float32_t vx_lpf_num;     /**< vx 一阶低通参数，0 表示关闭滤波。 */
    float32_t vy_lpf_num;     /**< vy 一阶低通参数，0 表示关闭滤波。 */
    float32_t period_s;       /**< 输入映射周期，单位 s。 */
    float32_t spin_wz;        /**< 小陀螺模式 wz 参考。 */
    float32_t swing_move_angle; /**< 运动时摆动 yaw 幅值，单位 rad。 */
    float32_t swing_stop_angle; /**< 静止时摆动 yaw 幅值，单位 rad。 */
    float32_t swing_step;     /**< 摆动相位每周期增量，单位 rad。 */
    uint16_t front_key;       /**< 前进键位掩码，0 表示禁用。 */
    uint16_t back_key;        /**< 后退键位掩码，0 表示禁用。 */
    uint16_t left_key;        /**< 左移键位掩码，0 表示禁用。 */
    uint16_t right_key;       /**< 右移键位掩码，0 表示禁用。 */
    uint16_t turn_l_key;      /**< 左转键位掩码，0 表示禁用。 */
    uint16_t turn_r_key;      /**< 右转键位掩码，0 表示禁用。 */
    uint16_t spin_key;        /**< 小陀螺模式键位掩码，0 表示禁用。 */
    uint16_t stop_key;        /**< 停止模式键位掩码，0 表示禁用。 */
    uint16_t follow_key;      /**< 跟随云台 yaw 模式键位掩码，0 表示禁用。 */
    uint16_t swing_key;       /**< 摆动模式键位掩码，0 表示禁用。 */
    uint16_t jump_key;        /**< 跳跃键位掩码，0 表示禁用。 */
    ChassisMode sw_up_mode;   /**< 拨杆上档对应模式。 */
    ChassisMode sw_mid_mode;  /**< 拨杆中档对应模式。 */
    ChassisMode sw_down_mode; /**< 拨杆下档对应模式。 */
    ChassisMode follow_down_mode; /**< 跟随拨杆下档时的底盘模式。 */
    ChassisMode follow_else_mode; /**< 跟随拨杆非下档时的底盘模式。 */
    uint8_t priority;         /**< 输出参考优先级。 */
} InputChassisRcMap;

/**
 * @brief RC 底盘映射上下文。
 */
typedef struct
{
    float32_t vx_out;      /**< vx 一阶低通输出。 */
    float32_t vy_out;      /**< vy 一阶低通输出。 */
    float32_t swing_phase; /**< 摆动相位，单位 rad。 */
    uint8_t swing_active;  /**< 非 0 表示摆动键处于按下状态。 */
} InputChassisRcCtx;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 初始化 RC 底盘映射上下文。
 *
 * @param ctx 映射上下文。
 */
void input_chassis_rc_init(InputChassisRcCtx *ctx);

/**
 * @brief 将 RC 物理输入映射为底盘参考。
 *
 * @param ctx 映射上下文。
 * @param rc DBUS 风格物理输入状态。
 * @param map 映射配置。
 * @param out 底盘参考输出。
 * @return 执行状态。
 */
InputStatus input_chassis_rc_map(InputChassisRcCtx *ctx,
                                 const RcDbusState *rc,
                                 const InputChassisRcMap *map,
                                 InputChassisRef *out);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_INPUT_CHASSIS_RC_H

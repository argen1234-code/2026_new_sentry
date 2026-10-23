/**
 ******************************************************************************
 * @file    gimbal_direct_yaw_pitch.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   yaw-pitch 直驱云台构型接口。
 *
 * @note    本构型只描述直驱 yaw/pitch 云台本体，不读取任务、遥控、INS、CAN
 *          或发射机构全局状态。
 * @note    运行时反馈、命令和时间由上层通过 GimbalInput 注入，输出由
 *          GimbalOutput 返回给上层发送。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_MECHANISM_GIMBAL_DIRECT_YAW_PITCH_H
#define PRINTK_MECHANISM_GIMBAL_DIRECT_YAW_PITCH_H

#include "gimbal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */

/**
 * @brief 直驱轴 PID 运行状态。
 */
typedef struct
{
    GimbalPidCfg cfg;  /**< PID 参数。 */
    float32_t set;     /**< 设定值。 */
    float32_t get;     /**< 反馈值。 */
    float32_t err;     /**< 本周期误差。 */
    float32_t last_err;/**< 上周期误差。 */
    float32_t iout;    /**< 积分输出。 */
    float32_t out;     /**< 总输出。 */
} GimbalDirectPid;

/**
 * @brief 直驱云台单轴状态。
 */
typedef struct
{
    GimbalMotorFb fb;              /**< 最近一次电机反馈。 */
    GimbalDirectPid abs_pid;       /**< 绝对角外环。 */
    GimbalDirectPid rel_pid;       /**< 相对角外环。 */
    GimbalDirectPid speed_pid;     /**< 速度环。 */
    GimbalMode mode;               /**< 当前控制模式。 */
    GimbalMode last_mode;          /**< 上一次控制模式。 */
    uint16_t offset_ecd;           /**< 机械中值编码器。 */
    float32_t max_relative;        /**< 最大相对角，单位 rad。 */
    float32_t min_relative;        /**< 最小相对角，单位 rad。 */
    float32_t relative;            /**< 当前相对角，单位 rad。 */
    float32_t relative_set;        /**< 相对角目标，单位 rad。 */
    float32_t absolute;            /**< 当前绝对角，单位 rad。 */
    float32_t absolute_set;        /**< 绝对角目标，单位 rad。 */
    float32_t gyro;                /**< 当前轴角速度，单位 rad/s。 */
    float32_t ref_rate;            /**< 上层目标生成出的参考角速度，单位 rad/s。 */
    float32_t gyro_set;            /**< 轴角速度目标，单位 rad/s。 */
    float32_t speed_rpm;           /**< 电机转速，单位 rpm。 */
    float32_t raw_current;         /**< RAW 模式电流目标。 */
    float32_t current_set;         /**< 浮点电流输出。 */
    int16_t given_current;         /**< int16 电流输出。 */
    uint8_t limit_valid;           /**< 非 0 表示相对角限位有效。 */
} GimbalDirectAxis;

/**
 * @brief yaw-pitch 直驱云台私有上下文。
 */
typedef struct
{
    GimbalDirectAxis yaw;      /**< yaw 轴状态。 */
    GimbalDirectAxis pitch;    /**< pitch 轴状态。 */
    GimbalCali cali;           /**< 当前校准结果。 */
    uint8_t feedback_ready;    /**< 非 0 表示已用反馈同步过目标。 */
} GimbalDirectCtx;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 初始化 yaw-pitch 直驱云台。
 *
 * @param gimbal 公共云台上下文。
 * @param impl GimbalDirectCtx 私有上下文。
 * @return 执行状态。
 */
GimbalStatus gimbal_direct_init(Gimbal *gimbal, void *impl);

/**
 * @brief 执行一次 yaw-pitch 直驱云台控制周期。
 *
 * @param gimbal 公共云台上下文。
 * @param impl GimbalDirectCtx 私有上下文。
 * @param input 周期输入。
 * @param output 周期输出。
 * @return 执行状态。
 */
GimbalStatus gimbal_direct_step(Gimbal *gimbal,
                                void *impl,
                                const GimbalInput *input,
                                GimbalOutput *output);

/**
 * @brief 更新 yaw-pitch 直驱云台反馈。
 *
 * @param gimbal 公共云台上下文。
 * @param impl GimbalDirectCtx 私有上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
GimbalStatus gimbal_direct_update(Gimbal *gimbal, void *impl, const GimbalInput *input);

/**
 * @brief 执行 yaw-pitch 直驱云台控制计算。
 *
 * @param gimbal 公共云台上下文。
 * @param impl GimbalDirectCtx 私有上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
GimbalStatus gimbal_direct_control(Gimbal *gimbal, void *impl, const GimbalInput *input);

/**
 * @brief 生成 yaw-pitch 直驱云台输出。
 *
 * @param gimbal 公共云台上下文。
 * @param impl GimbalDirectCtx 私有上下文。
 * @param output 周期输出。
 * @return 执行状态。
 */
GimbalStatus gimbal_direct_output(const Gimbal *gimbal, void *impl, GimbalOutput *output);

/**
 * @brief 将 yaw-pitch 直驱云台目标同步到当前反馈。
 *
 * @param gimbal 公共云台上下文。
 * @param impl GimbalDirectCtx 私有上下文。
 * @return 执行状态。
 */
GimbalStatus gimbal_direct_sync(Gimbal *gimbal, void *impl);

/**
 * @brief 写入 yaw-pitch 直驱云台校准结果。
 *
 * @param gimbal 公共云台上下文。
 * @param impl GimbalDirectCtx 私有上下文。
 * @param cali 校准结果。
 * @return 执行状态。
 */
GimbalStatus gimbal_direct_set_cali(Gimbal *gimbal, void *impl, const GimbalCali *cali);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_MECHANISM_GIMBAL_DIRECT_YAW_PITCH_H

/**
 ******************************************************************************
 * @file    aim.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   自瞄参考整形组件公共接口。
 *
 * @note    本组件只把视觉误差整形为云台控制增量，不读取遥控器、通信设备、
 *          HAL tick 或云台控制器内部状态。
 * @note    HostAim.yaw_rad/pitch_rad 在本组件内按视觉误差处理，单位为 rad。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_AIM_H
#define PRINTK_COMPONENT_AIM_H

#include <stdint.h>

#include "arm_math.h"
#include "gimbal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */
/** @brief yaw 视觉误差有效位。 */
#define AIM_YAW_VALID ((uint32_t)0x00000001UL)

/** @brief pitch 视觉误差有效位。 */
#define AIM_PITCH_VALID ((uint32_t)0x00000002UL)

/** @brief 默认最小 dt，单位 s。 */
#ifndef AIM_DT_MIN_DEFAULT
#define AIM_DT_MIN_DEFAULT 0.0005F
#endif

/** @brief 默认最大 dt，单位 s。 */
#ifndef AIM_DT_MAX_DEFAULT
#define AIM_DT_MAX_DEFAULT 0.0200F
#endif

/** @brief 默认 dt，单位 s。 */
#ifndef AIM_DT_DEFAULT
#define AIM_DT_DEFAULT 0.0020F
#endif

/** @brief 默认目标附近加速度软区，单位 rad。 */
#ifndef AIM_ACCEL_SOFT_ZONE_DEFAULT
#define AIM_ACCEL_SOFT_ZONE_DEFAULT (10.0F * 3.14159265358979323846F / 180.0F)
#endif

/** @brief 默认启用旧自瞄 5 点中值滤波。 */
#ifndef AIM_MEDIAN_ENABLE_DEFAULT
#define AIM_MEDIAN_ENABLE_DEFAULT 1U
#endif

/** @brief 默认允许连续 0 值保持最近有效输出的次数。 */
#ifndef AIM_ZERO_HOLD_DEFAULT
#define AIM_ZERO_HOLD_DEFAULT 2U
#endif

/**
 * @brief Aim 执行状态。
 */
typedef enum
{
    AimOk = 0,     /**< 执行成功。 */
    AimNull = 1,   /**< 输入指针为空。 */
    AimArg = 2,    /**< 配置参数非法。 */
    AimValue = 3,  /**< 输入数值非法。 */
    AimUninit = 4  /**< 上下文未初始化。 */
} AimStatus;

/**
 * @brief 单轴自瞄整形参数。
 */
typedef struct
{
    float32_t kv;        /**< 误差到角速度的比例系数。 */
    float32_t kd;        /**< 误差变化率阻尼系数。 */
    float32_t omega_max; /**< 最大输出角速度绝对值，单位 rad/s。 */
    float32_t accel_max; /**< 最大角加速度绝对值，单位 rad/s^2。 */
} AimAxisCfg;

/**
 * @brief Aim 配置。
 */
typedef struct
{
    AimAxisCfg yaw;          /**< yaw 轴整形参数。 */
    AimAxisCfg pitch;        /**< pitch 轴整形参数。 */
    float32_t err_soft_zone; /**< 误差软区，单位 rad。 */
    float32_t accel_soft_zone; /**< 加速度软区，单位 rad。 */
    float32_t d_lpf_alpha;   /**< 误差变化率一阶低通系数，范围 0~1。 */
    float32_t eps_angle;     /**< 到位误差阈值，单位 rad。 */
    float32_t eps_omega;     /**< 到位角速度阈值，单位 rad/s。 */
    float32_t dt_min;        /**< 最小 dt 钳位，单位 s。 */
    float32_t dt_max;        /**< 最大 dt 钳位，单位 s。 */
    float32_t dt_default;    /**< 默认 dt，单位 s。 */
    GimbalMode mode;         /**< 输出给云台的模式。 */
    uint8_t median_enable;   /**< 非 0 表示对输出增量执行 5 点中值滤波。 */
    uint8_t zero_hold_count; /**< 视觉短暂 0 值时保持最近有效输出的最大次数。 */
} AimCfg;

/**
 * @brief Aim 周期输入。
 */
typedef struct
{
    float32_t yaw_err;       /**< yaw 视觉误差，单位 rad。 */
    float32_t pitch_err;     /**< pitch 视觉误差，单位 rad。 */
    float32_t dt;            /**< 本周期时间间隔，单位 s。 */
    uint32_t valid_mask;     /**< AIM_* 有效位。 */
    uint32_t timestamp_us;   /**< 输入时间戳，单位 us。 */
    uint8_t enable;          /**< 非 0 表示自瞄参与输出。 */
} AimInput;

/**
 * @brief Aim 周期输出。
 */
typedef struct
{
    float32_t yaw;           /**< yaw 控制增量，单位 rad/周期。 */
    float32_t pitch;         /**< pitch 控制增量，单位 rad/周期。 */
    GimbalMode mode;         /**< 云台模式参考。 */
    uint32_t valid_mask;     /**< AIM_* 有效位。 */
    uint32_t timestamp_us;   /**< 输出时间戳，单位 us。 */
    uint8_t online;          /**< 非 0 表示输出有效。 */
} AimOutput;

/**
 * @brief Aim 单轴运行状态。
 */
typedef struct
{
    float32_t last_err;      /**< 上一次误差，单位 rad。 */
    float32_t d_lpf;         /**< 低通后的误差变化率。 */
    float32_t omega_cmd;     /**< 当前角速度命令，单位 rad/s。 */
    float32_t median_buf[5]; /**< 5 点中值滤波环形缓存。 */
    float32_t median_last;   /**< 最近一次中值滤波有效输出。 */
    uint8_t median_idx;      /**< 中值滤波写入位置。 */
    uint8_t zero_hold_cnt;   /**< 连续 0 值保持计数。 */
    uint8_t initialized;     /**< 非 0 表示已有历史误差。 */
} AimAxis;

/**
 * @brief Aim 上下文。
 */
typedef struct
{
    AimCfg cfg;              /**< 配置副本。 */
    AimAxis yaw;             /**< yaw 轴状态。 */
    AimAxis pitch;           /**< pitch 轴状态。 */
    AimOutput output;        /**< 最近一次输出。 */
    uint8_t initialized;     /**< 非 0 表示上下文已初始化。 */
} Aim;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 填充 Aim 默认配置。
 *
 * @param cfg 配置输出。
 * @return 执行状态。
 */
AimStatus aim_default_cfg(AimCfg *cfg);

/**
 * @brief 初始化 Aim 上下文。
 *
 * @param aim Aim 上下文。
 * @param cfg 配置参数；为空时使用默认配置。
 * @return 执行状态。
 */
AimStatus aim_init(Aim *aim, const AimCfg *cfg);

/**
 * @brief 重置 Aim 运行状态。
 *
 * @param aim Aim 上下文。
 * @return 执行状态。
 */
AimStatus aim_reset(Aim *aim);

/**
 * @brief 执行一次自瞄误差整形。
 *
 * @param aim Aim 上下文。
 * @param input 周期输入。
 * @param output 周期输出。
 * @return 执行状态。
 */
AimStatus aim_step(Aim *aim, const AimInput *input, AimOutput *output);

/**
 * @brief 读取最近一次 Aim 输出。
 *
 * @param aim Aim 上下文。
 * @param output 输出缓存。
 * @return 执行状态。
 */
AimStatus aim_get(const Aim *aim, AimOutput *output);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_AIM_H

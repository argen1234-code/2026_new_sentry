/**
 ******************************************************************************
 * @file    shoot.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   发射机构公共接口。
 *
 * @note    本文件定义不同发射构型共享的输入、输出、配置和私有上下文绑定语言。
 * @note    具体构型由 SHOOT_TYPE 宏在编译期选中，不支持运行时切换机械构型。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_MECHANISM_SHOOT_H
#define PRINTK_MECHANISM_SHOOT_H

#include "arm_math.h"
#include "robot.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */
/** @brief 无发射机构构型宏。 */
#define SHOOT_TYPE_NONE 0U

/** @brief 单组摩擦轮发射构型宏，对应旧 shoot_3508。 */
#define SHOOT_TYPE_MONO 1U

/** @brief 双组摩擦轮发射构型宏，对应旧 shoot_double。 */
#define SHOOT_TYPE_DOUBLE 2U

/** @brief 默认发射构型宏，工程可在编译选项或配置头中覆盖。 */
#ifndef SHOOT_TYPE
#define SHOOT_TYPE SHOOT_TYPE_MONO
#endif

/** @brief 发射机构最大摩擦轮数量。 */
#define SHOOT_FRIC_MAX 4U

/** @brief 默认 trigger 编码器范围。 */
#ifndef SHOOT_ECD_RANGE_DEFAULT
#define SHOOT_ECD_RANGE_DEFAULT 8192
#endif

/** @brief 默认 trigger 编码器半圈阈值。 */
#ifndef SHOOT_ECD_HALF_DEFAULT
#define SHOOT_ECD_HALF_DEFAULT 4096
#endif

/** @brief 默认 trigger 编码器 tick 到 rad 的换算系数。 */
#ifndef SHOOT_ECD_TO_RAD_DEFAULT
#define SHOOT_ECD_TO_RAD_DEFAULT 0.0000213019F
#endif

/** @brief 默认 trigger rpm 到 rad/s 的换算系数。 */
#ifndef SHOOT_RPM_TO_RAD_DEFAULT
#define SHOOT_RPM_TO_RAD_DEFAULT 0.0029088821F
#endif

/** @brief 默认摩擦轮 rpm 到控制速度的换算系数。 */
#ifndef SHOOT_FRIC_RPM_TO_SPEED_DEFAULT
#define SHOOT_FRIC_RPM_TO_SPEED_DEFAULT 1.0F
#endif

/** @brief 默认摩擦轮目标速度。 */
#ifndef SHOOT_FRIC_SPEED_DEFAULT
#define SHOOT_FRIC_SPEED_DEFAULT 0.0F
#endif

/** @brief 默认 trigger 每发推进角度，单位 rad。 */
#ifndef SHOOT_TRIGGER_STEP_DEFAULT
#define SHOOT_TRIGGER_STEP_DEFAULT 0.6283185307F
#endif

/** @brief 默认 trigger 到位误差，单位 rad。 */
#ifndef SHOOT_TRIGGER_DONE_ERR_DEFAULT
#define SHOOT_TRIGGER_DONE_ERR_DEFAULT 0.05F
#endif

/** @brief 默认 trigger 目标速度，单位 rad/s。 */
#ifndef SHOOT_TRIGGER_SPEED_DEFAULT
#define SHOOT_TRIGGER_SPEED_DEFAULT 8.0F
#endif

/** @brief 默认堵转速度阈值，单位 rad/s。 */
#ifndef SHOOT_BLOCK_SPEED_DEFAULT
#define SHOOT_BLOCK_SPEED_DEFAULT 0.2F
#endif

/** @brief 默认堵转确认周期数。 */
#ifndef SHOOT_BLOCK_TIME_DEFAULT
#define SHOOT_BLOCK_TIME_DEFAULT 80U
#endif

/** @brief 默认反转周期数。 */
#ifndef SHOOT_REVERSE_TIME_DEFAULT
#define SHOOT_REVERSE_TIME_DEFAULT 80U
#endif

/** @brief 默认摩擦轮达速比例。 */
#ifndef SHOOT_FRIC_READY_RATIO_DEFAULT
#define SHOOT_FRIC_READY_RATIO_DEFAULT 0.90F
#endif

/** @brief 默认每发热量消耗。 */
#ifndef SHOOT_HEAT_PER_BULLET_DEFAULT
#define SHOOT_HEAT_PER_BULLET_DEFAULT 10.0F
#endif

/** @brief 默认热量冷却速度，单位 heat/s。 */
#ifndef SHOOT_HEAT_COOL_DEFAULT
#define SHOOT_HEAT_COOL_DEFAULT 0.0F
#endif

/**
 * @brief 发射执行状态。
 */
typedef enum
{
    ShootOk = 0,    /**< 执行成功。 */
    ShootNull = 1,  /**< 输入指针为空。 */
    ShootArg = 2,   /**< 输入参数非法。 */
    ShootState = 3, /**< 当前上下文状态不允许执行。 */
    ShootValue = 4  /**< 输入数值非法。 */
} ShootStatus;

/**
 * @brief 发射构型。
 */
typedef enum
{
    ShootTypeNone = SHOOT_TYPE_NONE,     /**< 无发射机构。 */
    ShootTypeMono = SHOOT_TYPE_MONO,     /**< 单组摩擦轮构型。 */
    ShootTypeDouble = SHOOT_TYPE_DOUBLE  /**< 双组摩擦轮构型。 */
} ShootType;

/**
 * @brief 发射模式。
 */
typedef enum
{
    ShootModeStop = 0,        /**< 停止模式。 */
    ShootModeReadyFric = 1,   /**< 摩擦轮启动中。 */
    ShootModeReadyBullet = 2, /**< 摩擦轮已达速，允许发弹。 */
    ShootModeBullet = 3,      /**< 单发模式。 */
    ShootModeContinue = 4,    /**< 连发模式。 */
    ShootModeDone = 5         /**< 单发完成或热量不足。 */
} ShootMode;

/**
 * @brief 发射 PID 参数。
 */
typedef struct
{
    float32_t kp;       /**< 比例增益。 */
    float32_t ki;       /**< 积分增益。 */
    float32_t kd;       /**< 微分增益。 */
    float32_t max_out;  /**< 输出绝对值限幅。 */
    float32_t max_iout; /**< 积分输出绝对值限幅。 */
} ShootPidCfg;

/**
 * @brief 发射电机反馈。
 */
typedef struct
{
    uint16_t ecd;       /**< 编码器原始值。 */
    int16_t speed_rpm;  /**< 转速反馈，单位 rpm。 */
    int16_t current;    /**< 电流反馈。 */
    uint8_t temperature;/**< 电机温度，单位摄氏度。 */
    uint8_t online;     /**< 非 0 表示反馈在线。 */
} ShootMotorFb;

/**
 * @brief 发射上层命令。
 */
typedef struct
{
    uint8_t online;      /**< 非 0 表示命令源在线。 */
    uint8_t force_stop;  /**< 非 0 表示强制停止发射。 */
    uint8_t fric_enable; /**< 非 0 表示启动摩擦轮。 */
    uint8_t fire_once;   /**< 非 0 表示请求单发，上升沿有效。 */
    uint8_t fire_hold;   /**< 非 0 表示请求连发。 */
} ShootCmd;

/**
 * @brief 发射周期输入。
 */
typedef struct
{
    ShootCmd cmd;                         /**< 上层控制命令。 */
    ShootMotorFb trigger_fb;              /**< 拨弹电机反馈。 */
    ShootMotorFb fric_fb[SHOOT_FRIC_MAX]; /**< 摩擦轮电机反馈。 */
    float32_t heat;                       /**< 当前热量，可为裁判系统注入值。 */
    float32_t heat_limit;                 /**< 当前热量上限。 */
    uint32_t tick_ms;                     /**< 当前时间戳，单位 ms。 */
} ShootInput;

/**
 * @brief 发射周期输出。
 */
typedef struct
{
    int16_t trigger_current;              /**< 拨弹电机发送电流。 */
    int16_t fric_current[SHOOT_FRIC_MAX]; /**< 摩擦轮发送电流。 */
    ShootMode mode;                       /**< 本周期实际模式。 */
    uint8_t send_enable;                  /**< 非 0 表示输出允许发送。 */
} ShootOutput;

/**
 * @brief 发射配置。
 */
typedef struct
{
    ShootType type;                         /**< 当前构型类型。 */
    ShootPidCfg trigger_pid;                /**< 拨弹电机速度 PID。 */
    ShootPidCfg fric_pid;                   /**< 摩擦轮速度 PID。 */
    float32_t trigger_ecd_to_rad;           /**< 拨弹编码器 tick 到 rad 的换算系数。 */
    int32_t trigger_ecd_range;              /**< 拨弹编码器范围。 */
    int32_t trigger_half_ecd_range;         /**< 拨弹编码器半圈阈值。 */
    float32_t trigger_rpm_to_rad;           /**< 拨弹 rpm 到 rad/s 的换算系数。 */
    float32_t fric_rpm_to_speed;            /**< 摩擦轮 rpm 到控制速度的换算系数。 */
    float32_t trigger_step;                 /**< 每发推进角，单位 rad。 */
    float32_t trigger_done_err;             /**< 发弹到位误差，单位 rad。 */
    float32_t trigger_speed;                /**< 发弹时拨弹目标速度，单位 rad/s。 */
    float32_t block_speed;                  /**< 堵转速度阈值，单位 rad/s。 */
    uint16_t block_time;                    /**< 堵转确认周期数。 */
    uint16_t reverse_time;                  /**< 堵转反转周期数。 */
    float32_t heat_per_bullet;              /**< 每发热量消耗。 */
    float32_t heat_cool;                    /**< 热量冷却速度，单位 heat/s。 */
    float32_t fric_ready_ratio;             /**< 摩擦轮达速比例。 */
    float32_t fric_speed[SHOOT_FRIC_MAX];   /**< 摩擦轮目标速度。 */
    int8_t fric_polarity[SHOOT_FRIC_MAX];   /**< 摩擦轮电流极性。 */
    int8_t trigger_polarity;                /**< 拨弹电机电流极性。 */
} ShootCfg;

/**
 * @brief 发射公共上下文。
 */
typedef struct
{
    ShootCfg cfg;       /**< 公共配置副本。 */
    void *impl;         /**< 当前编译期构型的私有上下文。 */
    uint8_t initialized;/**< 非 0 表示公共上下文已初始化。 */
} Shoot;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 初始化当前编译期构型的发射上下文。
 *
 * @param shoot 发射公共上下文。
 * @param cfg 公共配置。
 * @param impl 当前构型私有上下文，可由具体构型决定是否允许为空。
 * @return 执行状态。
 */
ShootStatus shoot_init(Shoot *shoot, const ShootCfg *cfg, void *impl);

/**
 * @brief 执行一次完整发射周期。
 *
 * @param shoot 发射公共上下文。
 * @param input 周期输入。
 * @param output 周期输出。
 * @return 执行状态。
 */
ShootStatus shoot_step(Shoot *shoot, const ShootInput *input, ShootOutput *output);

/**
 * @brief 更新发射反馈。
 *
 * @param shoot 发射公共上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
ShootStatus shoot_update(Shoot *shoot, const ShootInput *input);

/**
 * @brief 执行发射控制计算。
 *
 * @param shoot 发射公共上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
ShootStatus shoot_control(Shoot *shoot, const ShootInput *input);

/**
 * @brief 生成发射输出。
 *
 * @param shoot 发射公共上下文。
 * @param output 周期输出。
 * @return 执行状态。
 */
ShootStatus shoot_output(const Shoot *shoot, ShootOutput *output);

/**
 * @brief 将发射控制目标同步到当前反馈状态。
 *
 * @param shoot 发射公共上下文。
 * @return 执行状态。
 */
ShootStatus shoot_sync(Shoot *shoot);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_MECHANISM_SHOOT_H

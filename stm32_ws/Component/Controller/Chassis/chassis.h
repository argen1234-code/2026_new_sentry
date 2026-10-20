/**
 ******************************************************************************
 * @file    chassis.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   底盘机构公共接口。
 *
 * @note    本文件定义不同底盘构型共享的输入、输出、配置和私有上下文绑定语言。
 * @note    具体构型由 CHASSIS_TYPE 宏在编译期选中，不支持运行时切换机械构型。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_MECHANISM_CHASSIS_H
#define PRINTK_MECHANISM_CHASSIS_H

#include "arm_math.h"
#include "power_ctrl.h"
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
/** @brief 麦克纳姆轮底盘构型宏。 */
#define CHASSIS_TYPE_MECANUM 1U

/** @brief 全向轮底盘构型宏。 */
#define CHASSIS_TYPE_OMNI 2U

/** @brief 舵轮底盘构型宏。 */
#define CHASSIS_TYPE_STEERING 3U

/** @brief 平衡底盘构型宏。 */
#define CHASSIS_TYPE_BALANCE 4U

/** @brief 默认底盘构型宏，工程可在编译选项或配置头中覆盖。 */
#ifndef CHASSIS_TYPE
#define CHASSIS_TYPE CHASSIS_TYPE_STEERING
#endif

/** @brief 底盘最大驱动轮数量。 */
#define CHASSIS_DRIVE_MAX 4U

/** @brief 底盘最大舵向电机数量。 */
#define CHASSIS_STEER_MAX 4U

/** @brief 平衡底盘最大关节电机数量。 */
#define CHASSIS_JOINT_MAX 4U

/** @brief 平衡底盘最大轮毂电机数量。 */
#define CHASSIS_WHEEL_MAX 2U

/** @brief 默认底盘驱动电机电流限幅。 */
#ifndef CHASSIS_CURRENT_MAX_DEFAULT
#define CHASSIS_CURRENT_MAX_DEFAULT 16000.0F
#endif

/** @brief 默认舵向电机电流限幅。 */
#ifndef CHASSIS_STEER_CURRENT_MAX_DEFAULT
#define CHASSIS_STEER_CURRENT_MAX_DEFAULT 12000.0F
#endif

/** @brief 默认关节电机力矩限幅。 */
#ifndef CHASSIS_JOINT_TORQUE_MAX_DEFAULT
#define CHASSIS_JOINT_TORQUE_MAX_DEFAULT 20.0F
#endif

/** @brief 默认轮毂电机力矩限幅。 */
#ifndef CHASSIS_WHEEL_TORQUE_MAX_DEFAULT
#define CHASSIS_WHEEL_TORQUE_MAX_DEFAULT 10.0F
#endif

/** @brief 默认驱动轮速度限幅。 */
#ifndef CHASSIS_WHEEL_SPEED_MAX_DEFAULT
#define CHASSIS_WHEEL_SPEED_MAX_DEFAULT 8.0F
#endif

/** @brief 默认底盘功率上限。 */
#ifndef CHASSIS_POWER_LIMIT_DEFAULT
#ifdef ROBOT_CHASSIS_POWER_LIMIT
#define CHASSIS_POWER_LIMIT_DEFAULT ROBOT_CHASSIS_POWER_LIMIT
#else
#define CHASSIS_POWER_LIMIT_DEFAULT 0.0F
#endif
#endif

/** @brief 默认驱动轮速度 PID KP。 */
#ifndef CHASSIS_DRIVE_PID_KP_DEFAULT
#define CHASSIS_DRIVE_PID_KP_DEFAULT 1000.0F
#endif

/** @brief 默认驱动轮速度 PID KI。 */
#ifndef CHASSIS_DRIVE_PID_KI_DEFAULT
#define CHASSIS_DRIVE_PID_KI_DEFAULT 0.0F
#endif

/** @brief 默认驱动轮速度 PID KD。 */
#ifndef CHASSIS_DRIVE_PID_KD_DEFAULT
#define CHASSIS_DRIVE_PID_KD_DEFAULT 0.0F
#endif

/** @brief 默认驱动轮速度 PID 输出限幅。 */
#ifndef CHASSIS_DRIVE_PID_MAX_DEFAULT
#define CHASSIS_DRIVE_PID_MAX_DEFAULT CHASSIS_CURRENT_MAX_DEFAULT
#endif

/** @brief 默认舵向角度 PID KP。 */
#ifndef CHASSIS_STEER_ANGLE_KP_DEFAULT
#define CHASSIS_STEER_ANGLE_KP_DEFAULT 10.0F
#endif

/** @brief 默认舵向角度 PID 输出限幅。 */
#ifndef CHASSIS_STEER_ANGLE_MAX_DEFAULT
#define CHASSIS_STEER_ANGLE_MAX_DEFAULT 1000.0F
#endif

/** @brief 默认舵向速度 PID KP。 */
#ifndef CHASSIS_STEER_SPEED_KP_DEFAULT
#define CHASSIS_STEER_SPEED_KP_DEFAULT 10.0F
#endif

/** @brief 默认舵向速度 PID 输出限幅。 */
#ifndef CHASSIS_STEER_SPEED_MAX_DEFAULT
#define CHASSIS_STEER_SPEED_MAX_DEFAULT CHASSIS_STEER_CURRENT_MAX_DEFAULT
#endif

/** @brief 默认 yaw 跟随 PID KP。 */
#ifndef CHASSIS_YAW_PID_KP_DEFAULT
#define CHASSIS_YAW_PID_KP_DEFAULT 10.0F
#endif

/** @brief 默认 yaw 跟随 PID KD。 */
#ifndef CHASSIS_YAW_PID_KD_DEFAULT
#define CHASSIS_YAW_PID_KD_DEFAULT 0.1F
#endif

/** @brief 默认 yaw 跟随 PID 输出限幅。 */
#ifndef CHASSIS_YAW_PID_MAX_DEFAULT
#define CHASSIS_YAW_PID_MAX_DEFAULT 6.0F
#endif

/** @brief 默认腿长 PID KP。 */
#ifndef CHASSIS_LEG_PID_KP_DEFAULT
#define CHASSIS_LEG_PID_KP_DEFAULT 500.0F
#endif

/** @brief 默认腿长 PID KD。 */
#ifndef CHASSIS_LEG_PID_KD_DEFAULT
#define CHASSIS_LEG_PID_KD_DEFAULT 3500.0F
#endif

/** @brief 默认腿长 PID 输出限幅。 */
#ifndef CHASSIS_LEG_PID_MAX_DEFAULT
#define CHASSIS_LEG_PID_MAX_DEFAULT 120.0F
#endif

/** @brief 默认 roll 支撑 PID KP。 */
#ifndef CHASSIS_ROLL_PID_KP_DEFAULT
#define CHASSIS_ROLL_PID_KP_DEFAULT 100.0F
#endif

/** @brief 默认 roll 支撑 PID 输出限幅。 */
#ifndef CHASSIS_ROLL_PID_MAX_DEFAULT
#define CHASSIS_ROLL_PID_MAX_DEFAULT 3.5F
#endif

/** @brief 默认转向 PID KP。 */
#ifndef CHASSIS_TURN_PID_KP_DEFAULT
#define CHASSIS_TURN_PID_KP_DEFAULT 1.2F
#endif

/** @brief 默认转向 PID KD。 */
#ifndef CHASSIS_TURN_PID_KD_DEFAULT
#define CHASSIS_TURN_PID_KD_DEFAULT 0.2F
#endif

/** @brief 默认转向 PID 输出限幅。 */
#ifndef CHASSIS_TURN_PID_MAX_DEFAULT
#define CHASSIS_TURN_PID_MAX_DEFAULT 2.0F
#endif

/** @brief 默认驱动电机 rpm 到速度的换算系数。 */
#ifndef CHASSIS_RPM_TO_SPEED_DEFAULT
#define CHASSIS_RPM_TO_SPEED_DEFAULT 1.0F
#endif

/** @brief 默认舵向电机 rpm 到速度的换算系数。 */
#ifndef CHASSIS_STEER_RPM_TO_SPEED_DEFAULT
#define CHASSIS_STEER_RPM_TO_SPEED_DEFAULT 1.0F
#endif

/** @brief 默认舵向编码器 tick 到 rad 的换算系数。 */
#ifndef CHASSIS_STEER_ECD_TO_RAD_DEFAULT
#define CHASSIS_STEER_ECD_TO_RAD_DEFAULT 0.0007669904F
#endif

/** @brief 默认底盘旋转速度缩放。 */
#ifndef CHASSIS_WZ_SCALE_DEFAULT
#define CHASSIS_WZ_SCALE_DEFAULT 0.1F
#endif

/** @brief 默认平移速度变化率上限，单位 m/s^2。 */
#ifndef CHASSIS_TRANSLATION_SLEW_DEFAULT
#define CHASSIS_TRANSLATION_SLEW_DEFAULT 2.5F
#endif

/** @brief 默认旋转角速度变化率上限，单位 rad/s^2。 */
#ifndef CHASSIS_ROTATION_SLEW_DEFAULT
#define CHASSIS_ROTATION_SLEW_DEFAULT 4.0F
#endif

/** @brief 默认轮到中心距离。 */
#ifndef CHASSIS_CENTER_DIST_DEFAULT
#define CHASSIS_CENTER_DIST_DEFAULT 0.22F
#endif

/** @brief 默认舵轮半车长。 */
#ifndef CHASSIS_HALF_LENGTH_DEFAULT
#define CHASSIS_HALF_LENGTH_DEFAULT 0.20F
#endif

/** @brief 默认舵轮半车宽。 */
#ifndef CHASSIS_HALF_WIDTH_DEFAULT
#define CHASSIS_HALF_WIDTH_DEFAULT 0.175F
#endif

/** @brief 默认平衡底盘腿长。 */
#ifndef CHASSIS_LEG_DEFAULT
#define CHASSIS_LEG_DEFAULT 0.30F
#endif

/** @brief 默认平衡底盘最小腿长。 */
#ifndef CHASSIS_LEG_MIN_DEFAULT
#define CHASSIS_LEG_MIN_DEFAULT 0.15F
#endif

/** @brief 默认平衡底盘最大腿长。 */
#ifndef CHASSIS_LEG_MAX_DEFAULT
#define CHASSIS_LEG_MAX_DEFAULT 0.40F
#endif

/**
 * @brief 底盘执行状态。
 */
typedef enum
{
    ChassisOk = 0,    /**< 执行成功。 */
    ChassisNull = 1,  /**< 输入指针为空。 */
    ChassisArg = 2,   /**< 输入参数非法。 */
    ChassisState = 3, /**< 当前上下文状态或反馈状态不允许执行。 */
    ChassisValue = 4  /**< 输入数值非法。 */
} ChassisStatus;

/**
 * @brief 底盘构型。
 */
typedef enum
{
    ChassisTypeMecanum = CHASSIS_TYPE_MECANUM,   /**< 麦克纳姆轮底盘。 */
    ChassisTypeOmni = CHASSIS_TYPE_OMNI,         /**< 全向轮底盘。 */
    ChassisTypeSteering = CHASSIS_TYPE_STEERING, /**< 舵轮底盘。 */
    ChassisTypeBalance = CHASSIS_TYPE_BALANCE    /**< 平衡底盘。 */
} ChassisType;

/**
 * @brief 底盘驱动输出语义。
 */
typedef enum
{
    ChassisDriveModeCurrent = 0, /**< 输出电流/力矩，速度环在 STM32。 */
    ChassisDriveModeSpeed = 1    /**< 输出速度目标，速度环在电机内部。 */
} ChassisDriveMode;

/**
 * @brief 底盘模式。
 */
typedef enum
{
    ChassisModeStop = 0,             /**< 停止模式。 */
    ChassisModeRaw = 1,              /**< 原始电流或原始力矩模式。 */
    ChassisModeNoFollow = 2,         /**< 不跟随 yaw，直接使用速度命令。 */
    ChassisModeFollowGimbalYaw = 3,  /**< 跟随云台 yaw 参考。 */
    ChassisModeFollowChassisYaw = 4, /**< 跟随底盘 yaw 参考。 */
    ChassisModeSpin = 5,             /**< 小陀螺模式。 */
    ChassisModeBalance = 6           /**< 平衡底盘控制模式。 */
} ChassisMode;

/**
 * @brief 底盘 PID 参数。
 */
typedef struct
{
    float32_t kp;       /**< 比例增益。 */
    float32_t ki;       /**< 积分增益。 */
    float32_t kd;       /**< 微分增益。 */
    float32_t max_out;  /**< 输出绝对值限幅。 */
    float32_t max_iout; /**< 积分输出绝对值限幅。 */
} ChassisPidCfg;

/**
 * @brief 普通底盘电机反馈。
 */
typedef struct
{
    uint16_t ecd;       /**< 编码器原始值。 */
    int16_t speed_rpm;  /**< 转速反馈，单位 rpm。 */
    int16_t current;    /**< 电流反馈。 */
    float32_t angle;    /**< 注入角度，单位 rad，舵向电机优先使用。 */
    float32_t speed;    /**< 注入速度，非 0 时优先于 speed_rpm 换算值。 */
    uint8_t online;     /**< 非 0 表示反馈在线。 */
} ChassisMotorFb;

/**
 * @brief MIT 类电机反馈。
 */
typedef struct
{
    float32_t position; /**< 位置反馈，单位 rad。 */
    float32_t velocity; /**< 速度反馈，单位 rad/s。 */
    float32_t torque;   /**< 力矩反馈，单位 N*m。 */
    uint8_t online;     /**< 非 0 表示反馈在线。 */
} ChassisMitFb;

/**
 * @brief 底盘上层命令。
 */
typedef struct
{
    uint8_t online;                                          /**< 非 0 表示命令源在线。 */
    uint8_t force_stop;                                      /**< 非 0 表示强制停止底盘。 */
    uint8_t jump;                                            /**< 非 0 表示平衡底盘跳跃请求。 */
    ChassisMode mode;                                        /**< 目标底盘模式。 */
    float32_t vx;                                            /**< 纵向速度命令。 */
    float32_t vy;                                            /**< 横向速度命令。 */
    float32_t wz;                                            /**< 角速度命令。 */
    float32_t yaw_ref;                                       /**< yaw 参考角，单位 rad。 */
    float32_t frame_angle;                                   /**< 速度坐标旋转角，单位 rad。 */
    float32_t leg_length;                                    /**< 平衡底盘统一目标腿长。 */
    float32_t leg_left;                                      /**< 平衡底盘左腿目标腿长。 */
    float32_t leg_right;                                     /**< 平衡底盘右腿目标腿长。 */
    int16_t raw_drive_current[CHASSIS_DRIVE_MAX];            /**< 原始驱动电流命令。 */
    int16_t raw_steer_current[CHASSIS_STEER_MAX];            /**< 原始舵向电流命令。 */
    float32_t raw_joint_torque[CHASSIS_JOINT_MAX];           /**< 原始关节力矩命令。 */
    float32_t raw_wheel_torque[CHASSIS_WHEEL_MAX];           /**< 原始轮毂力矩命令。 */
} ChassisCmd;

/**
 * @brief 底盘周期输入。
 */
typedef struct
{
    ChassisCmd cmd;                                    /**< 上层控制命令。 */
    ChassisMotorFb drive_fb[CHASSIS_DRIVE_MAX];        /**< 驱动轮电机反馈。 */
    ChassisMotorFb steer_fb[CHASSIS_STEER_MAX];        /**< 舵向电机反馈。 */
    ChassisMitFb joint_fb[CHASSIS_JOINT_MAX];          /**< 平衡底盘关节电机反馈。 */
    ChassisMitFb wheel_fb[CHASSIS_WHEEL_MAX];          /**< 平衡底盘轮毂电机反馈。 */
    float32_t yaw;                                     /**< 底盘 yaw 角，单位 rad。 */
    float32_t pitch;                                   /**< 底盘 pitch 角，单位 rad。 */
    float32_t roll;                                    /**< 底盘 roll 角，单位 rad。 */
    float32_t gyro[3];                                 /**< 机体系角速度，单位 rad/s。 */
    float32_t acc[3];                                  /**< 机体系加速度。 */
    float32_t leg_length[CHASSIS_WHEEL_MAX];           /**< 左右腿长反馈。 */
    float32_t leg_speed[CHASSIS_WHEEL_MAX];            /**< 左右腿长速度反馈。 */
    float32_t leg_theta[CHASSIS_WHEEL_MAX];            /**< 左右腿摆角反馈。 */
    float32_t leg_theta_speed[CHASSIS_WHEEL_MAX];      /**< 左右腿摆角速度反馈。 */
    float32_t power;                                   /**< 当前底盘功率。 */
    float32_t power_limit;                             /**< 当前底盘功率上限。 */
    float32_t buffer_energy;                           /**< 当前底盘缓冲能量。 */
    uint32_t tick_ms;                                  /**< 当前时间戳，单位 ms。 */
} ChassisInput;

/**
 * @brief 底盘周期输出。
 */
typedef struct
{
    int16_t drive_current[CHASSIS_DRIVE_MAX];       /**< 驱动轮电机输出电流。 */
    int16_t steer_current[CHASSIS_STEER_MAX];       /**< 舵向电机输出电流。 */
    float32_t joint_torque[CHASSIS_JOINT_MAX];      /**< 平衡底盘关节输出力矩。 */
    float32_t wheel_torque[CHASSIS_WHEEL_MAX];      /**< 平衡底盘轮毂输出力矩。 */
    float32_t drive_speed_set[CHASSIS_DRIVE_MAX];   /**< 驱动轮速度目标。 */
    float32_t steer_angle_set[CHASSIS_STEER_MAX];   /**< 舵向角度目标。 */
    float32_t joint_pos_set[CHASSIS_JOINT_MAX];     /**< 平衡底盘关节位置目标。 */
    ChassisMode mode;                               /**< 本周期实际模式。 */
    uint8_t send_enable;                            /**< 非 0 表示输出允许发送。 */
} ChassisOutput;

/**
 * @brief 底盘配置。
 */
typedef struct
{
    ChassisType type;                                      /**< 当前构型类型。 */
    ChassisDriveMode drive_mode;                           /**< 驱动输出语义。 */
    uint8_t min_online_motors;                             /**< 允许运行的最少在线驱动轮数。 */
    ChassisPidCfg drive_pid;                               /**< 驱动轮速度 PID。 */
    ChassisPidCfg steer_angle_pid;                         /**< 舵向角度 PID。 */
    ChassisPidCfg steer_speed_pid;                         /**< 舵向速度 PID。 */
    ChassisPidCfg yaw_pid;                                 /**< yaw 跟随 PID。 */
    ChassisPidCfg leg_pid;                                 /**< 平衡底盘腿长 PID。 */
    ChassisPidCfg roll_pid;                                /**< 平衡底盘 roll 支撑 PID。 */
    ChassisPidCfg turn_pid;                                /**< 平衡底盘转向 PID。 */
    float32_t drive_rpm_to_speed;                          /**< 驱动电机 rpm 到速度的换算系数。 */
    float32_t steer_rpm_to_speed;                          /**< 舵向电机 rpm 到速度的换算系数。 */
    float32_t steer_ecd_to_rad;                            /**< 舵向编码器 tick 到 rad 的换算系数。 */
    float32_t max_drive_speed;                             /**< 最大驱动轮速度。 */
    float32_t max_drive_current;                           /**< 最大驱动电流。 */
    float32_t max_steer_current;                           /**< 最大舵向电流。 */
    float32_t max_joint_torque;                            /**< 最大关节力矩。 */
    float32_t max_wheel_torque;                            /**< 最大轮毂力矩。 */
    float32_t power_limit;                                 /**< 默认功率上限。 */
    PowerCtrlCfg power_cfg;                                /**< 底盘驱动电机功率预测配置。 */
    float32_t mecanum_center;                              /**< 麦轮到中心距离。 */
    float32_t omni_center;                                 /**< 全向轮到中心距离。 */
    float32_t wz_scale;                                    /**< 旋转速度缩放。 */
    float32_t translation_slew;                            /**< 平移速度变化率上限，单位 m/s^2。 */
    float32_t rotation_slew;                               /**< 旋转角速度变化率上限，单位 rad/s^2。 */
    float32_t half_length;                                 /**< 舵轮半车长。 */
    float32_t half_width;                                  /**< 舵轮半车宽。 */
    float32_t steer_zero[CHASSIS_STEER_MAX];               /**< 舵向零位偏置。 */
    float32_t leg_default;                                 /**< 默认腿长。 */
    float32_t leg_min;                                     /**< 最小腿长。 */
    float32_t leg_max;                                     /**< 最大腿长。 */
    float32_t balance_lqr[12];                             /**< 平衡底盘 LQR 系数。 */
    int8_t drive_polarity[CHASSIS_DRIVE_MAX];              /**< 驱动电流极性。 */
    int8_t steer_polarity[CHASSIS_STEER_MAX];              /**< 舵向电流极性。 */
    int8_t joint_polarity[CHASSIS_JOINT_MAX];              /**< 关节力矩极性。 */
    int8_t wheel_polarity[CHASSIS_WHEEL_MAX];              /**< 轮毂力矩极性。 */
} ChassisCfg;

/**
 * @brief 底盘公共上下文。
 */
typedef struct
{
    ChassisCfg cfg;     /**< 公共配置副本。 */
    void *impl;         /**< 当前编译期构型的私有上下文。 */
    uint8_t initialized;/**< 非 0 表示公共上下文已初始化。 */
} Chassis;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 初始化当前编译期构型的底盘上下文。
 *
 * @param chassis 底盘公共上下文。
 * @param cfg 公共配置。
 * @param impl 当前构型私有上下文。
 * @return 执行状态。
 */
ChassisStatus chassis_init(Chassis *chassis, const ChassisCfg *cfg, void *impl);

/**
 * @brief 执行一次完整底盘周期。
 *
 * @param chassis 底盘公共上下文。
 * @param input 周期输入。
 * @param output 周期输出。
 * @return 执行状态。
 */
ChassisStatus chassis_step(Chassis *chassis, const ChassisInput *input, ChassisOutput *output);

/**
 * @brief 更新底盘反馈。
 *
 * @param chassis 底盘公共上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
ChassisStatus chassis_update(Chassis *chassis, const ChassisInput *input);

/**
 * @brief 执行底盘控制计算。
 *
 * @param chassis 底盘公共上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
ChassisStatus chassis_control(Chassis *chassis, const ChassisInput *input);

/**
 * @brief 生成底盘输出。
 *
 * @param chassis 底盘公共上下文。
 * @param output 周期输出。
 * @return 执行状态。
 */
ChassisStatus chassis_output(const Chassis *chassis, ChassisOutput *output);

/**
 * @brief 将底盘控制目标同步到当前反馈状态。
 *
 * @param chassis 底盘公共上下文。
 * @return 执行状态。
 */
ChassisStatus chassis_sync(Chassis *chassis);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_MECHANISM_CHASSIS_H

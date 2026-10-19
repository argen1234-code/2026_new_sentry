/**
 ******************************************************************************
 * @file    observer.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   观测器组件公共接口。
 *
 * @note    本组件只做姿态、底盘速度和平衡腿状态估计，不读取硬件、任务、CAN 或 RTOS。
 * @note    调用方负责把设备采样和电机反馈注入 Observer，再把 ObserverState 填入 Controller 输入。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_OBSERVER_H
#define PRINTK_COMPONENT_OBSERVER_H

#include <stdint.h>

#include "arm_math.h"
#include "common.h"
#include "kalman.h"
#include "mahony.h"
#include "robot.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 非 0 表示启用 INS 姿态观测。 */
#ifndef OBSERVER_USE_INS
#ifdef ROBOT_OBSERVER_USE_INS
/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */
#define OBSERVER_USE_INS ROBOT_OBSERVER_USE_INS
#else
#define OBSERVER_USE_INS 1U
#endif
#endif

/** @brief 非 0 表示启用底盘速度 Kalman 观测。 */
#ifndef OBSERVER_USE_CHASSIS
#ifdef ROBOT_OBSERVER_USE_CHASSIS
#define OBSERVER_USE_CHASSIS ROBOT_OBSERVER_USE_CHASSIS
#else
#define OBSERVER_USE_CHASSIS 1U
#endif
#endif

/** @brief 非 0 表示启用平衡腿状态观测。 */
#ifndef OBSERVER_USE_BALANCE
#ifdef ROBOT_OBSERVER_USE_BALANCE
#define OBSERVER_USE_BALANCE ROBOT_OBSERVER_USE_BALANCE
#else
#define OBSERVER_USE_BALANCE 1U
#endif
#endif

/** @brief 默认重力加速度，单位 m/s^2。 */
#ifndef OBSERVER_GRAVITY_DEFAULT
#define OBSERVER_GRAVITY_DEFAULT 9.80665F
#endif

/** @brief 默认 ready 时间，单位 s。 */
#ifndef OBSERVER_READY_TIME_DEFAULT
#define OBSERVER_READY_TIME_DEFAULT 0.5F
#endif

/** @brief 默认静止陀螺仪阈值，单位 rad/s。 */
#ifndef OBSERVER_STABLE_GYRO_DEFAULT
#define OBSERVER_STABLE_GYRO_DEFAULT 0.03F
#endif

/** @brief 默认静止加速度模长误差阈值，单位 m/s^2。 */
#ifndef OBSERVER_STABLE_ACCEL_DEFAULT
#define OBSERVER_STABLE_ACCEL_DEFAULT 0.5F
#endif

/** @brief 默认运动加速度低通时间常数，单位 s。 */
#ifndef OBSERVER_ACCEL_LPF_DEFAULT
#define OBSERVER_ACCEL_LPF_DEFAULT 0.02F
#endif

/** @brief 默认导航系 X/Y 运动加速度死区，单位 m/s^2。 */
#ifndef OBSERVER_ACCEL_DEAD_XY_DEFAULT
#define OBSERVER_ACCEL_DEAD_XY_DEFAULT 0.02F
#endif

/** @brief 默认导航系 Z 运动加速度死区，单位 m/s^2。 */
#ifndef OBSERVER_ACCEL_DEAD_Z_DEFAULT
#define OBSERVER_ACCEL_DEAD_Z_DEFAULT 0.04F
#endif

/** @brief 默认 Mahony 启动零偏采样次数。 */
#ifndef OBSERVER_GYRO_CALI_SAMPLES_DEFAULT
#define OBSERVER_GYRO_CALI_SAMPLES_DEFAULT 1000U
#endif

/** @brief 默认零偏采样陀螺仪模长阈值，单位 rad/s。 */
#ifndef OBSERVER_CALI_GYRO_TH_DEFAULT
#define OBSERVER_CALI_GYRO_TH_DEFAULT 0.03F
#endif

/** @brief 默认零偏采样加速度模长误差阈值，单位 m/s^2。 */
#ifndef OBSERVER_CALI_ACCEL_TH_DEFAULT
#define OBSERVER_CALI_ACCEL_TH_DEFAULT 0.5F
#endif

/** @brief 默认 Kalman 初始协方差。 */
#ifndef OBSERVER_KF_P_DEFAULT
#define OBSERVER_KF_P_DEFAULT 1.0F
#endif

/** @brief 默认 Kalman 过程噪声。 */
#ifndef OBSERVER_KF_Q_DEFAULT
#define OBSERVER_KF_Q_DEFAULT 0.5F
#endif

/** @brief 默认 Kalman 测量噪声。 */
#ifndef OBSERVER_KF_R_DEFAULT
#define OBSERVER_KF_R_DEFAULT 100.0F
#endif

/** @brief 默认速度 Kalman 状态转移 dt，单位 s。 */
#ifndef OBSERVER_KF_DT_DEFAULT
#define OBSERVER_KF_DT_DEFAULT 0.003F
#endif

/** @brief 默认五连杆 AE 长度，单位 m。 */
#ifndef OBSERVER_LEG_L5_DEFAULT
#define OBSERVER_LEG_L5_DEFAULT 0.0F
#endif

/** @brief 默认五连杆 AB 长度，单位 m。 */
#ifndef OBSERVER_LEG_L1_DEFAULT
#define OBSERVER_LEG_L1_DEFAULT 0.215F
#endif

/** @brief 默认五连杆 BC 长度，单位 m。 */
#ifndef OBSERVER_LEG_L2_DEFAULT
#define OBSERVER_LEG_L2_DEFAULT 0.258F
#endif

/** @brief 默认五连杆 CD 长度，单位 m。 */
#ifndef OBSERVER_LEG_L3_DEFAULT
#define OBSERVER_LEG_L3_DEFAULT 0.258F
#endif

/** @brief 默认五连杆 DE 长度，单位 m。 */
#ifndef OBSERVER_LEG_L4_DEFAULT
#define OBSERVER_LEG_L4_DEFAULT 0.215F
#endif

/** @brief 默认离地判断支撑力阈值。 */
#ifndef OBSERVER_GROUND_TH_DEFAULT
#define OBSERVER_GROUND_TH_DEFAULT 8.0F
#endif

/** @brief 默认支撑力估计中的轮端重力项。 */
#ifndef OBSERVER_WHEEL_WEIGHT_DEFAULT
#define OBSERVER_WHEEL_WEIGHT_DEFAULT 8.0F
#endif

/** @brief 观测轴 X 位。 */
#define OBSERVER_AXIS_X (1U << 0)

/** @brief 观测轴 Y 位。 */
#define OBSERVER_AXIS_Y (1U << 1)

/** @brief 观测轴 Z 位。 */
#define OBSERVER_AXIS_Z (1U << 2)

/** @brief INS 状态有效位。 */
#define OBSERVER_VALID_INS (1U << 0)

/** @brief 底盘速度状态有效位。 */
#define OBSERVER_VALID_CHASSIS (1U << 1)

/** @brief 平衡腿状态有效位。 */
#define OBSERVER_VALID_BALANCE (1U << 2)

/** @brief 底盘观测轴数量。 */
#define OBSERVER_CHASSIS_AXIS_NUM 3U

/** @brief 最大普通电机反馈数量。 */
#define OBSERVER_MOTOR_MAX 4U

/** @brief 平衡底盘腿数量。 */
#define OBSERVER_LEG_NUM 2U

/** @brief 平衡底盘关节反馈数量。 */
#define OBSERVER_JOINT_MAX 4U

/** @brief 平衡底盘轮毂反馈数量。 */
#define OBSERVER_WHEEL_MAX 2U

/** @brief 左腿索引。 */
#define OBSERVER_LEG_LEFT 0U

/** @brief 右腿索引。 */
#define OBSERVER_LEG_RIGHT 1U

/** @brief 每个速度 Kalman 工作区元素数量。 */
#define OBSERVER_KF_WORK_LEN 28U

/**
 * @brief 观测器执行状态。
 */
typedef enum
{
    ObserverOk = 0,       /**< 执行成功。 */
    ObserverNull = 1,     /**< 输入指针为空。 */
    ObserverArg = 2,      /**< 配置参数非法。 */
    ObserverValue = 3,    /**< 输入或内部数值包含 NaN/Inf。 */
    ObserverUninit = 4,   /**< 上下文未初始化或状态不允许执行。 */
    ObserverFusion = 5,   /**< 姿态解算失败。 */
    ObserverKalman = 6,   /**< Kalman 估计失败。 */
    ObserverGeometry = 7  /**< 平衡腿几何解算失败。 */
} ObserverStatus;

/**
 * @brief IMU 原始观测输入。
 */
typedef struct
{
    Vec3 gyro;              /**< 三轴角速度，单位 rad/s。 */
    Vec3 accel;             /**< 三轴加速度，单位 m/s^2。 */
    float32_t temperature;  /**< IMU 温度，单位摄氏度，仅透传。 */
    float32_t dt;           /**< 本周期时间间隔，单位 s。 */
    uint32_t timestamp_us;  /**< 输入时间戳，单位 us。 */
    uint8_t online;         /**< 非 0 表示 IMU 数据有效。 */
} ObserverImuInput;

/**
 * @brief 普通电机反馈。
 */
typedef struct
{
    uint16_t ecd;       /**< 编码器原始值。 */
    int16_t speed_rpm;  /**< 转速反馈，单位 rpm。 */
    int16_t current;    /**< 电流反馈。 */
    uint8_t online;     /**< 非 0 表示反馈在线。 */
} ObserverMotorFb;

/**
 * @brief MIT 执行器反馈。
 */
typedef struct
{
    float32_t pos;  /**< 位置反馈，单位 rad。 */
    float32_t vel;  /**< 速度反馈，单位 rad/s。 */
    float32_t tor;  /**< 力矩反馈，单位 N*m。 */
    uint8_t online; /**< 非 0 表示反馈在线。 */
} ObserverMitFb;

/**
 * @brief 观测器周期输入。
 */
typedef struct
{
    ObserverImuInput imu;                        /**< IMU 原始输入。 */
    ObserverMotorFb motor_fb[OBSERVER_MOTOR_MAX];/**< 普通电机反馈，供上层扩展速度观测。 */
    ObserverMitFb joint_fb[OBSERVER_JOINT_MAX];  /**< 平衡底盘关节反馈。 */
    ObserverMitFb wheel_fb[OBSERVER_WHEEL_MAX];  /**< 平衡底盘轮毂反馈。 */
    Vec3 chassis_vel;                            /**< 底盘速度测量。 */
    Vec3 chassis_acc;                            /**< 底盘加速度测量。 */
    float32_t leg_force[OBSERVER_LEG_NUM];       /**< 当前腿端轴向力估计或上一周期命令。 */
    float32_t leg_torque[OBSERVER_LEG_NUM];      /**< 当前腿端摆动方向力矩估计或上一周期命令。 */
    uint32_t timestamp_us;                       /**< 统一输入时间戳，单位 us。 */
    uint8_t vel_valid_mask;                      /**< 底盘速度测量有效轴掩码。 */
    uint8_t acc_valid_mask;                      /**< 底盘加速度测量有效轴掩码。 */
    uint8_t leg_force_valid_mask;                /**< 腿端力和力矩有效腿掩码。 */
} ObserverInput;

/**
 * @brief INS 姿态观测状态。
 */
typedef struct
{
    Quat quat;               /**< 姿态四元数。 */
    Euler euler;             /**< roll/pitch/yaw，单位 rad。 */
    Vec3 gyro_bias;          /**< 陀螺仪零偏估计，单位 rad/s。 */
    Vec3 gyro;               /**< 融合后的角速度，单位 rad/s。 */
    Vec3 gyro_raw;           /**< 原始角速度，单位 rad/s。 */
    Vec3 accel;              /**< 原始加速度，单位 m/s^2。 */
    Vec3 gravity_body;       /**< 重力在机体系下的向量，单位 m/s^2。 */
    Vec3 motion_accel_body;  /**< 机体系运动加速度，单位 m/s^2。 */
    Vec3 motion_accel_world; /**< 导航系运动加速度，单位 m/s^2。 */
    float32_t yaw_total;     /**< yaw 连续角，单位 rad。 */
    float32_t accel_norm;    /**< 加速度模长，单位 m/s^2。 */
    float32_t gyro_norm;     /**< 角速度模长，单位 rad/s。 */
    float32_t temperature;   /**< IMU 温度，单位摄氏度。 */
    uint32_t timestamp_us;   /**< 状态时间戳，单位 us。 */
    uint8_t ready;           /**< 非 0 表示姿态已 ready。 */
    uint8_t stable;          /**< 非 0 表示当前近似静止。 */
    uint8_t valid;           /**< 非 0 表示本状态有效。 */
} ObserverInsState;

/**
 * @brief 底盘速度观测状态。
 */
typedef struct
{
    Vec3 vel;              /**< 估计速度。 */
    Vec3 acc;              /**< 估计加速度。 */
    uint32_t timestamp_us; /**< 状态时间戳，单位 us。 */
    uint8_t valid_mask;    /**< 已有效估计的轴掩码。 */
    uint8_t valid;         /**< 非 0 表示至少一个轴有效。 */
} ObserverChassisState;

/**
 * @brief 单条平衡腿观测状态。
 */
typedef struct
{
    float32_t length;        /**< 腿长 L0，单位 m。 */
    float32_t length_speed;  /**< 腿长一阶导数，单位 m/s。 */
    float32_t length_accel;  /**< 腿长二阶导数，单位 m/s^2。 */
    float32_t phi0;          /**< 五连杆 C 点极角，单位 rad。 */
    float32_t alpha;         /**< 腿部 alpha 角，单位 rad。 */
    float32_t alpha_speed;   /**< alpha 一阶导数，单位 rad/s。 */
    float32_t theta;         /**< 腿摆角，单位 rad。 */
    float32_t theta_speed;   /**< 腿摆角速度，单位 rad/s。 */
    float32_t theta_accel;   /**< 腿摆角加速度，单位 rad/s^2。 */
    float32_t support_force; /**< 支撑力估计。 */
    uint8_t grounded;        /**< 非 0 表示判定接地。 */
    uint8_t valid;           /**< 非 0 表示本腿几何状态有效。 */
} ObserverLegState;

/**
 * @brief 平衡底盘观测状态。
 */
typedef struct
{
    ObserverLegState leg[OBSERVER_LEG_NUM]; /**< 左右腿观测状态。 */
    uint32_t timestamp_us;                  /**< 状态时间戳，单位 us。 */
    uint8_t valid_mask;                     /**< 有效腿掩码。 */
    uint8_t grounded_mask;                  /**< 接地腿掩码。 */
    uint8_t valid;                          /**< 非 0 表示至少一条腿有效。 */
} ObserverBalanceState;

/**
 * @brief 观测器整体输出状态。
 */
typedef struct
{
    ObserverInsState ins;             /**< INS 姿态状态。 */
    ObserverChassisState chassis;     /**< 底盘速度状态。 */
    ObserverBalanceState balance;     /**< 平衡腿状态。 */
    uint32_t timestamp_us;            /**< 最近一次整体更新时间戳，单位 us。 */
    uint8_t valid_mask;               /**< 已有效子状态掩码。 */
} ObserverState;

/**
 * @brief 单条平衡腿几何配置。
 */
typedef struct
{
    float32_t l1;            /**< 五连杆 AB 长度，单位 m。 */
    float32_t l2;            /**< 五连杆 BC 长度，单位 m。 */
    float32_t l3;            /**< 五连杆 CD 长度，单位 m。 */
    float32_t l4;            /**< 五连杆 DE 长度，单位 m。 */
    float32_t l5;            /**< 五连杆 AE 长度，单位 m。 */
    float32_t phi1_offset;   /**< 前关节角零位偏置，单位 rad。 */
    float32_t phi4_offset;   /**< 后关节角零位偏置，单位 rad。 */
    float32_t pitch_sign;    /**< 机体 pitch 对该腿摆角的符号。 */
    float32_t gyro_sign;     /**< 机体 pitch 角速度对该腿的符号。 */
    int8_t phi1_sign;        /**< 前关节角符号，0 按 1 处理。 */
    int8_t phi4_sign;        /**< 后关节角符号，0 按 1 处理。 */
} ObserverLegCfg;

/**
 * @brief 观测器配置。
 */
typedef struct
{
    MahonyCfg mahony;                              /**< Mahony 姿态解算配置。 */
    ObserverLegCfg leg[OBSERVER_LEG_NUM];          /**< 左右腿几何配置。 */
    float32_t gravity;                             /**< 重力加速度，单位 m/s^2。 */
    float32_t ready_time;                          /**< 姿态 ready 时间，单位 s。 */
    float32_t stable_gyro;                         /**< 静止陀螺仪阈值，单位 rad/s。 */
    float32_t stable_accel;                        /**< 静止加速度模长误差阈值，单位 m/s^2。 */
    float32_t accel_lpf;                           /**< 运动加速度低通时间常数，单位 s。 */
    float32_t accel_dead_xy;                       /**< 导航系 X/Y 运动加速度死区，单位 m/s^2。 */
    float32_t accel_dead_z;                        /**< 导航系 Z 运动加速度死区，单位 m/s^2。 */
    uint32_t gyro_cali_samples;                    /**< 启动阶段陀螺仪零偏采样次数。 */
    float32_t cali_gyro_th;                        /**< 零偏采样陀螺仪模长阈值，单位 rad/s。 */
    float32_t cali_accel_th;                       /**< 零偏采样加速度模长误差阈值，单位 m/s^2。 */
    float32_t kf_p;                                /**< 速度 Kalman 初始协方差。 */
    float32_t kf_q;                                /**< 速度 Kalman 过程噪声。 */
    float32_t kf_r;                                /**< 速度 Kalman 测量噪声。 */
    float32_t kf_dt;                               /**< 速度 Kalman 状态转移 dt，单位 s。 */
    float32_t ground_threshold;                    /**< 离地判断支撑力阈值。 */
    float32_t wheel_weight;                        /**< 支撑力估计轮端重力项。 */
} ObserverCfg;

/**
 * @brief 单轴速度 Kalman 私有上下文。
 */
typedef struct
{
    kf_t kf;                                  /**< Kalman 上下文。 */
    kf_work_t work;                           /**< Kalman 工作区视图。 */
    float32_t x[2];                           /**< 状态向量。 */
    float32_t p[4];                           /**< 协方差矩阵。 */
    float32_t f[4];                           /**< 状态转移矩阵。 */
    float32_t q[4];                           /**< 过程噪声矩阵。 */
    float32_t h[4];                           /**< 测量矩阵。 */
    float32_t r[4];                           /**< 测量噪声矩阵。 */
    float32_t k[4];                           /**< Kalman 增益矩阵。 */
    float32_t y[2];                           /**< 创新向量。 */
    float32_t s[4];                           /**< 创新协方差矩阵。 */
    float32_t work_buf[OBSERVER_KF_WORK_LEN]; /**< 临时矩阵缓冲区。 */
    uint8_t initialized;                      /**< 非 0 表示该轴 Kalman 已初始化。 */
} ObserverAxisKf;

/**
 * @brief 单条平衡腿私有上下文。
 */
typedef struct
{
    float32_t last_phi0;        /**< 上一次 phi0。 */
    float32_t last_length;      /**< 上一次腿长。 */
    float32_t last_length_speed;/**< 上一次腿长速度。 */
    float32_t last_theta_speed; /**< 上一次腿摆角速度。 */
    uint8_t history_valid;      /**< 非 0 表示历史差分状态有效。 */
} ObserverLegCtx;

/**
 * @brief 观测器上下文。
 */
typedef struct
{
    ObserverCfg cfg;                               /**< 配置副本。 */
    ObserverState state;                           /**< 当前输出状态。 */
    Mahony mahony;                                 /**< Mahony 姿态上下文。 */
    ObserverAxisKf chassis_kf[OBSERVER_CHASSIS_AXIS_NUM]; /**< 底盘速度 Kalman 上下文。 */
    ObserverLegCtx leg_ctx[OBSERVER_LEG_NUM];      /**< 平衡腿历史上下文。 */
    Vec3 motion_accel_body_lpf;                    /**< 机体系运动加速度低通状态。 */
    float32_t run_time;                            /**< 累计运行时间，单位 s。 */
    float32_t yaw_last;                            /**< 上一次 yaw，单位 rad。 */
    int32_t yaw_round_count;                       /**< yaw 圈数计数。 */
    uint8_t yaw_last_valid;                        /**< 非 0 表示 yaw 上一次值有效。 */
    uint8_t initialized;                           /**< 非 0 表示已初始化。 */
} Observer;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 初始化观测器组件。
 *
 * @param observer 观测器上下文。
 * @param cfg 观测器配置。
 * @return 执行状态。
 */
ObserverStatus observer_init(Observer *observer, const ObserverCfg *cfg);

/**
 * @brief 复位观测器动态状态。
 *
 * @param observer 观测器上下文。
 * @return 执行状态。
 */
ObserverStatus observer_reset(Observer *observer);

/**
 * @brief 执行一次完整观测周期。
 *
 * @param observer 观测器上下文。
 * @param input 本周期输入。
 * @param state 输出状态；可为空。
 * @return 执行状态。
 */
ObserverStatus observer_step(Observer *observer, const ObserverInput *input, ObserverState *state);

/**
 * @brief 更新观测器状态。
 *
 * @param observer 观测器上下文。
 * @param input 本周期输入。
 * @return 执行状态。
 */
ObserverStatus observer_update(Observer *observer, const ObserverInput *input);

/**
 * @brief 获取最近一次观测状态。
 *
 * @param observer 观测器上下文。
 * @param state 输出状态。
 * @return 执行状态。
 */
ObserverStatus observer_get(const Observer *observer, ObserverState *state);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_OBSERVER_H

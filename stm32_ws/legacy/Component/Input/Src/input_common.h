/**
 ******************************************************************************
 * @file    input_common.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   统一输入参考公共语言。
 *
 * @note    本文件定义 RC、视觉、SLAM、planner 等输入源进入机构层前的统一参考。
 * @note    Input 层只做输入解释、优先级仲裁和命令生成，不直接执行机构闭环控制。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_INPUT_COMMON_H
#define PRINTK_COMPONENT_INPUT_COMMON_H

#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */
/** @brief 云台 yaw 参考有效位。 */
#define INPUT_GIMBAL_YAW_VALID ((uint32_t)0x00000001UL)

/** @brief 云台 pitch 参考有效位。 */
#define INPUT_GIMBAL_PITCH_VALID ((uint32_t)0x00000002UL)

/** @brief 云台模式参考有效位。 */
#define INPUT_GIMBAL_MODE_VALID ((uint32_t)0x00000004UL)

/** @brief 底盘 vx 参考有效位。 */
#define INPUT_CHASSIS_VX_VALID ((uint32_t)0x00000001UL)

/** @brief 底盘 vy 参考有效位。 */
#define INPUT_CHASSIS_VY_VALID ((uint32_t)0x00000002UL)

/** @brief 底盘 wz 参考有效位。 */
#define INPUT_CHASSIS_WZ_VALID ((uint32_t)0x00000004UL)

/** @brief 底盘 yaw 参考有效位。 */
#define INPUT_CHASSIS_YAW_VALID ((uint32_t)0x00000008UL)

/** @brief 底盘坐标系角度参考有效位。 */
#define INPUT_CHASSIS_FRAME_VALID ((uint32_t)0x00000010UL)

/** @brief 底盘腿长统一参考有效位。 */
#define INPUT_CHASSIS_LEG_VALID ((uint32_t)0x00000020UL)

/** @brief 底盘左腿参考有效位。 */
#define INPUT_CHASSIS_LEFT_VALID ((uint32_t)0x00000040UL)

/** @brief 底盘右腿参考有效位。 */
#define INPUT_CHASSIS_RIGHT_VALID ((uint32_t)0x00000080UL)

/** @brief 底盘模式参考有效位。 */
#define INPUT_CHASSIS_MODE_VALID ((uint32_t)0x00000100UL)

/** @brief 发射摩擦轮参考有效位。 */
#define INPUT_SHOOT_FRIC_VALID ((uint32_t)0x00000001UL)

/** @brief 发射单发参考有效位。 */
#define INPUT_SHOOT_ONCE_VALID ((uint32_t)0x00000002UL)

/** @brief 发射连发参考有效位。 */
#define INPUT_SHOOT_HOLD_VALID ((uint32_t)0x00000004UL)

/** @brief 发射停止参考有效位。 */
#define INPUT_SHOOT_STOP_VALID ((uint32_t)0x00000008UL)

/** @brief 弹舱动作参考有效位。 */
#define INPUT_MAG_ACTION_VALID ((uint32_t)0x00000001UL)

/** @brief 校准动作参考有效位。 */
#define INPUT_CALI_ACTION_VALID ((uint32_t)0x00000001UL)

/**
 * @brief Input 执行状态。
 */
typedef enum
{
    InputOk = 0,    /**< 执行成功。 */
    InputNull = 1,  /**< 输入指针为空。 */
    InputArg = 2,   /**< 输入参数非法。 */
    InputValue = 3  /**< 输入值非法。 */
} InputStatus;

/**
 * @brief 输入源标识。
 */
typedef enum
{
    InputSourceNone = 0,    /**< 无输入源。 */
    InputSourceRc = 1,      /**< 遥控器输入源。 */
    InputSourceVision = 2,  /**< 视觉输入源。 */
    InputSourceSlam = 3,    /**< SLAM 输入源。 */
    InputSourcePlanner = 4  /**< planner 输入源。 */
} InputSource;

/**
 * @brief 单轴参考。
 */
typedef struct
{
    float32_t value;      /**< 参考值。 */
    uint8_t valid;        /**< 非 0 表示该轴有效。 */
    uint8_t priority;     /**< 仲裁优先级，数值越大优先级越高。 */
    InputSource source;   /**< 产生该参考的输入源。 */
    uint32_t tick_ms;     /**< 参考时间戳，单位 ms。 */
} InputAxisRef;

/**
 * @brief 云台统一参考。
 */
typedef struct
{
    InputAxisRef yaw;     /**< yaw 参考。 */
    InputAxisRef pitch;   /**< pitch 参考。 */
    GimbalMode mode;      /**< 云台模式参考。 */
    uint8_t mode_valid;   /**< 非 0 表示 mode 有效。 */
    uint8_t priority;     /**< mode、online、force_stop 仲裁优先级。 */
    uint8_t online;       /**< 非 0 表示该参考源在线。 */
    uint8_t force_stop;   /**< 非 0 表示强制停止云台命令输出。 */
    InputSource source;   /**< 产生该参考的输入源。 */
    uint32_t valid_mask;  /**< INPUT_GIMBAL_* 有效位。 */
    uint32_t tick_ms;     /**< 参考时间戳，单位 ms。 */
} InputGimbalRef;

/**
 * @brief 底盘统一参考。
 */
typedef struct
{
    InputAxisRef vx;          /**< 纵向速度参考。 */
    InputAxisRef vy;          /**< 横向速度参考。 */
    InputAxisRef wz;          /**< 角速度参考。 */
    InputAxisRef yaw_ref;     /**< yaw 角参考。 */
    InputAxisRef frame_angle; /**< 速度坐标系旋转角参考。 */
    InputAxisRef leg_length;  /**< 平衡底盘统一腿长参考。 */
    InputAxisRef leg_left;    /**< 平衡底盘左腿参考。 */
    InputAxisRef leg_right;   /**< 平衡底盘右腿参考。 */
    ChassisMode mode;         /**< 底盘模式参考。 */
    uint8_t mode_valid;       /**< 非 0 表示 mode 有效。 */
    uint8_t priority;         /**< mode、online、force_stop 仲裁优先级。 */
    uint8_t online;           /**< 非 0 表示该参考源在线。 */
    uint8_t force_stop;       /**< 非 0 表示强制停止底盘命令输出。 */
    uint8_t jump;             /**< 非 0 表示跳跃请求。 */
    InputSource source;       /**< 产生该参考的输入源。 */
    uint32_t valid_mask;      /**< INPUT_CHASSIS_* 有效位。 */
    uint32_t tick_ms;         /**< 参考时间戳，单位 ms。 */
} InputChassisRef;

/**
 * @brief 发射统一参考。
 */
typedef struct
{
    uint8_t online;      /**< 非 0 表示该参考源在线。 */
    uint8_t force_stop;  /**< 非 0 表示强制停止发射命令输出。 */
    uint8_t fric_enable; /**< 非 0 表示请求启动摩擦轮。 */
    uint8_t fire_once;   /**< 非 0 表示请求单发。 */
    uint8_t fire_hold;   /**< 非 0 表示请求连发。 */
    uint8_t priority;    /**< 仲裁优先级，数值越大优先级越高。 */
    InputSource source;  /**< 产生该参考的输入源。 */
    uint32_t valid_mask; /**< INPUT_SHOOT_* 有效位。 */
    uint32_t tick_ms;    /**< 参考时间戳，单位 ms。 */
} InputShootRef;

/**
 * @brief 弹舱动作。
 */
typedef enum
{
    InputMagHold = 0,  /**< 保持当前弹舱状态。 */
    InputMagOpen = 1,  /**< 打开弹舱。 */
    InputMagClose = 2  /**< 关闭弹舱。 */
} InputMagAction;

/**
 * @brief 弹舱统一参考。
 */
typedef struct
{
    InputMagAction action; /**< 弹舱动作参考。 */
    uint8_t online;        /**< 非 0 表示该参考源在线。 */
    uint8_t action_valid;  /**< 非 0 表示 action 有效。 */
    uint8_t priority;      /**< 仲裁优先级，数值越大优先级越高。 */
    InputSource source;    /**< 产生该参考的输入源。 */
    uint32_t valid_mask;   /**< INPUT_MAG_* 有效位。 */
    uint32_t tick_ms;      /**< 参考时间戳，单位 ms。 */
} InputMagRef;

/**
 * @brief 弹舱输出命令。
 */
typedef struct
{
    InputMagAction action; /**< 弹舱动作。 */
    uint8_t online;        /**< 非 0 表示命令源在线。 */
    uint8_t valid;         /**< 非 0 表示本周期存在有效动作命令。 */
} InputMagCmd;

/**
 * @brief 校准动作。
 */
typedef enum
{
    InputCaliNone = 0,   /**< 无校准动作。 */
    InputCaliGimbal = 1, /**< 触发云台限位校准。 */
    InputCaliImu = 2     /**< 触发 IMU 静态校准。 */
} InputCaliAction;

/**
 * @brief 校准统一参考。
 */
typedef struct
{
    InputCaliAction action; /**< 校准动作参考。 */
    uint8_t online;         /**< 非 0 表示该参考源在线。 */
    uint8_t action_valid;   /**< 非 0 表示 action 有效。 */
    uint8_t priority;       /**< 仲裁优先级，数值越大优先级越高。 */
    InputSource source;     /**< 产生该参考的输入源。 */
    uint32_t valid_mask;    /**< INPUT_CALI_* 有效位。 */
    uint32_t tick_ms;       /**< 参考时间戳，单位 ms。 */
} InputCaliRef;

/**
 * @brief 校准输出命令。
 */
typedef struct
{
    InputCaliAction action; /**< 校准动作。 */
    uint8_t online;         /**< 非 0 表示命令源在线。 */
    uint8_t valid;          /**< 非 0 表示本周期存在有效动作命令。 */
} InputCaliCmd;

/**
 * @brief 整车统一输入参考。
 */
typedef struct
{
    InputGimbalRef gimbal;   /**< 云台参考。 */
    InputChassisRef chassis; /**< 底盘参考。 */
    InputShootRef shoot;     /**< 发射参考。 */
    InputMagRef mag;         /**< 弹舱参考。 */
    InputCaliRef cali;       /**< 校准参考。 */
    uint32_t source_mask;    /**< 已参与仲裁的输入源位图。 */
    uint32_t tick_ms;        /**< 聚合时间戳，单位 ms。 */
} InputRef;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 清空单轴参考。
 *
 * @param axis 单轴参考指针。
 */
void input_axis_clear(InputAxisRef *axis);

/**
 * @brief 写入单轴参考。
 *
 * @param axis 单轴参考指针。
 * @param value 参考值。
 * @param priority 优先级。
 * @param source 输入源。
 * @param tick_ms 时间戳。
 */
void input_axis_set(InputAxisRef *axis,
                    float32_t value,
                    uint8_t priority,
                    InputSource source,
                    uint32_t tick_ms);

/**
 * @brief 按优先级选择单轴参考。
 *
 * @param dst 当前仲裁结果。
 * @param src 候选参考。
 */
void input_axis_choose(InputAxisRef *dst, const InputAxisRef *src);

/**
 * @brief 清空云台参考。
 *
 * @param ref 云台参考指针。
 */
void input_gimbal_clear(InputGimbalRef *ref);

/**
 * @brief 清空底盘参考。
 *
 * @param ref 底盘参考指针。
 */
void input_chassis_clear(InputChassisRef *ref);

/**
 * @brief 清空发射参考。
 *
 * @param ref 发射参考指针。
 */
void input_shoot_clear(InputShootRef *ref);

/**
 * @brief 清空弹舱参考。
 *
 * @param ref 弹舱参考指针。
 */
void input_mag_clear(InputMagRef *ref);

/**
 * @brief 清空校准参考。
 *
 * @param ref 校准参考指针。
 */
void input_cali_clear(InputCaliRef *ref);

/**
 * @brief 清空整车输入参考。
 *
 * @param ref 整车输入参考指针。
 */
void input_ref_clear(InputRef *ref);

/**
 * @brief 合并云台参考。
 *
 * @param dst 当前仲裁结果。
 * @param src 候选云台参考。
 */
void input_merge_gimbal(InputGimbalRef *dst, const InputGimbalRef *src);

/**
 * @brief 合并底盘参考。
 *
 * @param dst 当前仲裁结果。
 * @param src 候选底盘参考。
 */
void input_merge_chassis(InputChassisRef *dst, const InputChassisRef *src);

/**
 * @brief 合并发射参考。
 *
 * @param dst 当前仲裁结果。
 * @param src 候选发射参考。
 */
void input_merge_shoot(InputShootRef *dst, const InputShootRef *src);

/**
 * @brief 合并弹舱参考。
 *
 * @param dst 当前仲裁结果。
 * @param src 候选弹舱参考。
 */
void input_merge_mag(InputMagRef *dst, const InputMagRef *src);

/**
 * @brief 将校准参考按优先级合并到目标参考。
 *
 * @param dst 目标校准参考。
 * @param src 源校准参考。
 */
void input_merge_cali(InputCaliRef *dst, const InputCaliRef *src);

/**
 * @brief 根据云台参考生成机构命令。
 *
 * @param ref 云台参考。
 * @param cmd 云台命令输出。
 * @return 执行状态。
 */
InputStatus input_make_gimbal(const InputGimbalRef *ref, GimbalCmd *cmd);

/**
 * @brief 根据底盘参考生成机构命令。
 *
 * @param ref 底盘参考。
 * @param cmd 底盘命令输出。
 * @return 执行状态。
 */
InputStatus input_make_chassis(const InputChassisRef *ref, ChassisCmd *cmd);

/**
 * @brief 根据发射参考生成机构命令。
 *
 * @param ref 发射参考。
 * @param cmd 发射命令输出。
 * @return 执行状态。
 */
InputStatus input_make_shoot(const InputShootRef *ref, ShootCmd *cmd);

/**
 * @brief 根据弹舱参考生成弹舱命令。
 *
 * @param ref 弹舱参考。
 * @param cmd 弹舱命令输出。
 * @return 执行状态。
 */
InputStatus input_make_mag(const InputMagRef *ref, InputMagCmd *cmd);

/**
 * @brief 将校准参考转换为校准命令。
 *
 * @param ref 校准参考。
 * @param cmd 校准命令输出。
 * @return 执行状态。
 */
InputStatus input_make_cali(const InputCaliRef *ref, InputCaliCmd *cmd);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_INPUT_COMMON_H

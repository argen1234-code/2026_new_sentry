/**
 ******************************************************************************
 * @file    infantry_steer_2025_param.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   2025 舵轮步兵运行时参数配置。
 *
 * @note    本文件放输入映射、控制器参数、观测/温控/UI/调试/通讯等运行时配置。
 * @note    硬件接线、总线 ID 和 GPIO/TIM 映射不放在这里。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_ROBOT_INFANTRY_STEER_2025_PARAM_H
#define PRINTK_ROBOT_INFANTRY_STEER_2025_PARAM_H

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 通用运行参数
 * --------------------------------------------------------------------------
 * _ref 默认步兵中与整车行为相关的少量全局参数。
 * ========================================================================== */

#define ROBOT_SPIN_FACTOR                         10.0F                              /**< 小陀螺速度系数。 */
#define ROBOT_SPIN_SPEED                          0.7F                               /**< 小陀螺默认角速度系数。 */

/* ==========================================================================
 * Input 映射
 * --------------------------------------------------------------------------
 * 描述遥控器通道、按键和灵敏度如何映射为当前 target 的人工输入参考。
 * ========================================================================== */

#define ROBOT_INPUT_CHASSIS_X_CHANNEL             1U                                 /**< 底盘 vx 遥控通道。 */
#define ROBOT_INPUT_CHASSIS_Y_CHANNEL             0U                                 /**< 底盘 vy 遥控通道。 */
#define ROBOT_INPUT_CHASSIS_WZ_CHANNEL            2U                                 /**< 底盘 wz 遥控通道。 */
#define ROBOT_INPUT_CHASSIS_MODE_CHANNEL          0U                                 /**< 底盘模式拨杆通道。 */
#define ROBOT_INPUT_CHASSIS_FOLLOW_CHANNEL        1U                                 /**< 底盘跟随拨杆通道。 */
#define ROBOT_INPUT_CHASSIS_VX_RC_SEN             0.006F                             /**< 底盘 vx 遥控灵敏度。 */
#define ROBOT_INPUT_CHASSIS_VY_RC_SEN             0.005F                             /**< 底盘 vy 遥控灵敏度。 */
#define ROBOT_INPUT_CHASSIS_ANGLE_Z_RC_SEN        0.000002F                          /**< 底盘 yaw 角度遥控灵敏度。 */
#define ROBOT_INPUT_CHASSIS_WZ_RC_SEN             0.01F                              /**< 底盘 wz 遥控灵敏度。 */
#define ROBOT_INPUT_CHASSIS_RC_DEADBAND           25                                 /**< 底盘遥控死区。 */
#define ROBOT_INPUT_CHASSIS_SWING_KEY             RC_KEY_CTRL                        /**< 底盘摆动模式按键。 */
#define ROBOT_INPUT_CHASSIS_STOP_KEY              RC_KEY_X                           /**< 底盘停止覆盖按键。 */
#define ROBOT_INPUT_CHASSIS_SPIN_KEY              RC_KEY_SHIFT                       /**< 底盘小陀螺覆盖按键。 */
#define ROBOT_INPUT_CHASSIS_FOLLOW_KEY            RC_KEY_C                           /**< 底盘跟随云台覆盖按键。 */
#define ROBOT_INPUT_CHASSIS_FRONT_KEY             RC_KEY_W                           /**< 底盘前进按键。 */
#define ROBOT_INPUT_CHASSIS_BACK_KEY              RC_KEY_S                           /**< 底盘后退按键。 */
#define ROBOT_INPUT_CHASSIS_LEFT_KEY              RC_KEY_A                           /**< 底盘左移按键。 */
#define ROBOT_INPUT_CHASSIS_RIGHT_KEY             RC_KEY_D                           /**< 底盘右移按键。 */
#define ROBOT_INPUT_CHASSIS_KEY_VX_MAX            ROBOT_CHASSIS_MAX_SPEED_X          /**< 键盘前进速度参考。 */
#define ROBOT_INPUT_CHASSIS_KEY_VX_MIN            (-ROBOT_CHASSIS_MAX_SPEED_X)       /**< 键盘后退速度参考。 */
#define ROBOT_INPUT_CHASSIS_KEY_VY_MAX            ROBOT_CHASSIS_MAX_SPEED_Y          /**< 键盘左移速度参考。 */
#define ROBOT_INPUT_CHASSIS_KEY_VY_MIN            (-ROBOT_CHASSIS_MAX_SPEED_Y)       /**< 键盘右移速度参考。 */
#define ROBOT_INPUT_CHASSIS_SWING_STEP            0.0031415927F                      /**< 底盘摆动相位步进，单位 rad/周期。 */
#define ROBOT_INPUT_GIMBAL_YAW_CHANNEL            2U                                 /**< 云台 yaw 遥控通道。 */
#define ROBOT_INPUT_GIMBAL_PITCH_CHANNEL          3U                                 /**< 云台 pitch 遥控通道。 */
#define ROBOT_INPUT_GIMBAL_MODE_CHANNEL           0U                                 /**< 云台模式拨杆通道。 */
#define ROBOT_INPUT_GIMBAL_WZ_CHANNEL             2U                                 /**< 云台小陀螺遥控通道。 */
#define ROBOT_INPUT_GIMBAL_ZERO_KEY               RC_KEY_X                           /**< 云台零力覆盖按键。 */
#define ROBOT_INPUT_GIMBAL_SPIN_KEY               RC_KEY_SHIFT                       /**< 云台小陀螺覆盖按键。 */
#define ROBOT_INPUT_GIMBAL_RELATIVE_KEY           RC_KEY_C                           /**< 云台相对角覆盖按键。 */
#define ROBOT_INPUT_GIMBAL_TURN_KEY               RC_KEY_F                           /**< 云台反转按键。 */
#define ROBOT_INPUT_GIMBAL_TEST_KEY               RC_KEY_R                           /**< 云台测试按键。 */
#define ROBOT_INPUT_GIMBAL_TURN_DONE_ERR          0.01F                              /**< 云台反转完成误差，单位 rad。 */
#define ROBOT_INPUT_GIMBAL_RC_DEADBAND            10                                 /**< 云台遥控死区。 */
#define ROBOT_INPUT_GIMBAL_YAW_RC_SEN             (-0.000005F)                       /**< 云台 yaw 遥控灵敏度。 */
#define ROBOT_INPUT_GIMBAL_PITCH_RC_SEN           (-0.000006F)                       /**< 云台 pitch 遥控灵敏度。 */
#define ROBOT_INPUT_GIMBAL_YAW_MOUSE_SEN          0.00005F                           /**< 云台 yaw 鼠标灵敏度。 */
#define ROBOT_INPUT_GIMBAL_PITCH_MOUSE_SEN        0.00015F                           /**< 云台 pitch 鼠标灵敏度。 */
#define ROBOT_INPUT_SHOOT_MODE_CHANNEL            1U                                 /**< 发射模式拨杆通道。 */
#define ROBOT_INPUT_SHOOT_ON_KEY                  RC_KEY_Q                           /**< 发射开启按键。 */
#define ROBOT_INPUT_SHOOT_OFF_KEY                 RC_KEY_E                           /**< 发射关闭按键。 */
#define ROBOT_INPUT_SHOOT_PRESS_LONG_TIME         ROBOT_SHOOT_PRESS_LONG_TIME        /**< 鼠标长按连发判定周期。 */
#define ROBOT_INPUT_SHOOT_SWITCH_LONG_TIME        ROBOT_SHOOT_RC_S_LONG_TIME         /**< 拨杆长按连发判定周期。 */
#define ROBOT_INPUT_MAGAZINE_OPEN_KEY             RC_KEY_R                           /**< 弹舱打开按键。 */
#define ROBOT_INPUT_MAGAZINE_CLOSE_KEY            RC_KEY_G                           /**< 弹舱关闭按键。 */
#define ROBOT_INPUT_MAGAZINE_MODE_CHANNEL         ROBOT_INPUT_SHOOT_MODE_CHANNEL     /**< 弹舱拨杆动作使用发射模式拨杆。 */
#define ROBOT_INPUT_MAGAZINE_GUARD_CHANNEL        ROBOT_INPUT_GIMBAL_MODE_CHANNEL    /**< 弹舱拨杆动作保护拨杆。 */
#define ROBOT_INPUT_MAGAZINE_GUARD_VALUE          RC_SW_MID                          /**< 弹舱拨杆动作要求云台模式拨杆在中档。 */
#define ROBOT_INPUT_MAGAZINE_OPEN_SW              RC_SW_DOWN                         /**< 发射拨杆下档打开弹舱。 */
#define ROBOT_INPUT_MAGAZINE_CLOSE_SW             RC_SW_MID                          /**< 发射拨杆中档关闭弹舱。 */
#define ROBOT_INPUT_CALI_ENABLE_OLD_GESTURE       1U                                 /**< 启用旧摇杆组合校准入口。 */
#define ROBOT_INPUT_CALI_BEGIN_COUNT              100U                               /**< 旧校准入口进入手势保持周期。 */
#define ROBOT_INPUT_CALI_ACTION_COUNT             100U                               /**< 旧校准动作手势保持周期。 */
#define ROBOT_INPUT_CALI_GIMBAL_KEY               RC_KEY_B                           /**< 云台校准快捷键。 */
#define ROBOT_INPUT_CALI_IMU_KEY                  RC_KEY_V                           /**< IMU 校准快捷键。 */

/* ==========================================================================
 * Chassis 参数
 * --------------------------------------------------------------------------
 * _ref Steering_wheel 底盘参数，后续 App/Profile 应据此填充 ChassisCfg。
 * ========================================================================== */

#define ROBOT_CHASSIS_TASK_INIT_TIME              357U                               /**< 底盘任务旧初始化延时，单位 ms。 */
#define ROBOT_CHASSIS_ACCEL_X_NUM                 0.1666666667F                      /**< 底盘 vx 加速度平滑系数。 */
#define ROBOT_CHASSIS_ACCEL_Y_NUM                 0.3333333333F                      /**< 底盘 vy 加速度平滑系数。 */
#define ROBOT_CHASSIS_MOTOR_SPEED_TO_VX           0.25F                              /**< 旧轮速到 vx 换算。 */
#define ROBOT_CHASSIS_MOTOR_SPEED_TO_VY           0.25F                              /**< 旧轮速到 vy 换算。 */
#define ROBOT_CHASSIS_MOTOR_SPEED_TO_WZ           0.25F                              /**< 旧轮速到 wz 换算。 */
#define ROBOT_CHASSIS_CONTROL_TIME_MS             2U                                 /**< 底盘控制周期，单位 ms。 */
#define ROBOT_CHASSIS_CONTROL_TIME                0.002F                             /**< 底盘控制周期，单位 s。 */
#define ROBOT_CHASSIS_CONTROL_FREQ                500.0F                             /**< 底盘控制频率，单位 Hz。 */
#define ROBOT_CHASSIS_MAX_CURRENT                 16000.0F                           /**< DJI 电流输出限幅。 */
#define ROBOT_CHASSIS_RPM_TO_VECTOR               0.000415809748903494517209F        /**< M3508 rpm 到线速度换算。 */
#define ROBOT_CHASSIS_MAX_WHEEL_SPEED             4.0F                               /**< 最大轮速。 */
#define ROBOT_CHASSIS_MAX_SPEED_X                 2.5F                               /**< 普通模式最大 vx。 */
#define ROBOT_CHASSIS_MAX_SPEED_Y                 2.5F                               /**< 普通模式最大 vy。 */
#define ROBOT_CHASSIS_WZ_SET_SCALE                0.1F                               /**< wz 目标缩放。 */
#define ROBOT_CHASSIS_CENTER_DIST                 0.22F                              /**< 普通轮底盘轮到中心距离，单位 m。 */
#define ROBOT_CHASSIS_HALF_LENGTH                 0.20F                              /**< 舵轮底盘半车长，单位 m。 */
#define ROBOT_CHASSIS_HALF_WIDTH                  0.175F                             /**< 舵轮底盘半车宽，单位 m。 */
#define ROBOT_CHASSIS_SWING_NO_MOVE_ANGLE         0.7F                               /**< 静止摆动角度。 */
#define ROBOT_CHASSIS_SWING_MOVE_ANGLE            0.3141592653589793238462643383F    /**< 运动摆动角度。 */
#define ROBOT_CHASSIS_DRIVE_PID_KP                15000.0F                           /**< 驱动轮速度环 Kp。 */
#define ROBOT_CHASSIS_DRIVE_PID_KI                10.0F                              /**< 驱动轮速度环 Ki。 */
#define ROBOT_CHASSIS_DRIVE_PID_KD                0.0F                               /**< 驱动轮速度环 Kd。 */
#define ROBOT_CHASSIS_DRIVE_PID_MAX_OUT           ROBOT_CHASSIS_MAX_CURRENT          /**< 驱动轮速度环输出限幅。 */
#define ROBOT_CHASSIS_DRIVE_PID_MAX_IOUT          2000.0F                            /**< 驱动轮速度环积分限幅。 */
#define ROBOT_CHASSIS_STEER_ANGLE_RATIO           1303.63813886F                     /**< GM6020 弧度到编码值换算。 */
#define ROBOT_CHASSIS_MPS_TO_RPM                  3234.16461897F                     /**< 舵轮线速度到 rpm 换算。 */
#define ROBOT_CHASSIS_STEER_ANGLE_PID_KP          100.0F                             /**< 舵向角度环 Kp。 */
#define ROBOT_CHASSIS_STEER_ANGLE_PID_KI          0.01F                              /**< 舵向角度环 Ki。 */
#define ROBOT_CHASSIS_STEER_ANGLE_PID_KD          0.0F                               /**< 舵向角度环 Kd。 */
#define ROBOT_CHASSIS_STEER_ANGLE_PID_MAX_OUT     120.0F                             /**< 舵向角度环输出限幅。 */
#define ROBOT_CHASSIS_STEER_ANGLE_PID_MAX_IOUT    10.0F                              /**< 舵向角度环积分限幅。 */
#define ROBOT_CHASSIS_YAW_RETURN_PID_KP           450.0F                             /**< 舵轮回正 yaw Kp。 */
#define ROBOT_CHASSIS_YAW_RETURN_PID_KI           0.021F                             /**< 舵轮回正 yaw Ki。 */
#define ROBOT_CHASSIS_YAW_RETURN_PID_KD           20.0F                              /**< 舵轮回正 yaw Kd。 */
#define ROBOT_CHASSIS_YAW_RETURN_PID_MAX_OUT      600.0F                             /**< 舵轮回正 yaw 输出限幅。 */
#define ROBOT_CHASSIS_YAW_RETURN_PID_MAX_IOUT     28.0F                              /**< 舵轮回正 yaw 积分限幅。 */
#define ROBOT_CHASSIS_STEER_SPEED_PID_KP          55.0F                              /**< 舵向速度环 Kp。 */
#define ROBOT_CHASSIS_STEER_SPEED_PID_KI          2.4F                               /**< 舵向速度环 Ki。 */
#define ROBOT_CHASSIS_STEER_SPEED_PID_KD          80.0F                              /**< 舵向速度环 Kd。 */
#define ROBOT_CHASSIS_STEER_SPEED_PID_MAX_OUT     8000.0F                            /**< 舵向速度环输出限幅。 */
#define ROBOT_CHASSIS_STEER_SPEED_PID_MAX_IOUT    4000.0F                            /**< 舵向速度环积分限幅。 */
#define ROBOT_CHASSIS_STEER_CURRENT_MAX           ROBOT_CHASSIS_STEER_SPEED_PID_MAX_OUT /**< 舵向电机电流限幅。 */
#define ROBOT_CHASSIS_STEER_ZERO_0                1.95F                              /**< 舵轮 0 机械零位，单位 rad。 */
#define ROBOT_CHASSIS_STEER_ZERO_1                (-1.80F)                           /**< 舵轮 1 机械零位，单位 rad。 */
#define ROBOT_CHASSIS_STEER_ZERO_2                0.90F                              /**< 舵轮 2 机械零位，单位 rad。 */
#define ROBOT_CHASSIS_STEER_ZERO_3                3.30F                              /**< 舵轮 3 机械零位，单位 rad。 */
#define ROBOT_CHASSIS_DRIVE_POLARITY_0            1                                  /**< 驱动 0 电流极性。 */
#define ROBOT_CHASSIS_DRIVE_POLARITY_1            1                                  /**< 驱动 1 电流极性。 */
#define ROBOT_CHASSIS_DRIVE_POLARITY_2            1                                  /**< 驱动 2 电流极性。 */
#define ROBOT_CHASSIS_DRIVE_POLARITY_3            1                                  /**< 驱动 3 电流极性。 */
#define ROBOT_CHASSIS_STEER_POLARITY_0            1                                  /**< 舵向 0 电流极性。 */
#define ROBOT_CHASSIS_STEER_POLARITY_1            1                                  /**< 舵向 1 电流极性。 */
#define ROBOT_CHASSIS_STEER_POLARITY_2            1                                  /**< 舵向 2 电流极性。 */
#define ROBOT_CHASSIS_STEER_POLARITY_3            1                                  /**< 舵向 3 电流极性。 */
#define ROBOT_CHASSIS_FOLLOW_PID_KP               10.0F                              /**< 跟随云台 yaw Kp。 */
#define ROBOT_CHASSIS_FOLLOW_PID_KI               0.0F                               /**< 跟随云台 yaw Ki。 */
#define ROBOT_CHASSIS_FOLLOW_PID_KD               0.1F                               /**< 跟随云台 yaw Kd。 */
#define ROBOT_CHASSIS_FOLLOW_PID_MAX_OUT          6.0F                               /**< 跟随云台 yaw 输出限幅。 */
#define ROBOT_CHASSIS_FOLLOW_PID_MAX_IOUT         0.2F                               /**< 跟随云台 yaw 积分限幅。 */
#define ROBOT_CHASSIS_POWER_LIMIT                 90.0F                              /**< 底盘功率预算默认值，单位 W。 */
#define ROBOT_POWER_CHASSIS_K1                    1.453e-07F                         /**< 底盘预测功率速度平方项系数。 */
#define ROBOT_POWER_CHASSIS_K2                    1.23e-07F                          /**< 底盘预测功率电流平方项系数。 */
#define ROBOT_POWER_CHASSIS_OFFSET                4.081F                             /**< 底盘预测功率静态偏置，单位 W。 */
#define ROBOT_POWER_CHASSIS_TORQUE                (0.01562F * 0.001220703125F)       /**< DJI 电流命令到转矩的等效系数。 */
#define ROBOT_POWER_CHASSIS_RPM_RAD               9.55F                              /**< rpm 到 rad/s 换算分母。 */
#define ROBOT_POWER_CHASSIS_RESERVE               16.0F                              /**< 给舵向 6020 等附属电机预留的功率，单位 W。 */
#define ROBOT_POWER_CHASSIS_SCALE_MIN             0.01F                              /**< 功率超限时最小电流缩放系数。 */
#define ROBOT_POWER_CHASSIS_SCALE_FALLBACK        0.5F                               /**< 二次方程求解异常时的默认缩放系数。 */
#define CHASSIS_CURRENT_MAX_DEFAULT               ROBOT_CHASSIS_MAX_CURRENT          /**< Controller 默认驱动电流限幅。 */
#define CHASSIS_WHEEL_SPEED_MAX_DEFAULT           ROBOT_CHASSIS_MAX_WHEEL_SPEED      /**< Controller 默认轮速限幅。 */
#define CHASSIS_DRIVE_PID_KP_DEFAULT              ROBOT_CHASSIS_DRIVE_PID_KP         /**< Controller 默认驱动 PID Kp。 */
#define CHASSIS_DRIVE_PID_KI_DEFAULT              ROBOT_CHASSIS_DRIVE_PID_KI         /**< Controller 默认驱动 PID Ki。 */
#define CHASSIS_DRIVE_PID_KD_DEFAULT              ROBOT_CHASSIS_DRIVE_PID_KD         /**< Controller 默认驱动 PID Kd。 */
#define CHASSIS_DRIVE_PID_MAX_DEFAULT             ROBOT_CHASSIS_DRIVE_PID_MAX_OUT    /**< Controller 默认驱动 PID 输出限幅。 */
#define CHASSIS_STEER_ANGLE_KP_DEFAULT            ROBOT_CHASSIS_STEER_ANGLE_PID_KP   /**< Controller 默认舵向角度 Kp。 */
#define CHASSIS_STEER_ANGLE_MAX_DEFAULT           ROBOT_CHASSIS_STEER_ANGLE_PID_MAX_OUT /**< Controller 默认舵向角度输出限幅。 */
#define CHASSIS_STEER_SPEED_KP_DEFAULT            ROBOT_CHASSIS_STEER_SPEED_PID_KP   /**< Controller 默认舵向速度 Kp。 */
#define CHASSIS_STEER_SPEED_MAX_DEFAULT           ROBOT_CHASSIS_STEER_SPEED_PID_MAX_OUT /**< Controller 默认舵向速度输出限幅。 */
#define CHASSIS_YAW_PID_KP_DEFAULT                ROBOT_CHASSIS_FOLLOW_PID_KP        /**< Controller 默认 yaw Kp。 */
#define CHASSIS_YAW_PID_KD_DEFAULT                ROBOT_CHASSIS_FOLLOW_PID_KD        /**< Controller 默认 yaw Kd。 */
#define CHASSIS_YAW_PID_MAX_DEFAULT               ROBOT_CHASSIS_FOLLOW_PID_MAX_OUT   /**< Controller 默认 yaw 输出限幅。 */
#define CHASSIS_RPM_TO_SPEED_DEFAULT              ROBOT_CHASSIS_RPM_TO_VECTOR        /**< Controller 默认 rpm 到速度换算。 */
#define CHASSIS_STEER_RPM_TO_SPEED_DEFAULT        ROBOT_CHASSIS_MPS_TO_RPM           /**< Controller 默认舵轮速度换算。 */
#define CHASSIS_STEER_ECD_TO_RAD_DEFAULT          0.000766990394F                    /**< Controller 默认舵向编码到弧度换算。 */
#define CHASSIS_WZ_SCALE_DEFAULT                  ROBOT_CHASSIS_WZ_SET_SCALE         /**< Controller 默认 wz 缩放。 */

/* ==========================================================================
 * Gimbal 参数
 * --------------------------------------------------------------------------
 * _ref yaw_pitch_direct 云台参数，后续 App/Profile 应据此填充 GimbalCfg。
 * ========================================================================== */

#define ROBOT_GIMBAL_PITCH_TURN                   0U                                 /**< pitch 电机安装反向标志。 */
#define ROBOT_GIMBAL_YAW_TURN                     0U                                 /**< yaw 电机安装反向标志。 */
#define ROBOT_GIMBAL_FEEDFORWARD_GAIN             0.00F                              /**< 旧前馈增益。 */
#define ROBOT_GIMBAL_PITCH_SPEED_PID_KP           3000.0F                            /**< pitch 速度环 Kp。 */
#define ROBOT_GIMBAL_PITCH_SPEED_PID_KI           80.0F                              /**< pitch 速度环 Ki。 */
#define ROBOT_GIMBAL_PITCH_SPEED_PID_KD           0.0F                               /**< pitch 速度环 Kd。 */
#define ROBOT_GIMBAL_PITCH_SPEED_PID_MAX_OUT      30000.0F                           /**< pitch 速度环输出限幅。 */
#define ROBOT_GIMBAL_PITCH_SPEED_PID_MAX_IOUT     10000.0F                           /**< pitch 速度环积分限幅。 */
#define ROBOT_GIMBAL_YAW_SPEED_PID_KP             3600.0F                            /**< yaw 速度环 Kp。 */
#define ROBOT_GIMBAL_YAW_SPEED_PID_KI             20.0F                              /**< yaw 速度环 Ki。 */
#define ROBOT_GIMBAL_YAW_SPEED_PID_KD             0.0F                               /**< yaw 速度环 Kd。 */
#define ROBOT_GIMBAL_YAW_SPEED_PID_MAX_OUT        30000.0F                           /**< yaw 速度环输出限幅。 */
#define ROBOT_GIMBAL_YAW_SPEED_PID_MAX_IOUT       5000.0F                            /**< yaw 速度环积分限幅。 */
#define ROBOT_GIMBAL_PITCH_ABS_PID_KP             15.0F                              /**< pitch 绝对角环 Kp。 */
#define ROBOT_GIMBAL_PITCH_ABS_PID_KI             0.0F                               /**< pitch 绝对角环 Ki。 */
#define ROBOT_GIMBAL_PITCH_ABS_PID_KD             0.0F                               /**< pitch 绝对角环 Kd。 */
#define ROBOT_GIMBAL_PITCH_ABS_PID_MAX_OUT        10.0F                              /**< pitch 绝对角环输出限幅。 */
#define ROBOT_GIMBAL_PITCH_ABS_PID_MAX_IOUT       0.0F                               /**< pitch 绝对角环积分限幅。 */
#define ROBOT_GIMBAL_YAW_ABS_PID_KP               28.0F                              /**< yaw 绝对角环 Kp。 */
#define ROBOT_GIMBAL_YAW_ABS_PID_KI               0.0F                               /**< yaw 绝对角环 Ki。 */
#define ROBOT_GIMBAL_YAW_ABS_PID_KD               0.5F                               /**< yaw 绝对角环 Kd。 */
#define ROBOT_GIMBAL_YAW_ABS_PID_MAX_OUT          10.0F                              /**< yaw 绝对角环输出限幅。 */
#define ROBOT_GIMBAL_YAW_ABS_PID_MAX_IOUT         0.0F                               /**< yaw 绝对角环积分限幅。 */
#define ROBOT_GIMBAL_PITCH_REL_PID_KP             80.0F                              /**< pitch 相对角环 Kp。 */
#define ROBOT_GIMBAL_PITCH_REL_PID_KI             0.0F                               /**< pitch 相对角环 Ki。 */
#define ROBOT_GIMBAL_PITCH_REL_PID_KD             0.5F                               /**< pitch 相对角环 Kd。 */
#define ROBOT_GIMBAL_PITCH_REL_PID_MAX_OUT        10.0F                              /**< pitch 相对角环输出限幅。 */
#define ROBOT_GIMBAL_PITCH_REL_PID_MAX_IOUT       0.0F                               /**< pitch 相对角环积分限幅。 */
#define ROBOT_GIMBAL_YAW_REL_PID_KP               28.0F                              /**< yaw 相对角环 Kp。 */
#define ROBOT_GIMBAL_YAW_REL_PID_KI               0.0F                               /**< yaw 相对角环 Ki。 */
#define ROBOT_GIMBAL_YAW_REL_PID_KD               0.5F                               /**< yaw 相对角环 Kd。 */
#define ROBOT_GIMBAL_YAW_REL_PID_MAX_OUT          10.0F                              /**< yaw 相对角环输出限幅。 */
#define ROBOT_GIMBAL_YAW_REL_PID_MAX_IOUT         0.0F                               /**< yaw 相对角环积分限幅。 */
#define ROBOT_GIMBAL_YAW_CURRENT_POLARITY         1                                  /**< yaw 电流输出极性。 */
#define ROBOT_GIMBAL_PITCH_CURRENT_POLARITY       1                                  /**< pitch 电流输出极性。 */
#define ROBOT_GIMBAL_TASK_INIT_TIME               200U                               /**< 云台旧初始化延时，单位 ms。 */
#define ROBOT_GIMBAL_TURN_SPEED                   0.04F                              /**< 云台反转速度。 */
#define ROBOT_GIMBAL_YAW_ENCODE_SEN               0.01F                              /**< yaw 编码输入灵敏度。 */
#define ROBOT_GIMBAL_PITCH_ENCODE_SEN             0.01F                              /**< pitch 编码输入灵敏度。 */
#define ROBOT_GIMBAL_CONTROL_TIME_MS              1U                                 /**< 云台控制周期，单位 ms。 */
#define ROBOT_GIMBAL_TEST_MODE                    0U                                 /**< 云台测试模式标志。 */
#define ROBOT_GIMBAL_ECD_HALF_RANGE               4096                               /**< 云台编码半圈阈值。 */
#define ROBOT_GIMBAL_ECD_RANGE                    8191                               /**< 云台编码最大值。 */
#define ROBOT_GIMBAL_INIT_ANGLE_ERROR             0.1F                               /**< 云台初始化角度误差阈值。 */
#define ROBOT_GIMBAL_INIT_STOP_TIME               100U                               /**< 云台初始化停止判定时间。 */
#define ROBOT_GIMBAL_INIT_TIME                    6000U                              /**< 云台初始化总时长。 */
#define ROBOT_GIMBAL_CALI_REDUNDANT_ANGLE         0.1F                               /**< 云台标定冗余角。 */
#define ROBOT_GIMBAL_INIT_PITCH_SPEED             0.004F                             /**< pitch 初始化速度。 */
#define ROBOT_GIMBAL_INIT_YAW_SPEED               0.005F                             /**< yaw 初始化速度。 */
#define ROBOT_GIMBAL_CALI_MOTOR_SET               8000                               /**< 云台标定电机输出。 */
#define ROBOT_GIMBAL_CALI_STEP_TIME               2000U                              /**< 云台标定步骤时长。 */
#define ROBOT_GIMBAL_CALI_GYRO_LIMIT              0.1F                               /**< 云台标定陀螺阈值。 */
#define ROBOT_GIMBAL_CALI_PITCH_MAX_STEP          1U                                 /**< pitch 最大角标定步骤。 */
#define ROBOT_GIMBAL_CALI_PITCH_MIN_STEP          2U                                 /**< pitch 最小角标定步骤。 */
#define ROBOT_GIMBAL_CALI_YAW_MAX_STEP            3U                                 /**< yaw 最大角标定步骤。 */
#define ROBOT_GIMBAL_CALI_YAW_MIN_STEP            4U                                 /**< yaw 最小角标定步骤。 */
#define ROBOT_GIMBAL_CALI_START_STEP              ROBOT_GIMBAL_CALI_PITCH_MAX_STEP   /**< 云台标定起始步骤。 */
#define ROBOT_GIMBAL_CALI_END_STEP                5U                                 /**< 云台标定结束步骤。 */
#define ROBOT_GIMBAL_MOTIONLESS_RC_DEADBAND       10                                 /**< 云台静止遥控死区。 */
#define ROBOT_GIMBAL_MOTIONLESS_TIME_MAX          3000U                              /**< 云台静止最大判定时间。 */
#define ROBOT_GIMBAL_INIT_YAW_SET                 0.0F                               /**< yaw 初始设定值。 */
#define ROBOT_GIMBAL_INIT_PITCH_SET               0.0F                               /**< pitch 初始设定值。 */
#define ROBOT_GIMBAL_MOTOR_ECD_TO_RAD             0.000766990394F                    /**< 云台电机编码到弧度换算。 */
#define GIMBAL_YAW_CORR_RATE_MAX                  2.0F                               /**< yaw 位置纠偏速度限幅。 */

/* ==========================================================================
 * Shoot 参数
 * --------------------------------------------------------------------------
 * _ref friction_3508 发射参数，后续 App/Profile 应据此填充 ShootCfg。
 * ========================================================================== */

#define ROBOT_SHOOT_CONTROL_TIME_MS               ROBOT_GIMBAL_CONTROL_TIME_MS       /**< 发射控制周期，单位 ms。 */
#define ROBOT_SHOOT_FRIC_PWM_ADD_VALUE            100.0F                             /**< 旧摩擦轮 PWM 增量。 */
#define ROBOT_SHOOT_DONE_KEY_OFF_TIME             10U                                /**< 发射完成后按键释放判定时间。 */
#define ROBOT_SHOOT_PRESS_LONG_TIME               2U                                 /**< 长按判定时间。 */
#define ROBOT_SHOOT_RC_S_LONG_TIME                2000U                              /**< 遥控拨杆长按判定时间。 */
#define ROBOT_SHOOT_UP_ADD_TIME                   80U                                /**< 旧上升补偿时间。 */
#define ROBOT_SHOOT_ECD_HALF_RANGE                4096                               /**< trigger 编码半圈阈值。 */
#define ROBOT_SHOOT_ECD_RANGE                     8191                               /**< trigger 编码最大值。 */
#define ROBOT_SHOOT_RPM_TO_SPEED                  0.00290888208665721596153948461415F /**< trigger rpm 到速度换算。 */
#define ROBOT_SHOOT_FRIC_RPM_TO_SPEED             (ROBOT_SHOOT_RPM_TO_SPEED / 4.0F)  /**< 摩擦轮 rpm 到控制速度换算。 */
#define ROBOT_SHOOT_ECD_TO_ANGLE                  0.000021305288720633905968306772076277F /**< trigger 编码到角度换算。 */
#define ROBOT_SHOOT_FULL_COUNT                    18U                                /**< 拨盘一圈弹丸数量。 */
#define ROBOT_SHOOT_TRIGGER_SPEED                 (-10.0F)                           /**< 单发 trigger 速度。 */
#define ROBOT_SHOOT_CONTINUE_TRIGGER_SPEED        (-2.0F)                            /**< 连发 trigger 速度。 */
#define ROBOT_SHOOT_READY_TRIGGER_SPEED           (-10.0F)                           /**< 准备 trigger 速度。 */
#define ROBOT_SHOOT_KEY_OFF_JUDGE_TIME            500U                               /**< 按键释放判定时间。 */
#define ROBOT_SHOOT_SWITCH_TRIGGER_ON             0U                                 /**< 微动开关触发值。 */
#define ROBOT_SHOOT_SWITCH_TRIGGER_OFF            1U                                 /**< 微动开关释放值。 */
#define ROBOT_SHOOT_BLOCK_TRIGGER_SPEED           1.0F                               /**< 堵转速度阈值。 */
#define ROBOT_SHOOT_BLOCK_TIME_MS                 700U                               /**< 堵转判定时间。 */
#define ROBOT_SHOOT_REVERSE_TIME_MS               500U                               /**< 堵转反转时间。 */
#define ROBOT_SHOOT_REVERSE_SPEED_LIMIT           13.0F                              /**< 堵转反转速度限幅。 */
#define ROBOT_SHOOT_PI_FOUR                       0.78539816339744830961566084581988F /**< 四分之一 pi。 */
#define ROBOT_SHOOT_STEP_ANGLE                    0.31415F                           /**< trigger 单次推进角度。 */
#define ROBOT_SHOOT_TRIGGER_DONE_ERR              0.05F                              /**< trigger 单发到位误差，单位 rad。 */
#define ROBOT_SHOOT_TRIGGER_PID_KP                5000.0F                            /**< trigger 角度环 Kp。 */
#define ROBOT_SHOOT_TRIGGER_PID_KI                2.0F                               /**< trigger 角度环 Ki。 */
#define ROBOT_SHOOT_TRIGGER_PID_KD                0.0F                               /**< trigger 角度环 Kd。 */
#define ROBOT_SHOOT_TRIGGER_BULLET_MAX_OUT        10000.0F                           /**< 单发 trigger 输出限幅。 */
#define ROBOT_SHOOT_TRIGGER_BULLET_MAX_IOUT       9000.0F                            /**< 单发 trigger 积分限幅。 */
#define ROBOT_SHOOT_TRIGGER_READY_MAX_OUT         10000.0F                           /**< 准备 trigger 输出限幅。 */
#define ROBOT_SHOOT_TRIGGER_READY_MAX_IOUT        7000.0F                            /**< 准备 trigger 积分限幅。 */
#define ROBOT_SHOOT_FRIC_PID_KP                   15000.0F                           /**< 摩擦轮速度环 Kp。 */
#define ROBOT_SHOOT_FRIC_PID_KI                   10.0F                              /**< 摩擦轮速度环 Ki。 */
#define ROBOT_SHOOT_FRIC_PID_KD                   0.0F                               /**< 摩擦轮速度环 Kd。 */
#define ROBOT_SHOOT_FRIC_MAX_OUT                  10000.0F                           /**< 摩擦轮输出限幅。 */
#define ROBOT_SHOOT_FRIC_MAX_IOUT                 2000.0F                            /**< 摩擦轮积分限幅。 */
#define ROBOT_SHOOT_FRIC_UP_SPEED                 (-4.45F)                           /**< 摩擦轮开启目标速度。 */
#define ROBOT_SHOOT_FRIC_DOWN_SPEED               0.0F                               /**< 摩擦轮关闭目标速度。 */
#define ROBOT_SHOOT_FRIC_READY_RATIO              0.95F                              /**< 摩擦轮达速比例。 */
#define ROBOT_SHOOT_TRIGGER_POLARITY              1                                  /**< trigger 电流输出极性。 */
#define ROBOT_SHOOT_FRIC_POLARITY_0               1                                  /**< 摩擦轮 0 电流输出极性。 */
#define ROBOT_SHOOT_FRIC_POLARITY_1               1                                  /**< 摩擦轮 1 电流输出极性。 */
#define ROBOT_SHOOT_FRIC_POLARITY_2               1                                  /**< 摩擦轮 2 电流输出极性。 */
#define ROBOT_SHOOT_FRIC_POLARITY_3               1                                  /**< 摩擦轮 3 电流输出极性。 */
#define ROBOT_SHOOT_HEAT_REMAIN_VALUE             80U                                /**< 热量剩余保护值。 */
#define ROBOT_SHOOT_LIMIT                         230U                               /**< 发射热量上限默认值。 */
#define ROBOT_SHOOT_COOL                          14U                                /**< 发射冷却默认值。 */
#define ROBOT_SHOOT_HEAT_PER_BULLET               10U                                /**< 单发热量默认值。 */
#define ROBOT_SHOOT_FRIC_TEMP_LIMIT               80U                                /**< 摩擦轮安全温度上限，单位摄氏度。 */
#define SHOOT_ECD_RANGE_DEFAULT                   ROBOT_SHOOT_ECD_RANGE              /**< Controller 默认 trigger 编码范围。 */
#define SHOOT_ECD_HALF_DEFAULT                    ROBOT_SHOOT_ECD_HALF_RANGE         /**< Controller 默认 trigger 半圈阈值。 */
#define SHOOT_ECD_TO_RAD_DEFAULT                  ROBOT_SHOOT_ECD_TO_ANGLE           /**< Controller 默认 trigger 编码到 rad。 */
#define SHOOT_RPM_TO_RAD_DEFAULT                  ROBOT_SHOOT_RPM_TO_SPEED           /**< Controller 默认 trigger rpm 到 rad/s。 */
#define SHOOT_FRIC_RPM_TO_SPEED_DEFAULT           ROBOT_SHOOT_FRIC_RPM_TO_SPEED      /**< Controller 默认摩擦轮 rpm 到控制速度换算。 */
#define SHOOT_FRIC_SPEED_DEFAULT                  ROBOT_SHOOT_FRIC_UP_SPEED          /**< Controller 默认摩擦轮目标速度。 */
#define SHOOT_TRIGGER_STEP_DEFAULT                ROBOT_SHOOT_STEP_ANGLE             /**< Controller 默认 trigger 步进角。 */
#define SHOOT_TRIGGER_DONE_ERR_DEFAULT            ROBOT_SHOOT_TRIGGER_DONE_ERR       /**< Controller 默认 trigger 到位误差。 */
#define SHOOT_TRIGGER_SPEED_DEFAULT               ROBOT_SHOOT_TRIGGER_SPEED           /**< Controller 默认 trigger 速度。 */
#define SHOOT_BLOCK_SPEED_DEFAULT                 ROBOT_SHOOT_BLOCK_TRIGGER_SPEED     /**< Controller 默认堵转速度阈值。 */
#define SHOOT_BLOCK_TIME_DEFAULT                  ROBOT_SHOOT_BLOCK_TIME_MS           /**< Controller 默认堵转时间。 */
#define SHOOT_REVERSE_TIME_DEFAULT                ROBOT_SHOOT_REVERSE_TIME_MS         /**< Controller 默认反转时间。 */
#define SHOOT_FRIC_READY_RATIO_DEFAULT            ROBOT_SHOOT_FRIC_READY_RATIO       /**< Controller 默认摩擦轮达速比例。 */
#define SHOOT_HEAT_PER_BULLET_DEFAULT             ((float)ROBOT_SHOOT_HEAT_PER_BULLET) /**< Controller 默认单发热量。 */
#define SHOOT_HEAT_COOL_DEFAULT                   ((float)ROBOT_SHOOT_COOL)           /**< Controller 默认冷却速度。 */

/* ==========================================================================
 * Head 校准元信息
 * --------------------------------------------------------------------------
 * 保存当前 target 的身份、固件版本、IMU 控温目标生成边界和地理纬度。
 * ========================================================================== */

#define ROBOT_HEAD_SELF_ID                        0U                                 /**< 当前 target 自身 ID。 */
#define ROBOT_FIRMWARE_VERSION                    12345U                             /**< 当前固件版本号。 */
#define ROBOT_LOCATION_LATITUDE_DEG               22.0F                              /**< 当前默认地理纬度，单位 degree。 */
#define ROBOT_HEAD_IMU_TEMP_MAX                   45.0F                              /**< Head 默认 IMU 控温目标上限，单位摄氏度。 */

/* ==========================================================================
 * Observer、Thermal、UI、Debug 与 Communication
 * --------------------------------------------------------------------------
 * 当前 target 的组件级默认策略；这些宏后续会继续被 App 启动层收敛使用。
 * ========================================================================== */

#define ROBOT_OBSERVER_USE_INS                    1U                                 /**< 启用 INS 观测。 */
#define ROBOT_OBSERVER_USE_CHASSIS                1U                                 /**< 启用底盘速度观测。 */
#define ROBOT_OBSERVER_USE_BALANCE                0U                                 /**< 舵轮步兵不启用平衡腿观测。 */
#define ROBOT_THERMAL_IMU_TARGET_TEMP             45.0F                              /**< IMU 温控目标温度。 */
#define ROBOT_UI_FIGURE_MAX                       16U                                /**< UI 图元容量。 */
#define ROBOT_UI_STRING_MAX                       16U                                /**< UI 字符串容量。 */
#define ROBOT_APP_REFEREE_PERIOD_MS               20U                                /**< 裁判系统 app 基础周期，单位 ms。 */
#define ROBOT_APP_REFEREE_TIMEOUT_MS              100U                               /**< 裁判系统在线超时，单位 ms。 */
#define ROBOT_APP_REFEREE_UI_PERIOD_MS            100U                               /**< 裁判客户端 UI 发送周期，单位 ms。 */
#define ROBOT_APP_REFEREE_SHOOTER_ID              1U                                 /**< 默认读取的 17mm 发射机构编号。 */
#define ROBOT_APP_REFEREE_UI_SEND_PER_STEP        1U                                 /**< 每次 UI 周期最多发送的包数量。 */
#define ROBOT_APP_MAGAZINE_PERIOD_MS              20U                                /**< 弹舱 app 周期，单位 ms。 */
#define ROBOT_APP_MAGAZINE_STACK_SIZE             1024U                              /**< 弹舱 app 线程栈大小，单位 byte。 */
#define ROBOT_APP_MAGAZINE_PRIORITY               osPriorityNormal                   /**< 弹舱 app 线程优先级。 */
#define ROBOT_APP_CALI_PERIOD_MS                  20U                                /**< Calibration app 周期，单位 ms。 */
#define ROBOT_APP_CALI_STACK_SIZE                 2048U                              /**< Calibration app 线程栈大小，单位 byte。 */
#define ROBOT_APP_CALI_PRIORITY                   osPriorityLow                      /**< Calibration app 线程优先级。 */
#define ROBOT_APP_CALI_FLASH_ADDR                 (ROBOT_FLASH_DEVICE_SIZE - ROBOT_FLASH_DEVICE_ERASE_SIZE) /**< 校准记录 Flash 地址。 */
#define ROBOT_APP_BOARD_LINK_PERIOD_MS            5U                                 /**< BoardLink app 周期，单位 ms。 */
#define ROBOT_APP_BOARD_LINK_STACK_SIZE           1536U                              /**< BoardLink app 线程栈大小，单位 byte。 */
#define ROBOT_APP_BOARD_LINK_PRIORITY             osPriorityBelowNormal              /**< BoardLink app 线程优先级。 */
#define ROBOT_DEBUG_ENABLE                        1U                                 /**< 启用 Debug probe。 */
#define ROBOT_DEBUG_LOG_ENABLE                    1U                                 /**< 启用 Debug LOG。 */
#define ROBOT_DEBUG_PROBE_MAX                     32U                                /**< Debug probe 最大数量。 */
#define ROBOT_DEBUG_LOG_SIZE                      256U                               /**< Debug LOG 缓冲大小。 */
#define ROBOT_DEBUG_PROFILER_ENABLE               0U                                 /**< 默认关闭 profiler VOFA 输出。 */
#define ROBOT_COMM_AIM_PRIORITY                   80U                                /**< 自瞄输入默认优先级。 */
#define ROBOT_COMM_HOST_RX_BUF_SIZE               128U                               /**< Host 485 接收缓存长度。 */
#define ROBOT_COMM_BOARD_RX_BUF_SIZE              128U                               /**< 板间通讯接收缓存长度。 */
#define ROBOT_COMM_VOFA_TX_BUF_SIZE               256U                               /**< VOFA 发送缓存长度。 */
#define ROBOT_COMM_TIME_INITIATOR                 0U                                 /**< 当前 target 只响应 probe，不主动发起同步。 */
#define ROBOT_COMM_TIME_MAX_RTT_US                0U                                 /**< 时间同步最大 RTT，0 表示不限制。 */

/* ==========================================================================
 * Daemon 参数
 * --------------------------------------------------------------------------
 * 当前 target 的集中健康表和边沿事件配置；只做观测，不反向接管各控制 App 的安全钳位。
 * ========================================================================== */

#define ROBOT_APP_DAEMON_PERIOD_MS                10U                                /**< Daemon 守护周期，单位 ms。 */
#define ROBOT_APP_DAEMON_STACK_SIZE               2048U                              /**< Daemon 线程栈大小，单位 byte。 */
#define ROBOT_APP_DAEMON_PRIORITY                 osPriorityLow                      /**< Daemon 线程优先级。 */
#define ROBOT_APP_DAEMON_EVENT_MAX                32U                                /**< Daemon 边沿事件 ring 容量。 */
#define ROBOT_APP_DAEMON_INIT_DELAY_MS            57U                                /**< Daemon 启动稳定等待时间，单位 ms。 */
#define ROBOT_APP_DAEMON_USE_TFMINI               0U                                 /**< 当前 target 不强制启用 TFmini 健康项。 */
#define ROBOT_APP_DAEMON_RC_OFFLINE_MS            30U                                /**< RC 离线超时，单位 ms。 */
#define ROBOT_APP_DAEMON_RC_ONLINE_MS             40U                                /**< RC 恢复稳定时间，单位 ms。 */
#define ROBOT_APP_DAEMON_RC_PRIORITY              15U                                /**< RC 错误优先级。 */
#define ROBOT_APP_DAEMON_REFEREE_OFFLINE_MS       100U                               /**< 裁判系统离线超时，单位 ms。 */
#define ROBOT_APP_DAEMON_REFEREE_ONLINE_MS        100U                               /**< 裁判系统恢复稳定时间，单位 ms。 */
#define ROBOT_APP_DAEMON_REFEREE_PRIORITY         5U                                 /**< 裁判系统错误优先级。 */
#define ROBOT_APP_DAEMON_COMM_OFFLINE_MS          100U                               /**< Host 通讯离线超时，单位 ms。 */
#define ROBOT_APP_DAEMON_COMM_ONLINE_MS           100U                               /**< Host 通讯恢复稳定时间，单位 ms。 */
#define ROBOT_APP_DAEMON_COMM_PRIORITY            6U                                 /**< Host 通讯错误优先级。 */
#define ROBOT_APP_DAEMON_VISION_OFFLINE_MS        100U                               /**< 自瞄输入离线超时，单位 ms。 */
#define ROBOT_APP_DAEMON_VISION_ONLINE_MS         100U                               /**< 自瞄输入恢复稳定时间，单位 ms。 */
#define ROBOT_APP_DAEMON_VISION_PRIORITY          6U                                 /**< 自瞄输入错误优先级。 */
#define ROBOT_APP_DAEMON_IMU_OFFLINE_MS           10U                                /**< IMU/INS 离线超时，单位 ms。 */
#define ROBOT_APP_DAEMON_IMU_ONLINE_MS            10U                                /**< IMU/INS 恢复稳定时间，单位 ms。 */
#define ROBOT_APP_DAEMON_IMU_PRIORITY             7U                                 /**< IMU/INS 错误优先级。 */
#define ROBOT_APP_DAEMON_GIMBAL_YAW_OFFLINE_MS    2U                                 /**< yaw 电机反馈离线超时，单位 ms。 */
#define ROBOT_APP_DAEMON_GIMBAL_YAW_ONLINE_MS     3U                                 /**< yaw 电机反馈恢复稳定时间，单位 ms。 */
#define ROBOT_APP_DAEMON_GIMBAL_YAW_PRIORITY      14U                                /**< yaw 电机反馈错误优先级。 */
#define ROBOT_APP_DAEMON_GIMBAL_PITCH_OFFLINE_MS  2U                                 /**< pitch 电机反馈离线超时，单位 ms。 */
#define ROBOT_APP_DAEMON_GIMBAL_PITCH_ONLINE_MS   3U                                 /**< pitch 电机反馈恢复稳定时间，单位 ms。 */
#define ROBOT_APP_DAEMON_GIMBAL_PITCH_PRIORITY    13U                                /**< pitch 电机反馈错误优先级。 */
#define ROBOT_APP_DAEMON_SHOOT_OFFLINE_MS         10U                                /**< 发射电机反馈离线超时，单位 ms。 */
#define ROBOT_APP_DAEMON_SHOOT_ONLINE_MS          10U                                /**< 发射电机反馈恢复稳定时间，单位 ms。 */
#define ROBOT_APP_DAEMON_SHOOT_PRIORITY           12U                                /**< 发射电机反馈错误优先级。 */
#define ROBOT_APP_DAEMON_CHASSIS_OFFLINE_MS       10U                                /**< 底盘反馈离线超时，单位 ms。 */
#define ROBOT_APP_DAEMON_CHASSIS_ONLINE_MS        10U                                /**< 底盘反馈恢复稳定时间，单位 ms。 */
#define ROBOT_APP_DAEMON_CHASSIS_PRIORITY         11U                                /**< 底盘反馈错误优先级。 */
#define ROBOT_APP_DAEMON_TFMINI_OFFLINE_MS        100U                               /**< TFmini 离线超时，单位 ms。 */
#define ROBOT_APP_DAEMON_TFMINI_ONLINE_MS         100U                               /**< TFmini 恢复稳定时间，单位 ms。 */
#define ROBOT_APP_DAEMON_TFMINI_PRIORITY          3U                                 /**< TFmini 错误优先级。 */
#define ROBOT_APP_ALARM_PERIOD_MS                 10U                                /**< Alarm 告警周期，单位 ms。 */
#define ROBOT_APP_ALARM_STACK_SIZE                1024U                              /**< Alarm 线程栈大小，单位 byte。 */
#define ROBOT_APP_ALARM_PRIORITY                  osPriorityLow                      /**< Alarm 线程优先级。 */
#define ROBOT_APP_ALARM_BEEP_OFF_MS               500U                               /**< 单次响声前静音时间，单位 ms。 */
#define ROBOT_APP_ALARM_BEEP_ON_MS                500U                               /**< 单次响声持续时间，单位 ms。 */
#define ROBOT_APP_ALARM_REPEAT_GAP_MS             1000U                              /**< 一组响声结束后的重复间隔，单位 ms。 */
#define ROBOT_APP_ALARM_FREQ_HZ                   3000U                              /**< 蜂鸣器告警频率，单位 Hz。 */
#define ROBOT_APP_ALARM_DUTY                      500U                               /**< 蜂鸣器告警占空比，单位千分比。 */
#define ROBOT_APP_ALARM_MAX_BEEP                  16U                                /**< 单组告警最大响声次数。 */

/* ==========================================================================
 * Auto Aim 参数
 * --------------------------------------------------------------------------
 * _ref 步兵自瞄控制参数，后续普通 gimbal control 或 Input glue 可直接引用。
 * ========================================================================== */

#define ROBOT_AA_KV_YAW                           4.0F                               /**< 自瞄 yaw 速度增益。 */
#define ROBOT_AA_KV_PITCH                         (-2.0F)                            /**< 自瞄 pitch 速度增益。 */
#define ROBOT_AA_KD_YAW                           0.05F                              /**< 自瞄 yaw 阻尼增益。 */
#define ROBOT_AA_KD_PITCH                         0.1F                               /**< 自瞄 pitch 阻尼增益。 */
#define ROBOT_AA_ERR_SOFT_ZONE_RAD                (5.0F * 3.14159265358979323846F / 180.0F) /**< 自瞄误差软区，单位 rad。 */
#define ROBOT_AA_D_LPF_ALPHA                      0.4F                               /**< 自瞄 D 项低通系数。 */
#define ROBOT_AA_OMEGA_MAX_YAW                    (180.0F * 3.14159265358979323846F / 180.0F) /**< yaw 最大角速度。 */
#define ROBOT_AA_OMEGA_MAX_PITCH                  (180.0F * 3.14159265358979323846F / 180.0F) /**< pitch 最大角速度。 */
#define ROBOT_AA_ACCEL_MAX_YAW                    (720.0F * 3.14159265358979323846F / 180.0F) /**< yaw 最大角加速度。 */
#define ROBOT_AA_ACCEL_MAX_PITCH                  (720.0F * 3.14159265358979323846F / 180.0F) /**< pitch 最大角加速度。 */
#define ROBOT_AA_EPS_TH_RAD                       (0.5F * 3.14159265358979323846F / 180.0F) /**< 自瞄角度到位阈值。 */
#define ROBOT_AA_EPS_OMEGA_RAD                    (3.0F * 3.14159265358979323846F / 180.0F) /**< 自瞄角速度到位阈值。 */
#define ROBOT_AA_MEDIAN_ENABLE                    1U                                  /**< 启用旧自瞄 5 点中值滤波。 */
#define ROBOT_AA_ZERO_HOLD_COUNT                  2U                                  /**< 自瞄短暂 0 值保持最近有效输出的次数。 */

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_ROBOT_INFANTRY_STEER_2025_PARAM_H */

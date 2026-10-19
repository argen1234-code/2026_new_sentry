/**
 ******************************************************************************
 * @file    motor.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-15
 * @brief   电机设备 C++ 编译期包装。
 *
 * @note    本文件只做 C++ 包装和模板绑定，运行时实现全部位于 Src/motor.c。
 * @note    C 文件应包含 Src/motor.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_MOTOR_HPP
#define PRINTK_DEVICE_MOTOR_HPP

#include "Src/motor.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只包装设备层 C API，便于 C++ 调用侧保持零成本访问。
 * ========================================================================== */
namespace Device::Motor {

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief DJI 反馈起始 ID。 */
inline constexpr uint16_t DjiFbFirst = MOTOR_DJI_FB_FIRST;

/** @brief DJI 反馈结束 ID。 */
inline constexpr uint16_t DjiFbLast = MOTOR_DJI_FB_LAST;

/** @brief MIT 反馈起始 ID。 */
inline constexpr uint16_t MitFbFirst = MOTOR_MIT_FB_FIRST;

/** @brief MIT 反馈结束 ID。 */
inline constexpr uint16_t MitFbLast = MOTOR_MIT_FB_LAST;

/** @brief DJI 低组发送 ID。 */
inline constexpr uint16_t DjiTxLow = DJI_MOTOR_GROUP_LOW_ID;

/** @brief DJI 高组发送 ID。 */
inline constexpr uint16_t DjiTxHigh = DJI_MOTOR_GROUP_HIGH_ID;

/** @brief 云台 yaw 电机反馈 ID。 */
inline constexpr uint16_t GimbalYawId = MOTOR_GIMBAL_YAW_ID;

/** @brief 云台 pitch 电机反馈 ID。 */
inline constexpr uint16_t GimbalPitchId = MOTOR_GIMBAL_PITCH_ID;

/** @brief 拨弹电机反馈 ID。 */
inline constexpr uint16_t TriggerId = MOTOR_TRIGGER_ID;

/** @brief 摩擦轮 0 反馈 ID。 */
inline constexpr uint16_t Fric0Id = MOTOR_FRIC_0_ID;

/** @brief 摩擦轮 1 反馈 ID。 */
inline constexpr uint16_t Fric1Id = MOTOR_FRIC_1_ID;

/** @brief 摩擦轮 2 反馈 ID。 */
inline constexpr uint16_t Fric2Id = MOTOR_FRIC_2_ID;

/** @brief 摩擦轮 3 反馈 ID。 */
inline constexpr uint16_t Fric3Id = MOTOR_FRIC_3_ID;

/** @brief 电机设备状态。 */
using Status = motor_status_t;

/** @brief MIT 反馈快照。 */
using MitFb = motor_mit_fb_t;

/** @brief 初始化电机设备绑定。 */
inline Status init()
{
    return motor_init();
}

/** @brief 绑定 MIT 电机量程。 */
inline Status mit_bind(bsp_fdcan_id_t bus, uint16_t rx_id, const MitMotorLimit *limit)
{
    return motor_mit_bind(bus, rx_id, limit);
}

/** @brief 绑定达妙 MIT 电机量程。 */
inline Status dm_bind(bsp_fdcan_id_t bus, uint16_t rx_id, DmMotorModel model)
{
    return motor_dm_bind(bus, rx_id, model);
}

/** @brief 获取 DJI 电机反馈。 */
inline Status dji_get(bsp_fdcan_id_t bus, uint16_t rx_id, DjiMotorFb *out)
{
    return motor_dji_get(bus, rx_id, out);
}

/** @brief 获取 MIT 电机反馈。 */
inline Status mit_get(bsp_fdcan_id_t bus, uint16_t rx_id, MitFb *out)
{
    return motor_mit_get(bus, rx_id, out);
}

/** @brief 发送 DJI 四路控制帧。 */
inline Status dji_send(bsp_fdcan_id_t bus,
                       uint16_t tx_id,
                       int16_t m1,
                       int16_t m2,
                       int16_t m3,
                       int16_t m4)
{
    return motor_dji_send(bus, tx_id, m1, m2, m3, m4);
}

/** @brief 发送达妙 MIT 控制帧。 */
inline Status dm_mit(bsp_fdcan_id_t bus, uint16_t id, DmMotorModel model, const MitMotorCmd *cmd)
{
    return motor_dm_mit(bus, id, model, cmd);
}

/** @brief 发送达妙特殊命令。 */
inline Status dm_cmd(bsp_fdcan_id_t bus, uint16_t id, DmMotorCmdKind kind)
{
    return motor_dm_cmd(bus, id, kind);
}

/** @brief 发送 RS 电机特殊命令。 */
inline Status rs_cmd(bsp_fdcan_id_t bus, uint16_t id, RsMotorCmdKind kind)
{
    return motor_rs_cmd(bus, id, kind);
}

/** @brief 发送底盘四路 DJI 控制量。 */
inline Status chassis(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    return motor_chassis(m1, m2, m3, m4);
}

/** @brief 发送底盘舵向四路 DJI 控制量。 */
inline Status chassis_steer(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    return motor_chassis_steer(m1, m2, m3, m4);
}

/** @brief 发送云台 DJI 控制量。 */
inline Status gimbal(int16_t yaw, int16_t pitch, int16_t trigger, int16_t rev)
{
    return motor_gimbal(yaw, pitch, trigger, rev);
}

/** @brief 更新并发送云台 yaw/pitch 槽位。 */
inline Status gimbal_yaw_pitch(int16_t yaw, int16_t pitch, int16_t rev)
{
    return motor_gimbal_yaw_pitch(yaw, pitch, rev);
}

/** @brief 更新并发送云台 trigger 槽位。 */
inline Status gimbal_trigger(int16_t trigger)
{
    return motor_gimbal_trigger(trigger);
}

/** @brief 发送摩擦轮 DJI 控制量。 */
inline Status fric(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    return motor_fric(m1, m2, m3, m4);
}

/**
 * @brief DJI 电机编译期绑定。
 *
 * @tparam Bus FDCAN 总线标识。
 * @tparam RxId DJI 电机反馈 ID。
 */
template<bsp_fdcan_id_t Bus, uint16_t RxId>
struct DjiMotor {
    /** @brief 获取 DJI 电机反馈。 */
    static Status feedback(DjiMotorFb *out)
    {
        return Device::Motor::dji_get(Bus, RxId, out);
    }
};

/**
 * @brief MIT 电机编译期绑定。
 *
 * @tparam Bus FDCAN 总线标识。
 * @tparam RxId MIT 电机反馈 ID。
 */
template<bsp_fdcan_id_t Bus, uint16_t RxId>
struct MitMotor {
    /** @brief 绑定 MIT 通用量程。 */
    static Status bind(const MitMotorLimit *limit)
    {
        return Device::Motor::mit_bind(Bus, RxId, limit);
    }

    /** @brief 绑定达妙电机量程。 */
    static Status bind(DmMotorModel model)
    {
        return Device::Motor::dm_bind(Bus, RxId, model);
    }

    /** @brief 获取 MIT 电机反馈。 */
    static Status feedback(MitFb *out)
    {
        return Device::Motor::mit_get(Bus, RxId, out);
    }
};

/**
 * @brief DJI 四路控制组编译期绑定。
 *
 * @tparam Bus FDCAN 总线标识。
 * @tparam TxId DJI 控制帧 ID。
 */
template<bsp_fdcan_id_t Bus, uint16_t TxId>
struct DjiGroup4 {
    /** @brief 发送四路 DJI 控制量。 */
    static Status send(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
    {
        return Device::Motor::dji_send(Bus, TxId, m1, m2, m3, m4);
    }
};

} // namespace Device::Motor

#endif // PRINTK_DEVICE_MOTOR_HPP

/**
 ******************************************************************************
 * @file    imu.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   IMU 设备 C++ 包装。
 *
 * @note    本文件只做 C++ 薄包装，运行时实现全部位于 Src/imu.c 和 Src/imu_heat.c。
 * @note    C 文件应包含 Src/imu.h 或 Src/imu_heat.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_IMU_HPP
#define PRINTK_DEVICE_IMU_HPP

#include "Src/imu.h"
#include "Src/imu_heat.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只包装设备层 C API，便于 C++ 调用侧保持零成本访问。
 * ========================================================================== */
namespace Device::Imu {

/** @brief IMU 设备状态。 */
using Status = imu_status_t;

/** @brief IMU 采样值。 */
using Sample = imu_sample_t;

/** @brief IMU 标定参数。 */
using Cali = ImuCali;

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 初始化板载 IMU。 */
inline Status init(uint8_t calibrate)
{
    return imu_init(calibrate);
}

/** @brief 读取并更新板载 IMU 测量值。 */
inline Status read()
{
    return imu_read();
}

/** @brief 执行板载 IMU 在线标定。 */
inline Status calibrate()
{
    return imu_calibrate();
}

/** @brief 应用外部保存的 IMU 标定参数。 */
inline Status set_cali(const Cali &cali)
{
    return imu_set_cali(&cali);
}

/** @brief 读取当前 IMU 标定参数。 */
inline Status get_cali(Cali &out)
{
    return imu_get_cali(&out);
}

/** @brief 获取最近一次 IMU 采样值。 */
inline Status sample(Sample *out)
{
    return imu_sample(out);
}

/** @brief 获取 IMU 默认标定温度。 */
inline float cal_temp()
{
    return imu_cal_temp();
}

namespace Heat {

/** @brief 初始化 IMU 加热 PWM。 */
inline HAL_StatusTypeDef init()
{
    return imu_heat_init();
}

/** @brief 设置 IMU 加热 PWM 比较值。 */
inline HAL_StatusTypeDef compare(uint32_t value)
{
    return imu_heat_set(value);
}

/** @brief 关闭 IMU 加热。 */
inline HAL_StatusTypeDef off()
{
    return imu_heat_off();
}

/** @brief 获取最大 PWM 比较值。 */
inline uint32_t max()
{
    return imu_heat_max();
}

/** @brief 获取当前 PWM 比较值。 */
inline uint32_t compare()
{
    return imu_heat_get();
}

} // namespace Heat

} // namespace Device::Imu

#endif // PRINTK_DEVICE_IMU_HPP

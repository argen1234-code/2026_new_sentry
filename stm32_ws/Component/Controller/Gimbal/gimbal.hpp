/**
 ******************************************************************************
 * @file    gimbal.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   云台机构公共接口的 C++ 零成本包装。
 *
 * @note    本文件只做类型别名和 inline 转发，不承载运行时控制逻辑。
 * @note    C 文件应包含 gimbal.h，C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_MECHANISM_GIMBAL_HPP
#define PRINTK_MECHANISM_GIMBAL_HPP

#include "gimbal.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只包装组件 C API 和类型别名，不写运行时控制逻辑。
 * ========================================================================== */
namespace Mechanism::Gimbal {

/** @brief 云台公共上下文。 */
using Context = ::Gimbal;

/** @brief 云台公共配置。 */
using Config = ::GimbalCfg;

/** @brief 云台周期输入。 */
using Input = ::GimbalInput;

/** @brief 云台周期输出。 */
using Output = ::GimbalOutput;

/** @brief 云台校准结果。 */
using Cali = ::GimbalCali;

/** @brief 云台执行状态。 */
using Status = ::GimbalStatus;

/** @brief 云台构型。 */
enum class Type : uint32_t {
    Direct = GIMBAL_TYPE_DIRECT_YAW_PITCH,   /**< yaw/pitch 直驱构型。 */
    Double = GIMBAL_TYPE_DOUBLE_YAW_PITCH,   /**< 双 yaw/pitch 构型。 */
    Linkage = GIMBAL_TYPE_LINKAGE_YAW_PITCH, /**< yaw/pitch 联动构型。 */
};

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 当前工程默认云台构型。 */
inline constexpr Type DefaultType = static_cast<Type>(GIMBAL_TYPE);

/** @brief 初始化当前编译期构型的云台上下文。 */
inline Status init(Context &ctx, const Config &cfg, void *impl = nullptr)
{
    return gimbal_init(&ctx, &cfg, impl);
}

/** @brief 执行一次完整云台周期。 */
inline Status step(Context &ctx, const Input &input, Output &output)
{
    return gimbal_step(&ctx, &input, &output);
}

/** @brief 更新云台反馈。 */
inline Status update(Context &ctx, const Input &input)
{
    return gimbal_update(&ctx, &input);
}

/** @brief 执行云台控制计算。 */
inline Status control(Context &ctx, const Input &input)
{
    return gimbal_control(&ctx, &input);
}

/** @brief 生成云台输出。 */
inline Status output(const Context &ctx, Output &out)
{
    return gimbal_output(&ctx, &out);
}

/** @brief 同步云台控制目标。 */
inline Status sync(Context &ctx)
{
    return gimbal_sync(&ctx);
}

/** @brief 写入云台校准结果。 */
inline Status set_cali(Context &ctx, const Cali &cali)
{
    return gimbal_set_cali(&ctx, &cali);
}

/**
 * @brief 当前编译期构型便捷入口。
 */
struct Port {
    /** @brief 初始化当前编译期构型的云台上下文。 */
    static Status init(Context &ctx, const Config &cfg, void *impl = nullptr)
    {
        return Mechanism::Gimbal::init(ctx, cfg, impl);
    }

    /** @brief 执行一次完整云台周期。 */
    static Status step(Context &ctx, const Input &input, Output &out)
    {
        return Mechanism::Gimbal::step(ctx, input, out);
    }

    /** @brief 同步云台控制目标。 */
    static Status sync(Context &ctx)
    {
        return Mechanism::Gimbal::sync(ctx);
    }

    /** @brief 写入云台校准结果。 */
    static Status set_cali(Context &ctx, const Cali &cali)
    {
        return Mechanism::Gimbal::set_cali(ctx, cali);
    }
};

} // namespace Mechanism::Gimbal

#endif // PRINTK_MECHANISM_GIMBAL_HPP

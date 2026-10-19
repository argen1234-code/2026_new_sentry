/**
 ******************************************************************************
 * @file    chassis.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   底盘机构公共接口的 C++ 零成本包装。
 *
 * @note    本文件只做类型别名和 inline 转发，不承载运行时控制逻辑。
 * @note    C 文件应包含 chassis.h，C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_MECHANISM_CHASSIS_HPP
#define PRINTK_MECHANISM_CHASSIS_HPP

#include "chassis.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只包装组件 C API 和类型别名，不写运行时控制逻辑。
 * ========================================================================== */
namespace Mechanism::Chassis {

/** @brief 底盘公共上下文。 */
using Context = ::Chassis;

/** @brief 底盘公共配置。 */
using Config = ::ChassisCfg;

/** @brief 底盘周期输入。 */
using Input = ::ChassisInput;

/** @brief 底盘周期输出。 */
using Output = ::ChassisOutput;

/** @brief 底盘执行状态。 */
using Status = ::ChassisStatus;

/** @brief 底盘构型。 */
enum class Type : uint32_t {
    Mecanum = CHASSIS_TYPE_MECANUM,   /**< 麦克纳姆轮底盘。 */
    Omni = CHASSIS_TYPE_OMNI,         /**< 全向轮底盘。 */
    Steering = CHASSIS_TYPE_STEERING, /**< 舵轮底盘。 */
    Balance = CHASSIS_TYPE_BALANCE,   /**< 平衡底盘。 */
};

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 当前工程默认底盘构型。 */
inline constexpr Type DefaultType = static_cast<Type>(CHASSIS_TYPE);

/** @brief 初始化当前编译期构型的底盘上下文。 */
inline Status init(Context &ctx, const Config &cfg, void *impl = nullptr)
{
    return chassis_init(&ctx, &cfg, impl);
}

/** @brief 执行一次完整底盘周期。 */
inline Status step(Context &ctx, const Input &input, Output &output)
{
    return chassis_step(&ctx, &input, &output);
}

/** @brief 更新底盘反馈。 */
inline Status update(Context &ctx, const Input &input)
{
    return chassis_update(&ctx, &input);
}

/** @brief 执行底盘控制计算。 */
inline Status control(Context &ctx, const Input &input)
{
    return chassis_control(&ctx, &input);
}

/** @brief 生成底盘输出。 */
inline Status output(const Context &ctx, Output &out)
{
    return chassis_output(&ctx, &out);
}

/** @brief 同步底盘控制目标。 */
inline Status sync(Context &ctx)
{
    return chassis_sync(&ctx);
}

/** @brief 当前编译期构型便捷入口。 */
struct Port {
    /** @brief 初始化当前编译期构型的底盘上下文。 */
    static Status init(Context &ctx, const Config &cfg, void *impl = nullptr)
    {
        return Mechanism::Chassis::init(ctx, cfg, impl);
    }

    /** @brief 执行一次完整底盘周期。 */
    static Status step(Context &ctx, const Input &input, Output &output)
    {
        return Mechanism::Chassis::step(ctx, input, output);
    }

    /** @brief 更新底盘反馈。 */
    static Status update(Context &ctx, const Input &input)
    {
        return Mechanism::Chassis::update(ctx, input);
    }

    /** @brief 执行底盘控制计算。 */
    static Status control(Context &ctx, const Input &input)
    {
        return Mechanism::Chassis::control(ctx, input);
    }

    /** @brief 生成底盘输出。 */
    static Status output(const Context &ctx, Output &out)
    {
        return Mechanism::Chassis::output(ctx, out);
    }

    /** @brief 同步底盘控制目标。 */
    static Status sync(Context &ctx)
    {
        return Mechanism::Chassis::sync(ctx);
    }
};

} // namespace Mechanism::Chassis

#endif // PRINTK_MECHANISM_CHASSIS_HPP

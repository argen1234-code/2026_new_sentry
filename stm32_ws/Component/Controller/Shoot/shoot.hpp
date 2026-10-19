/**
 ******************************************************************************
 * @file    shoot.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   发射机构公共接口的 C++ 零成本包装。
 *
 * @note    本文件只做类型别名和 inline 转发，不承载运行时控制逻辑。
 * @note    C 文件应包含 shoot.h，C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_MECHANISM_SHOOT_HPP
#define PRINTK_MECHANISM_SHOOT_HPP

#include "shoot.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只包装组件 C API 和类型别名，不写运行时控制逻辑。
 * ========================================================================== */
namespace Mechanism::Shoot {

/** @brief 发射公共上下文。 */
using Context = ::Shoot;

/** @brief 发射公共配置。 */
using Config = ::ShootCfg;

/** @brief 发射周期输入。 */
using Input = ::ShootInput;

/** @brief 发射周期输出。 */
using Output = ::ShootOutput;

/** @brief 发射执行状态。 */
using Status = ::ShootStatus;

/** @brief 发射构型。 */
enum class Type : uint32_t {
    None = SHOOT_TYPE_NONE,     /**< 无发射机构。 */
    Mono = SHOOT_TYPE_MONO,     /**< 单组摩擦轮构型。 */
    Double = SHOOT_TYPE_DOUBLE, /**< 双组摩擦轮构型。 */
};

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 当前工程默认发射构型。 */
inline constexpr Type DefaultType = static_cast<Type>(SHOOT_TYPE);

/** @brief 初始化当前编译期构型的发射上下文。 */
inline Status init(Context &ctx, const Config &cfg, void *impl = nullptr)
{
    return shoot_init(&ctx, &cfg, impl);
}

/** @brief 执行一次完整发射周期。 */
inline Status step(Context &ctx, const Input &input, Output &output)
{
    return shoot_step(&ctx, &input, &output);
}

/** @brief 更新发射反馈。 */
inline Status update(Context &ctx, const Input &input)
{
    return shoot_update(&ctx, &input);
}

/** @brief 执行发射控制计算。 */
inline Status control(Context &ctx, const Input &input)
{
    return shoot_control(&ctx, &input);
}

/** @brief 生成发射输出。 */
inline Status output(const Context &ctx, Output &out)
{
    return shoot_output(&ctx, &out);
}

/** @brief 同步发射控制目标。 */
inline Status sync(Context &ctx)
{
    return shoot_sync(&ctx);
}

/** @brief 当前编译期构型便捷入口。 */
struct Port {
    /** @brief 初始化当前编译期构型的发射上下文。 */
    static Status init(Context &ctx, const Config &cfg, void *impl = nullptr)
    {
        return Mechanism::Shoot::init(ctx, cfg, impl);
    }

    /** @brief 执行一次完整发射周期。 */
    static Status step(Context &ctx, const Input &input, Output &out)
    {
        return Mechanism::Shoot::step(ctx, input, out);
    }

    /** @brief 同步发射控制目标。 */
    static Status sync(Context &ctx)
    {
        return Mechanism::Shoot::sync(ctx);
    }
};

} // namespace Mechanism::Shoot

#endif // PRINTK_MECHANISM_SHOOT_HPP

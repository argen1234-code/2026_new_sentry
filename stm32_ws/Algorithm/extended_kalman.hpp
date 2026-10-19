/**
 ******************************************************************************
 * @file    extended_kalman.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   扩展 Kalman 滤波器 C++ 固定维度包装。
 *
 * @note    本文件只做 C++ 零开销胶水包装，运行时实现仍位于 Src/extended_kalman.c。
 *
 * @note    模板只用于编译期固定维度和静态缓冲区生成，不使用虚函数、RTTI 或动态内存。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_ALGORITHM_EKF_HPP
#define PRINTK_ALGORITHM_EKF_HPP

#include "Src/extended_kalman.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只聚合类型别名、constexpr 和 inline 转发，不引入运行时控制逻辑。
 * ========================================================================== */
namespace Algo::Ekf {

/** @brief 扩展 Kalman 执行状态。 */
using Status = ekf_status_t;

/** @brief 扩展 Kalman 协方差更新模式。 */
using Cov = ekf_cov_t;

/** @brief 状态方程回调。 */
using StateFn = ekf_state_fn;

/** @brief 状态雅可比回调。 */
using JacFn = ekf_jac_fn;

/** @brief 测量方程回调。 */
using MeasFn = ekf_meas_fn;

/** @brief 测量雅可比回调。 */
using HJacFn = ekf_hjac_fn;

/** @brief 测量残差回调。 */
using ResFn = ekf_res_fn;

/** @brief 状态注入回调。 */
using InjectFn = ekf_inject_fn;

/** @brief 状态正常。 */
inline constexpr Status Ok = EKF_OK;

/** @brief 紧凑协方差更新模式。 */
inline constexpr Cov Compact = EKF_COV_COMPACT;

/** @brief 对称化协方差更新模式。 */
inline constexpr Cov Symmetric = EKF_COV_SYMMETRIC;

/** @brief Joseph 协方差更新模式。 */
inline constexpr Cov Joseph = EKF_COV_JOSEPH;

/**
 * @brief 扩展 Kalman 回调集合。
 */
struct Model {
    StateFn state;   /**< 非线性状态方程回调。 */
    JacFn jac;       /**< 状态方程雅可比回调。 */
    MeasFn meas;     /**< 非线性测量方程回调。 */
    HJacFn hjac;     /**< 测量方程雅可比回调。 */
    ResFn res;       /**< 测量残差回调，为空时使用默认残差。 */
    InjectFn inject; /**< 状态注入回调，为空时使用默认注入。 */
    void *state_arg; /**< 状态模型用户上下文。 */
    void *meas_arg;  /**< 测量模型用户上下文。 */
};

/**
 * @brief 固定维度扩展 Kalman 滤波器。
 *
 * @tparam X 状态向量维度。
 * @tparam Z 测量向量维度。
 * @tparam U 控制向量维度，无控制输入时为 0。
 */
template<uint16_t X, uint16_t Z, uint16_t U = 0U>
struct Filter {
    static_assert(X > 0U, "Extended Kalman state dimension must be greater than zero");
    static_assert(Z > 0U, "Extended Kalman measurement dimension must be greater than zero");

    /** @brief 状态向量维度。 */
    static constexpr uint16_t StateDim = X;

    /** @brief 测量向量维度。 */
    static constexpr uint16_t MeasureDim = Z;

    /** @brief 控制向量维度。 */
    static constexpr uint16_t ControlDim = U;

    /** @brief 工作区元素数量。 */
    static constexpr uint32_t WorkSize =
        X + Z + (3U * X * X) + (2U * X * Z) + (2U * Z * Z);

    /** @brief 状态向量 x。 */
    float32_t x[X]{};

    /** @brief 状态协方差 P。 */
    float32_t p[X * X]{};

    /** @brief 过程噪声协方差 Q。 */
    float32_t q[X * X]{};

    /** @brief 测量噪声协方差 R。 */
    float32_t r[Z * Z]{};

    /** @brief 状态雅可比矩阵 F。 */
    float32_t f[X * X]{};

    /** @brief 测量雅可比矩阵 H。 */
    float32_t h[Z * X]{};

    /** @brief 控制向量 u。 */
    float32_t u[(U > 0U) ? U : 1U]{};

    /** @brief Kalman 增益 K。 */
    float32_t k[X * Z]{};

    /** @brief 创新向量 y。 */
    float32_t y[Z]{};

    /** @brief 创新协方差 S。 */
    float32_t s[Z * Z]{};

    /** @brief C 实现上下文。 */
    ekf_t ctx{};

    /** @brief C 实现工作区视图。 */
    ekf_work_t work{};

    /** @brief 工作区原始缓冲区。 */
    float32_t work_buf[WorkSize]{};

    /**
     * @brief 初始化滤波器上下文和工作区。
     *
     * @param model 非线性模型回调集合。
     * @param cov 协方差更新模式。
     * @return 初始化状态。
     */
    Status init(const Model &model, Cov cov = Compact)
    {
        ekf_cfg_t cfg{};

        cfg.x_dim = X;
        cfg.z_dim = Z;
        cfg.u_dim = U;
        cfg.x = x;
        cfg.p = p;
        cfg.q = q;
        cfg.r = r;
        cfg.f = f;
        cfg.h = h;
        cfg.u = (U > 0U) ? u : nullptr;
        cfg.k = k;
        cfg.y = y;
        cfg.s = s;
        cfg.cov = cov;
        cfg.state = model.state;
        cfg.jac = model.jac;
        cfg.meas = model.meas;
        cfg.hjac = model.hjac;
        cfg.res = model.res;
        cfg.inject = model.inject;
        cfg.state_arg = model.state_arg;
        cfg.meas_arg = model.meas_arg;

        Status status = ekf_init(&ctx, &cfg);
        if (status != Ok)
        {
            return status;
        }

        return ekf_work_init(&work, work_buf, WorkSize, X, Z);
    }

    /** @brief 执行预测步骤。 */
    Status predict()
    {
        return ekf_predict(&ctx, &work);
    }

    /** @brief 执行校正步骤。 */
    Status correct(const float32_t *z)
    {
        return ekf_correct(&ctx, &work, z);
    }

    /** @brief 执行一次预测和校正。 */
    Status step(const float32_t *z)
    {
        return ekf_step(&ctx, &work, z);
    }

    /** @brief 获取状态向量首地址。 */
    float32_t *state()
    {
        return x;
    }

    /** @brief 获取只读状态向量首地址。 */
    const float32_t *state() const
    {
        return x;
    }

    /** @brief 获取状态向量元素引用。 */
    float32_t &state(uint16_t i)
    {
        return x[i];
    }

    /** @brief 获取状态协方差元素引用。 */
    float32_t &covariance(uint16_t row, uint16_t col)
    {
        return p[((uint32_t)row * X) + col];
    }
};

} // namespace Algo::Ekf

/**
 * @brief 声明一个固定维度扩展 Kalman 滤波器对象。
 */
#define ALG_EKF(name, x_dim, z_dim, u_dim) \
    Algo::Ekf::Filter<(x_dim), (z_dim), (u_dim)> name

#endif // PRINTK_ALGORITHM_EKF_HPP

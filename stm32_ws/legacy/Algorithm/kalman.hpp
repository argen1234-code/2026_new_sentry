/**
 ******************************************************************************
 * @file    kalman.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   线性 Kalman 滤波器 C++ 固定维度包装。
 *
 * @note    本文件只做 C++ 零开销胶水包装，运行时实现仍位于 Src/kalman.c。
 *
 * @note    模板只用于编译期固定维度和静态缓冲区生成，不使用虚函数、RTTI 或动态内存。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_ALGORITHM_KF_HPP
#define PRINTK_ALGORITHM_KF_HPP

#include "Src/kalman.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只聚合类型别名、constexpr 和 inline 转发，不引入运行时控制逻辑。
 * ========================================================================== */
namespace Algo::Kf {

/** @brief 线性 Kalman 执行状态。 */
using Status = kf_status_t;

/** @brief 线性 Kalman 协方差更新模式。 */
using Cov = kf_cov_t;

/** @brief 状态正常。 */
inline constexpr Status Ok = KF_OK;

/** @brief 紧凑协方差更新模式。 */
inline constexpr Cov Compact = KF_COV_COMPACT;

/** @brief 对称化协方差更新模式。 */
inline constexpr Cov Symmetric = KF_COV_SYMMETRIC;

/** @brief Joseph 协方差更新模式。 */
inline constexpr Cov Joseph = KF_COV_JOSEPH;

/**
 * @brief 固定维度线性 Kalman 滤波器。
 *
 * @tparam X 状态向量维度。
 * @tparam Z 测量向量维度。
 * @tparam U 控制向量维度，无控制输入时为 0。
 */
template<uint16_t X, uint16_t Z, uint16_t U = 0U>
struct Filter {
    static_assert(X > 0U, "Kalman state dimension must be greater than zero");
    static_assert(Z > 0U, "Kalman measurement dimension must be greater than zero");

    /** @brief 状态向量维度。 */
    static constexpr uint16_t StateDim = X;

    /** @brief 测量向量维度。 */
    static constexpr uint16_t MeasureDim = Z;

    /** @brief 控制向量维度。 */
    static constexpr uint16_t ControlDim = U;

    /** @brief 工作区元素数量。 */
    static constexpr uint32_t WorkSize =
        (2U * X) + (3U * X * X) + (2U * X * Z) + (2U * Z * Z);

    /** @brief 状态向量 x。 */
    float32_t x[X]{};

    /** @brief 状态协方差 P。 */
    float32_t p[X * X]{};

    /** @brief 状态转移矩阵 F。 */
    float32_t f[X * X]{};

    /** @brief 过程噪声协方差 Q。 */
    float32_t q[X * X]{};

    /** @brief 测量矩阵 H。 */
    float32_t h[Z * X]{};

    /** @brief 测量噪声协方差 R。 */
    float32_t r[Z * Z]{};

    /** @brief 控制矩阵 B。 */
    float32_t b[(U > 0U) ? (X * U) : 1U]{};

    /** @brief 控制向量 u。 */
    float32_t u[(U > 0U) ? U : 1U]{};

    /** @brief Kalman 增益 K。 */
    float32_t k[X * Z]{};

    /** @brief 创新向量 y。 */
    float32_t y[Z]{};

    /** @brief 创新协方差 S。 */
    float32_t s[Z * Z]{};

    /** @brief C 实现上下文。 */
    kf_t ctx{};

    /** @brief C 实现工作区视图。 */
    kf_work_t work{};

    /** @brief 工作区原始缓冲区。 */
    float32_t work_buf[WorkSize]{};

    /**
     * @brief 初始化滤波器上下文和工作区。
     *
     * @param cov 协方差更新模式。
     * @return 初始化状态。
     */
    Status init(Cov cov = Compact)
    {
        kf_cfg_t cfg{};

        cfg.x_dim = X;
        cfg.z_dim = Z;
        cfg.u_dim = U;
        cfg.x = x;
        cfg.p = p;
        cfg.f = f;
        cfg.q = q;
        cfg.h = h;
        cfg.r = r;
        cfg.b = (U > 0U) ? b : nullptr;
        cfg.u = (U > 0U) ? u : nullptr;
        cfg.k = k;
        cfg.y = y;
        cfg.s = s;
        cfg.cov = cov;

        Status status = kf_init(&ctx, &cfg);
        if (status != Ok)
        {
            return status;
        }

        return kf_work_init(&work, work_buf, WorkSize, X, Z);
    }

    /** @brief 执行预测步骤。 */
    Status predict()
    {
        return kf_predict(&ctx, &work);
    }

    /** @brief 执行校正步骤。 */
    Status correct(const float32_t *z)
    {
        return kf_correct(&ctx, &work, z);
    }

    /** @brief 执行一次预测和校正。 */
    Status step(const float32_t *z)
    {
        return kf_step(&ctx, &work, z);
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

} // namespace Algo::Kf

/**
 * @brief 声明一个固定维度线性 Kalman 滤波器对象。
 */
#define ALG_KF(name, x_dim, z_dim, u_dim) \
    Algo::Kf::Filter<(x_dim), (z_dim), (u_dim)> name

#endif // PRINTK_ALGORITHM_KF_HPP

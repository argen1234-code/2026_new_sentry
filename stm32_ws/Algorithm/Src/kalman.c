/**
 ******************************************************************************
 * @file    kalman.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   线性 Kalman 滤波器实现。
 *
 * @note    本模块使用 CMSIS-DSP arm_matrix_instance_f32 执行矩阵计算。
 *
 * @note    所有矩阵和临时缓冲区均由外部提供，内部不申请堆内存。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "kalman.h"

#include <stddef.h>
#include "common.h"

/* ==========================================================================
 * 参数与工作区检查
 * --------------------------------------------------------------------------
 * 线性 Kalman 所有矩阵和外部工作区都先在这里检查，后续计算默认维度已经有效。
 * ========================================================================== */
/**
 * @brief 检查线性 Kalman 上下文基础参数。
 *
 * @param kalman 线性 Kalman 滤波器上下文。
 * @return 检查状态。
 */
static kf_status_t check_kf(const kf_t *kalman)
{
    if (kalman == NULL)
    {
        return KF_ERR_ARG;
    }

    if ((kalman->x_dim == 0U) || (kalman->z_dim == 0U))
    {
        return KF_ERR_SIZE;
    }

    if ((kalman->cov != KF_COV_COMPACT) &&
        (kalman->cov != KF_COV_SYMMETRIC) &&
        (kalman->cov != KF_COV_JOSEPH))
    {
        return KF_ERR_SIZE;
    }

    if ((kalman->x.pData == NULL) || (kalman->p.pData == NULL) ||
        (kalman->f.pData == NULL) || (kalman->q.pData == NULL) ||
        (kalman->h.pData == NULL) || (kalman->r.pData == NULL) ||
        (kalman->k.pData == NULL) || (kalman->y.pData == NULL) ||
        (kalman->s.pData == NULL))
    {
        return KF_ERR_ARG;
    }

    if ((kalman->has_ctrl != 0U) &&
        ((kalman->b.pData == NULL) || (kalman->u.pData == NULL)))
    {
        return KF_ERR_ARG;
    }

    return KF_OK;
}

/**
 * @brief 检查线性 Kalman 工作区基础参数。
 *
 * @param work 线性 Kalman 工作区。
 * @param x_dim 状态维度。
 * @param z_dim 测量维度。
 * @return 检查状态。
 */
static kf_status_t check_work(const kf_work_t *work,
                                  uint16_t x_dim,
                                  uint16_t z_dim)
{
    if ((work == NULL) || (work->buf == NULL))
    {
        return KF_ERR_ARG;
    }

    if (work->len < kf_work_size(x_dim, z_dim))
    {
        return KF_ERR_SIZE;
    }

    if ((work->x1.pData == NULL) || (work->x2.pData == NULL) ||
        (work->nn1.pData == NULL) || (work->nn2.pData == NULL) ||
        (work->nn3.pData == NULL) || (work->nm1.pData == NULL) ||
        (work->mn1.pData == NULL) || (work->mm1.pData == NULL) ||
        (work->mm2.pData == NULL))
    {
        return KF_ERR_ARG;
    }

    if ((work->x1.numRows != x_dim) || (work->x1.numCols != 1U) ||
        (work->x2.numRows != x_dim) || (work->x2.numCols != 1U) ||
        (work->nn1.numRows != x_dim) || (work->nn1.numCols != x_dim) ||
        (work->nn2.numRows != x_dim) || (work->nn2.numCols != x_dim) ||
        (work->nn3.numRows != x_dim) || (work->nn3.numCols != x_dim) ||
        (work->nm1.numRows != x_dim) || (work->nm1.numCols != z_dim) ||
        (work->mn1.numRows != z_dim) || (work->mn1.numCols != x_dim) ||
        (work->mm1.numRows != z_dim) || (work->mm1.numCols != z_dim) ||
        (work->mm2.numRows != z_dim) || (work->mm2.numCols != z_dim))
    {
        return KF_ERR_SIZE;
    }

    return KF_OK;
}

/**
 * @brief 将 CMSIS-DSP 状态码转换为线性 Kalman 状态码。
 *
 * @param status CMSIS-DSP 状态码。
 * @return 线性 Kalman 状态码。
 */
static kf_status_t map_status(arm_status status)
{
    if (status == ARM_MATH_SUCCESS)
    {
        return KF_OK;
    }

    if (status == ARM_MATH_SINGULAR)
    {
        return KF_ERR_SINGULAR;
    }

    return KF_ERR_MATH;
}

/* ==========================================================================
 * 工作区布局
 * --------------------------------------------------------------------------
 * 工作区由调用方静态提供，本组负责计算需求大小并把连续 buffer 切成矩阵视图。
 * ========================================================================== */
uint32_t kf_work_size(uint16_t x_dim, uint16_t z_dim)
{
    uint32_t size;

    if ((x_dim == 0U) || (z_dim == 0U))
    {
        return 0U;
    }

    size = 0U;
    if ((common_add_size(&size, (uint32_t)x_dim) == 0U) ||
        (common_add_size(&size, (uint32_t)x_dim) == 0U) ||
        (common_add_size(&size, COMMON_MUL(x_dim, x_dim)) == 0U) ||
        (common_add_size(&size, COMMON_MUL(x_dim, x_dim)) == 0U) ||
        (common_add_size(&size, COMMON_MUL(x_dim, x_dim)) == 0U) ||
        (common_add_size(&size, COMMON_MUL(x_dim, z_dim)) == 0U) ||
        (common_add_size(&size, COMMON_MUL(z_dim, x_dim)) == 0U) ||
        (common_add_size(&size, COMMON_MUL(z_dim, z_dim)) == 0U) ||
        (common_add_size(&size, COMMON_MUL(z_dim, z_dim)) == 0U))
    {
        return 0U;
    }

    return size;
}

kf_status_t kf_init(kf_t *kalman, const kf_cfg_t *cfg)
{
    if ((kalman == NULL) || (cfg == NULL))
    {
        return KF_ERR_ARG;
    }

    if ((cfg->x_dim == 0U) || (cfg->z_dim == 0U))
    {
        return KF_ERR_SIZE;
    }

    if ((cfg->cov != KF_COV_COMPACT) &&
        (cfg->cov != KF_COV_SYMMETRIC) &&
        (cfg->cov != KF_COV_JOSEPH))
    {
        return KF_ERR_SIZE;
    }

    if ((cfg->x == NULL) || (cfg->p == NULL) || (cfg->f == NULL) ||
        (cfg->q == NULL) || (cfg->h == NULL) || (cfg->r == NULL) ||
        (cfg->k == NULL) || (cfg->y == NULL) || (cfg->s == NULL))
    {
        return KF_ERR_ARG;
    }

    if ((cfg->u_dim > 0U) && ((cfg->b == NULL) || (cfg->u == NULL)))
    {
        return KF_ERR_ARG;
    }

    kalman->x_dim = cfg->x_dim;
    kalman->z_dim = cfg->z_dim;
    kalman->u_dim = cfg->u_dim;
    kalman->has_ctrl = (cfg->u_dim > 0U) ? 1U : 0U;
    kalman->cov = cfg->cov;

    arm_mat_init_f32(&kalman->x, cfg->x_dim, 1U, cfg->x);
    arm_mat_init_f32(&kalman->p, cfg->x_dim, cfg->x_dim, cfg->p);
    arm_mat_init_f32(&kalman->f, cfg->x_dim, cfg->x_dim, cfg->f);
    arm_mat_init_f32(&kalman->q, cfg->x_dim, cfg->x_dim, cfg->q);
    arm_mat_init_f32(&kalman->h, cfg->z_dim, cfg->x_dim, cfg->h);
    arm_mat_init_f32(&kalman->r, cfg->z_dim, cfg->z_dim, cfg->r);
    arm_mat_init_f32(&kalman->k, cfg->x_dim, cfg->z_dim, cfg->k);
    arm_mat_init_f32(&kalman->y, cfg->z_dim, 1U, cfg->y);
    arm_mat_init_f32(&kalman->s, cfg->z_dim, cfg->z_dim, cfg->s);

    if (kalman->has_ctrl != 0U)
    {
        arm_mat_init_f32(&kalman->b, cfg->x_dim, cfg->u_dim, cfg->b);
        arm_mat_init_f32(&kalman->u, cfg->u_dim, 1U, cfg->u);
    }
    else
    {
        arm_mat_init_f32(&kalman->b, cfg->x_dim, 0U, NULL);
        arm_mat_init_f32(&kalman->u, 0U, 1U, NULL);
    }

    return KF_OK;
}

kf_status_t kf_work_init(kf_work_t *work,
                                 float32_t *buf,
                                 uint32_t len,
                                 uint16_t x_dim,
                                 uint16_t z_dim)
{
    uint32_t need;
    uint32_t off;

    if ((work == NULL) || (buf == NULL))
    {
        return KF_ERR_ARG;
    }

    need = kf_work_size(x_dim, z_dim);
    if ((need == 0U) || (len < need))
    {
        return KF_ERR_SIZE;
    }

    work->buf = buf;
    work->len = len;
    off = 0U;

    arm_mat_init_f32(&work->x1, x_dim, 1U, &buf[off]);
    off += (uint32_t)x_dim;
    arm_mat_init_f32(&work->x2, x_dim, 1U, &buf[off]);
    off += (uint32_t)x_dim;
    arm_mat_init_f32(&work->nn1, x_dim, x_dim, &buf[off]);
    off += COMMON_MUL(x_dim, x_dim);
    arm_mat_init_f32(&work->nn2, x_dim, x_dim, &buf[off]);
    off += COMMON_MUL(x_dim, x_dim);
    arm_mat_init_f32(&work->nn3, x_dim, x_dim, &buf[off]);
    off += COMMON_MUL(x_dim, x_dim);
    arm_mat_init_f32(&work->nm1, x_dim, z_dim, &buf[off]);
    off += COMMON_MUL(x_dim, z_dim);
    arm_mat_init_f32(&work->mn1, z_dim, x_dim, &buf[off]);
    off += COMMON_MUL(z_dim, x_dim);
    arm_mat_init_f32(&work->mm1, z_dim, z_dim, &buf[off]);
    off += COMMON_MUL(z_dim, z_dim);
    arm_mat_init_f32(&work->mm2, z_dim, z_dim, &buf[off]);

    return KF_OK;
}

/* ==========================================================================
 * Predict 步骤
 * --------------------------------------------------------------------------
 * 预测阶段只推进状态和协方差，不读取任何业务层全局状态。
 * ========================================================================== */
kf_status_t kf_predict(kf_t *kalman, kf_work_t *work)
{
    kf_status_t check;
    arm_status status;
    uint32_t x_len;
    uint32_t p_len;

    check = check_kf(kalman);
    if (check != KF_OK)
    {
        return check;
    }

    check = check_work(work, kalman->x_dim, kalman->z_dim);
    if (check != KF_OK)
    {
        return check;
    }

    x_len = (uint32_t)kalman->x_dim;
    p_len = COMMON_MUL(kalman->x_dim, kalman->x_dim);

    status = arm_mat_mult_f32(&kalman->f, &kalman->x, &work->x1);
    if (!common_arm_ok(status))
    {
        return map_status(status);
    }

    if (kalman->has_ctrl != 0U)
    {
        status = arm_mat_mult_f32(&kalman->b, &kalman->u, &work->x2);
        if (!common_arm_ok(status))
        {
            return map_status(status);
        }
        common_add_to(work->x1.pData, work->x2.pData, x_len);
    }

    status = arm_mat_mult_f32(&kalman->f, &kalman->p, &work->nn1);
    if (!common_arm_ok(status))
    {
        return map_status(status);
    }

    status = arm_mat_trans_f32(&kalman->f, &work->nn2);
    if (!common_arm_ok(status))
    {
        return map_status(status);
    }

    /*
     * P 先写入工作区，待所有矩阵计算成功后再提交，避免失败时留下半更新状态。
     */
    status = arm_mat_mult_f32(&work->nn1, &work->nn2, &work->nn3);
    if (!common_arm_ok(status))
    {
        return map_status(status);
    }

    common_add_to(work->nn3.pData, kalman->q.pData, p_len);
    common_copy(kalman->x.pData, work->x1.pData, x_len);
    common_copy(kalman->p.pData, work->nn3.pData, p_len);

    return KF_OK;
}

/* ==========================================================================
 * Correct 步骤
 * --------------------------------------------------------------------------
 * 校正阶段使用当前测量更新状态，奇异矩阵和 CMSIS-DSP 错误会返回明确状态。
 * ========================================================================== */
kf_status_t kf_correct(kf_t *kalman,
                               kf_work_t *work,
                               const float32_t *z)
{
    kf_status_t check;
    arm_status status;
    uint32_t x_len;
    uint32_t z_len;
    uint32_t p_len;
    uint32_t s_len;

    if (z == NULL)
    {
        return KF_ERR_ARG;
    }

    check = check_kf(kalman);
    if (check != KF_OK)
    {
        return check;
    }

    check = check_work(work, kalman->x_dim, kalman->z_dim);
    if (check != KF_OK)
    {
        return check;
    }

    x_len = (uint32_t)kalman->x_dim;
    z_len = (uint32_t)kalman->z_dim;
    p_len = COMMON_MUL(kalman->x_dim, kalman->x_dim);
    s_len = COMMON_MUL(kalman->z_dim, kalman->z_dim);

    status = arm_mat_mult_f32(&kalman->h, &kalman->x, &kalman->y);
    if (!common_arm_ok(status))
    {
        return map_status(status);
    }
    common_residual(kalman->y.pData, z, kalman->y.pData, z_len);

    status = arm_mat_mult_f32(&kalman->h, &kalman->p, &work->mn1);
    if (!common_arm_ok(status))
    {
        return map_status(status);
    }

    status = arm_mat_trans_f32(&kalman->h, &work->nm1);
    if (!common_arm_ok(status))
    {
        return map_status(status);
    }

    status = arm_mat_mult_f32(&work->mn1, &work->nm1, &kalman->s);
    if (!common_arm_ok(status))
    {
        return map_status(status);
    }
    common_add_to(kalman->s.pData, kalman->r.pData, s_len);

    if (kalman->z_dim == 1U)
    {
        if (kalman->s.pData[0] == 0.0F)
        {
            return KF_ERR_SINGULAR;
        }
        work->mm1.pData[0] = 1.0F / kalman->s.pData[0];
    }
    else
    {
        /*
         * CMSIS-DSP 求逆会修改输入矩阵，S 需要保留为外部可读的创新协方差。
         */
        common_copy(work->mm2.pData, kalman->s.pData, s_len);
        status = arm_mat_inverse_f32(&work->mm2, &work->mm1);
        if (!common_arm_ok(status))
        {
            return map_status(status);
        }
    }

    status = arm_mat_mult_f32(&kalman->p, &work->nm1, &kalman->k);
    if (!common_arm_ok(status))
    {
        return map_status(status);
    }

    status = arm_mat_mult_f32(&kalman->k, &work->mm1, &work->nm1);
    if (!common_arm_ok(status))
    {
        return map_status(status);
    }
    common_copy(kalman->k.pData, work->nm1.pData, COMMON_MUL(kalman->x_dim, kalman->z_dim));

    status = arm_mat_mult_f32(&kalman->k, &kalman->y, &work->x1);
    if (!common_arm_ok(status))
    {
        return map_status(status);
    }

    if (kalman->cov == KF_COV_JOSEPH)
    {
        status = arm_mat_mult_f32(&kalman->k, &kalman->h, &work->nn1);
        if (!common_arm_ok(status))
        {
            return map_status(status);
        }

        common_identity(&work->nn2);
        common_sub_from(work->nn2.pData, work->nn1.pData, p_len);

        status = arm_mat_mult_f32(&work->nn2, &kalman->p, &work->nn3);
        if (!common_arm_ok(status))
        {
            return map_status(status);
        }

        status = arm_mat_trans_f32(&work->nn2, &work->nn1);
        if (!common_arm_ok(status))
        {
            return map_status(status);
        }

        status = arm_mat_mult_f32(&work->nn3, &work->nn1, &work->nn2);
        if (!common_arm_ok(status))
        {
            return map_status(status);
        }

        status = arm_mat_mult_f32(&kalman->k, &kalman->r, &work->nm1);
        if (!common_arm_ok(status))
        {
            return map_status(status);
        }

        status = arm_mat_trans_f32(&kalman->k, &work->mn1);
        if (!common_arm_ok(status))
        {
            return map_status(status);
        }

        status = arm_mat_mult_f32(&work->nm1, &work->mn1, &work->nn3);
        if (!common_arm_ok(status))
        {
            return map_status(status);
        }

        common_add_to(work->nn2.pData, work->nn3.pData, p_len);
        common_symmetrize(&work->nn2);
    }
    else
    {
        /*
         * 紧凑形式复用前面已得到的 H*P，减少一次矩阵乘法。
         */
        status = arm_mat_mult_f32(&kalman->k, &work->mn1, &work->nn1);
        if (!common_arm_ok(status))
        {
            return map_status(status);
        }

        common_copy(work->nn2.pData, kalman->p.pData, p_len);
        common_sub_from(work->nn2.pData, work->nn1.pData, p_len);

        if (kalman->cov == KF_COV_SYMMETRIC)
        {
            common_symmetrize(&work->nn2);
        }
    }

    common_add_to(kalman->x.pData, work->x1.pData, x_len);
    common_copy(kalman->p.pData, work->nn2.pData, p_len);

    return KF_OK;
}

/* ==========================================================================
 * Step 组合接口
 * --------------------------------------------------------------------------
 * 组合接口按 predict 再 correct 的顺序执行，方便 App/Component 周期调用。
 * ========================================================================== */
kf_status_t kf_step(kf_t *kalman,
                            kf_work_t *work,
                            const float32_t *z)
{
    kf_status_t status;

    status = kf_predict(kalman, work);
    if (status != KF_OK)
    {
        return status;
    }

    return kf_correct(kalman, work, z);
}

const char *kf_status_str(kf_status_t status)
{
    switch (status)
    {
    case KF_OK:
        return "KF_OK";
    case KF_ERR_ARG:
        return "KF_ERR_ARG";
    case KF_ERR_SIZE:
        return "KF_ERR_SIZE";
    case KF_ERR_MATH:
        return "KF_ERR_MATH";
    case KF_ERR_SINGULAR:
        return "KF_ERR_SINGULAR";
    default:
        return "KF_UNKNOWN";
    }
}

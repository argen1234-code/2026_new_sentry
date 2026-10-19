/**
 ******************************************************************************
 * @file    extended_kalman.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   扩展 Kalman 滤波器实现。
 *
 * @note    本模块使用 CMSIS-DSP arm_matrix_instance_f32 执行矩阵计算。
 *
 * @note    非线性模型由外部回调提供，矩阵与工作区由调用者静态分配。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "extended_kalman.h"

#include <stddef.h>
#include "common.h"

/* ==========================================================================
 * 参数与工作区检查
 * --------------------------------------------------------------------------
 * 扩展 Kalman 所有矩阵和外部工作区都先在这里检查，后续计算默认维度已经有效。
 * ========================================================================== */
/**
 * @brief 检查扩展 Kalman 上下文基础参数。
 *
 * @param kalman 扩展 Kalman 滤波器上下文。
 * @return 检查状态。
 */
static ekf_status_t check_ekf(const ekf_t *kalman)
{
    if (kalman == NULL)
    {
        return EKF_ERR_ARG;
    }

    if ((kalman->x_dim == 0U) || (kalman->z_dim == 0U))
    {
        return EKF_ERR_SIZE;
    }

    if ((kalman->cov != EKF_COV_COMPACT) &&
        (kalman->cov != EKF_COV_SYMMETRIC) &&
        (kalman->cov != EKF_COV_JOSEPH))
    {
        return EKF_ERR_SIZE;
    }

    if ((kalman->x.pData == NULL) || (kalman->p.pData == NULL) ||
        (kalman->q.pData == NULL) || (kalman->r.pData == NULL) ||
        (kalman->f.pData == NULL) || (kalman->h.pData == NULL) ||
        (kalman->k.pData == NULL) || (kalman->y.pData == NULL) ||
        (kalman->s.pData == NULL) || (kalman->state == NULL) ||
        (kalman->jac == NULL) || (kalman->meas == NULL) ||
        (kalman->hjac == NULL))
    {
        return EKF_ERR_ARG;
    }

    if ((kalman->has_ctrl != 0U) && (kalman->u.pData == NULL))
    {
        return EKF_ERR_ARG;
    }

    return EKF_OK;
}

/**
 * @brief 检查扩展 Kalman 工作区基础参数。
 *
 * @param work 扩展 Kalman 工作区。
 * @param x_dim 状态维度。
 * @param z_dim 测量维度。
 * @return 检查状态。
 */
static ekf_status_t check_work(const ekf_work_t *work,
                                           uint16_t x_dim,
                                           uint16_t z_dim)
{
    if ((work == NULL) || (work->buf == NULL))
    {
        return EKF_ERR_ARG;
    }

    if (work->len < ekf_work_size(x_dim, z_dim))
    {
        return EKF_ERR_SIZE;
    }

    if ((work->x1.pData == NULL) || (work->z1.pData == NULL) ||
        (work->nn1.pData == NULL) || (work->nn2.pData == NULL) ||
        (work->nn3.pData == NULL) || (work->nm1.pData == NULL) ||
        (work->mn1.pData == NULL) || (work->mm1.pData == NULL) ||
        (work->mm2.pData == NULL))
    {
        return EKF_ERR_ARG;
    }

    if ((work->x1.numRows != x_dim) || (work->x1.numCols != 1U) ||
        (work->z1.numRows != z_dim) || (work->z1.numCols != 1U) ||
        (work->nn1.numRows != x_dim) || (work->nn1.numCols != x_dim) ||
        (work->nn2.numRows != x_dim) || (work->nn2.numCols != x_dim) ||
        (work->nn3.numRows != x_dim) || (work->nn3.numCols != x_dim) ||
        (work->nm1.numRows != x_dim) || (work->nm1.numCols != z_dim) ||
        (work->mn1.numRows != z_dim) || (work->mn1.numCols != x_dim) ||
        (work->mm1.numRows != z_dim) || (work->mm1.numCols != z_dim) ||
        (work->mm2.numRows != z_dim) || (work->mm2.numCols != z_dim))
    {
        return EKF_ERR_SIZE;
    }

    return EKF_OK;
}

/**
 * @brief 将 CMSIS-DSP 状态码转换为扩展 Kalman 状态码。
 *
 * @param status CMSIS-DSP 状态码。
 * @return 扩展 Kalman 状态码。
 */
static ekf_status_t map_status(arm_status status)
{
    if (status == ARM_MATH_SUCCESS)
    {
        return EKF_OK;
    }

    if (status == ARM_MATH_SINGULAR)
    {
        return EKF_ERR_SINGULAR;
    }

    return EKF_ERR_MATH;
}

/* ==========================================================================
 * 工作区布局
 * --------------------------------------------------------------------------
 * 工作区由调用方静态提供，本组负责计算需求大小并把连续 buffer 切成矩阵视图。
 * ========================================================================== */
uint32_t ekf_work_size(uint16_t x_dim, uint16_t z_dim)
{
    uint32_t size;

    if ((x_dim == 0U) || (z_dim == 0U))
    {
        return 0U;
    }

    size = 0U;
    if ((common_add_size(&size, (uint32_t)x_dim) == 0U) ||
        (common_add_size(&size, (uint32_t)z_dim) == 0U) ||
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

ekf_status_t ekf_init(ekf_t *kalman,
                                              const ekf_cfg_t *cfg)
{
    if ((kalman == NULL) || (cfg == NULL))
    {
        return EKF_ERR_ARG;
    }

    if ((cfg->x_dim == 0U) || (cfg->z_dim == 0U))
    {
        return EKF_ERR_SIZE;
    }

    if ((cfg->cov != EKF_COV_COMPACT) &&
        (cfg->cov != EKF_COV_SYMMETRIC) &&
        (cfg->cov != EKF_COV_JOSEPH))
    {
        return EKF_ERR_SIZE;
    }

    if ((cfg->x == NULL) || (cfg->p == NULL) || (cfg->q == NULL) ||
        (cfg->r == NULL) || (cfg->f == NULL) || (cfg->h == NULL) ||
        (cfg->k == NULL) || (cfg->y == NULL) || (cfg->s == NULL) ||
        (cfg->state == NULL) || (cfg->jac == NULL) ||
        (cfg->meas == NULL) || (cfg->hjac == NULL))
    {
        return EKF_ERR_ARG;
    }

    if ((cfg->u_dim > 0U) && (cfg->u == NULL))
    {
        return EKF_ERR_ARG;
    }

    kalman->x_dim = cfg->x_dim;
    kalman->z_dim = cfg->z_dim;
    kalman->u_dim = cfg->u_dim;
    kalman->has_ctrl = (cfg->u_dim > 0U) ? 1U : 0U;
    kalman->cov = cfg->cov;

    arm_mat_init_f32(&kalman->x, cfg->x_dim, 1U, cfg->x);
    arm_mat_init_f32(&kalman->p, cfg->x_dim, cfg->x_dim, cfg->p);
    arm_mat_init_f32(&kalman->q, cfg->x_dim, cfg->x_dim, cfg->q);
    arm_mat_init_f32(&kalman->r, cfg->z_dim, cfg->z_dim, cfg->r);
    arm_mat_init_f32(&kalman->f, cfg->x_dim, cfg->x_dim, cfg->f);
    arm_mat_init_f32(&kalman->h, cfg->z_dim, cfg->x_dim, cfg->h);
    arm_mat_init_f32(&kalman->k, cfg->x_dim, cfg->z_dim, cfg->k);
    arm_mat_init_f32(&kalman->y, cfg->z_dim, 1U, cfg->y);
    arm_mat_init_f32(&kalman->s, cfg->z_dim, cfg->z_dim, cfg->s);

    if (kalman->has_ctrl != 0U)
    {
        arm_mat_init_f32(&kalman->u, cfg->u_dim, 1U, cfg->u);
    }
    else
    {
        arm_mat_init_f32(&kalman->u, 0U, 1U, NULL);
    }

    kalman->state = cfg->state;
    kalman->jac = cfg->jac;
    kalman->meas = cfg->meas;
    kalman->hjac = cfg->hjac;
    kalman->res = cfg->res;
    kalman->inject = cfg->inject;
    kalman->state_arg = cfg->state_arg;
    kalman->meas_arg = cfg->meas_arg;

    return EKF_OK;
}

ekf_status_t ekf_work_init(ekf_work_t *work,
                                                   float32_t *buf,
                                                   uint32_t len,
                                                   uint16_t x_dim,
                                                   uint16_t z_dim)
{
    uint32_t need;
    uint32_t off;

    if ((work == NULL) || (buf == NULL))
    {
        return EKF_ERR_ARG;
    }

    need = ekf_work_size(x_dim, z_dim);
    if ((need == 0U) || (len < need))
    {
        return EKF_ERR_SIZE;
    }

    work->buf = buf;
    work->len = len;
    off = 0U;

    arm_mat_init_f32(&work->x1, x_dim, 1U, &buf[off]);
    off += (uint32_t)x_dim;
    arm_mat_init_f32(&work->z1, z_dim, 1U, &buf[off]);
    off += (uint32_t)z_dim;
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

    return EKF_OK;
}

/* ==========================================================================
 * Predict 步骤
 * --------------------------------------------------------------------------
 * 预测阶段只推进状态和协方差，不读取任何业务层全局状态。
 * ========================================================================== */
ekf_status_t ekf_predict(ekf_t *kalman,
                                                 ekf_work_t *work)
{
    ekf_status_t status;
    arm_status arm_ret;
    const float32_t *u_data;
    uint32_t x_len;
    uint32_t p_len;

    status = check_ekf(kalman);
    if (status != EKF_OK)
    {
        return status;
    }

    status = check_work(work, kalman->x_dim, kalman->z_dim);
    if (status != EKF_OK)
    {
        return status;
    }

    x_len = (uint32_t)kalman->x_dim;
    p_len = COMMON_MUL(kalman->x_dim, kalman->x_dim);
    u_data = (kalman->has_ctrl != 0U) ? kalman->u.pData : NULL;

    /*
     * 雅可比必须在预测前状态处计算，随后状态会被非线性预测值覆盖。
     */
    status = kalman->jac(kalman->f.pData,
                         kalman->x.pData,
                         u_data,
                         kalman->x_dim,
                         kalman->u_dim,
                         kalman->state_arg);
    if (status != EKF_OK)
    {
        return status;
    }

    status = kalman->state(work->x1.pData,
                           kalman->x.pData,
                           u_data,
                           kalman->x_dim,
                           kalman->u_dim,
                           kalman->state_arg);
    if (status != EKF_OK)
    {
        return status;
    }

    arm_ret = arm_mat_mult_f32(&kalman->f, &kalman->p, &work->nn1);
    if (!common_arm_ok(arm_ret))
    {
        return map_status(arm_ret);
    }

    arm_ret = arm_mat_trans_f32(&kalman->f, &work->nn2);
    if (!common_arm_ok(arm_ret))
    {
        return map_status(arm_ret);
    }

    arm_ret = arm_mat_mult_f32(&work->nn1, &work->nn2, &work->nn3);
    if (!common_arm_ok(arm_ret))
    {
        return map_status(arm_ret);
    }

    common_add_to(work->nn3.pData, kalman->q.pData, p_len);
    common_copy(kalman->p.pData, work->nn3.pData, p_len);
    common_copy(kalman->x.pData, work->x1.pData, x_len);

    return EKF_OK;
}

/* ==========================================================================
 * Correct 步骤
 * --------------------------------------------------------------------------
 * 校正阶段使用当前测量更新状态，奇异矩阵和 CMSIS-DSP 错误会返回明确状态。
 * ========================================================================== */
ekf_status_t ekf_correct(ekf_t *kalman,
                                                 ekf_work_t *work,
                                                 const float32_t *z)
{
    ekf_status_t status;
    arm_status arm_ret;
    uint32_t x_len;
    uint32_t z_len;
    uint32_t p_len;
    uint32_t s_len;

    if (z == NULL)
    {
        return EKF_ERR_ARG;
    }

    status = check_ekf(kalman);
    if (status != EKF_OK)
    {
        return status;
    }

    status = check_work(work, kalman->x_dim, kalman->z_dim);
    if (status != EKF_OK)
    {
        return status;
    }

    x_len = (uint32_t)kalman->x_dim;
    z_len = (uint32_t)kalman->z_dim;
    p_len = COMMON_MUL(kalman->x_dim, kalman->x_dim);
    s_len = COMMON_MUL(kalman->z_dim, kalman->z_dim);

    status = kalman->meas(work->z1.pData,
                          kalman->x.pData,
                          kalman->x_dim,
                          kalman->z_dim,
                          kalman->meas_arg);
    if (status != EKF_OK)
    {
        return status;
    }

    if (kalman->res != NULL)
    {
        status = kalman->res(kalman->y.pData,
                             z,
                             work->z1.pData,
                             kalman->z_dim,
                             kalman->meas_arg);
        if (status != EKF_OK)
        {
            return status;
        }
    }
    else
    {
        common_residual(kalman->y.pData, z, work->z1.pData, z_len);
    }

    status = kalman->hjac(kalman->h.pData,
                          kalman->x.pData,
                          kalman->x_dim,
                          kalman->z_dim,
                          kalman->meas_arg);
    if (status != EKF_OK)
    {
        return status;
    }

    arm_ret = arm_mat_mult_f32(&kalman->h, &kalman->p, &work->mn1);
    if (!common_arm_ok(arm_ret))
    {
        return map_status(arm_ret);
    }

    arm_ret = arm_mat_trans_f32(&kalman->h, &work->nm1);
    if (!common_arm_ok(arm_ret))
    {
        return map_status(arm_ret);
    }

    arm_ret = arm_mat_mult_f32(&work->mn1, &work->nm1, &kalman->s);
    if (!common_arm_ok(arm_ret))
    {
        return map_status(arm_ret);
    }
    common_add_to(kalman->s.pData, kalman->r.pData, s_len);

    if (kalman->z_dim == 1U)
    {
        if (kalman->s.pData[0] == 0.0F)
        {
            return EKF_ERR_SINGULAR;
        }
        work->mm1.pData[0] = 1.0F / kalman->s.pData[0];
    }
    else
    {
        common_copy(work->mm2.pData, kalman->s.pData, s_len);
        arm_ret = arm_mat_inverse_f32(&work->mm2, &work->mm1);
        if (!common_arm_ok(arm_ret))
        {
            return map_status(arm_ret);
        }
    }

    arm_ret = arm_mat_mult_f32(&kalman->p, &work->nm1, &kalman->k);
    if (!common_arm_ok(arm_ret))
    {
        return map_status(arm_ret);
    }

    arm_ret = arm_mat_mult_f32(&kalman->k, &work->mm1, &work->nm1);
    if (!common_arm_ok(arm_ret))
    {
        return map_status(arm_ret);
    }
    common_copy(kalman->k.pData, work->nm1.pData, COMMON_MUL(kalman->x_dim, kalman->z_dim));

    arm_ret = arm_mat_mult_f32(&kalman->k, &kalman->y, &work->x1);
    if (!common_arm_ok(arm_ret))
    {
        return map_status(arm_ret);
    }

    if (kalman->cov == EKF_COV_JOSEPH)
    {
        arm_ret = arm_mat_mult_f32(&kalman->k, &kalman->h, &work->nn1);
        if (!common_arm_ok(arm_ret))
        {
            return map_status(arm_ret);
        }

        common_identity(&work->nn2);
        common_sub_from(work->nn2.pData, work->nn1.pData, p_len);

        arm_ret = arm_mat_mult_f32(&work->nn2, &kalman->p, &work->nn3);
        if (!common_arm_ok(arm_ret))
        {
            return map_status(arm_ret);
        }

        arm_ret = arm_mat_trans_f32(&work->nn2, &work->nn1);
        if (!common_arm_ok(arm_ret))
        {
            return map_status(arm_ret);
        }

        arm_ret = arm_mat_mult_f32(&work->nn3, &work->nn1, &work->nn2);
        if (!common_arm_ok(arm_ret))
        {
            return map_status(arm_ret);
        }

        arm_ret = arm_mat_mult_f32(&kalman->k, &kalman->r, &work->nm1);
        if (!common_arm_ok(arm_ret))
        {
            return map_status(arm_ret);
        }

        arm_ret = arm_mat_trans_f32(&kalman->k, &work->mn1);
        if (!common_arm_ok(arm_ret))
        {
            return map_status(arm_ret);
        }

        arm_ret = arm_mat_mult_f32(&work->nm1, &work->mn1, &work->nn3);
        if (!common_arm_ok(arm_ret))
        {
            return map_status(arm_ret);
        }

        common_add_to(work->nn2.pData, work->nn3.pData, p_len);
        common_symmetrize(&work->nn2);
    }
    else
    {
        arm_ret = arm_mat_mult_f32(&kalman->k, &work->mn1, &work->nn1);
        if (!common_arm_ok(arm_ret))
        {
            return map_status(arm_ret);
        }

        common_copy(work->nn2.pData, kalman->p.pData, p_len);
        common_sub_from(work->nn2.pData, work->nn1.pData, p_len);

        if (kalman->cov == EKF_COV_SYMMETRIC)
        {
            common_symmetrize(&work->nn2);
        }
    }

    if (kalman->inject != NULL)
    {
        status = kalman->inject(work->nn1.pData,
                                kalman->x.pData,
                                work->x1.pData,
                                kalman->x_dim,
                                kalman->state_arg);
        if (status != EKF_OK)
        {
            return status;
        }
    }
    else
    {
        common_inject(work->nn1.pData, kalman->x.pData, work->x1.pData, x_len);
    }

    common_copy(kalman->x.pData, work->nn1.pData, x_len);
    common_copy(kalman->p.pData, work->nn2.pData, p_len);

    return EKF_OK;
}

/* ==========================================================================
 * Step 组合接口
 * --------------------------------------------------------------------------
 * 组合接口按 predict 再 correct 的顺序执行，方便 App/Component 周期调用。
 * ========================================================================== */
ekf_status_t ekf_step(ekf_t *kalman,
                                              ekf_work_t *work,
                                              const float32_t *z)
{
    ekf_status_t status;

    status = ekf_predict(kalman, work);
    if (status != EKF_OK)
    {
        return status;
    }

    return ekf_correct(kalman, work, z);
}

const char *ekf_status_str(ekf_status_t status)
{
    switch (status)
    {
    case EKF_OK:
        return "EKF_OK";
    case EKF_ERR_ARG:
        return "EKF_ERR_ARG";
    case EKF_ERR_SIZE:
        return "EKF_ERR_SIZE";
    case EKF_ERR_MATH:
        return "EKF_ERR_MATH";
    case EKF_ERR_SINGULAR:
        return "EKF_ERR_SINGULAR";
    case EKF_ERR_FUNC:
        return "EKF_ERR_FUNC";
    default:
        return "EKF_UNKNOWN";
    }
}

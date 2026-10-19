/**
 ******************************************************************************
 * @file    common.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   算法层通用数学实现。
 *
 * @note    本文件实现算法层共用的矩阵、数组、向量、姿态和几何数学工具。
 *
 * @note    调用者应保证传入数组长度和矩阵维度有效；
 *          本模块对指针、数值有限性和奇异情况做明确检查。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "common.h"

#include <math.h>
#include <stddef.h>

/* ==========================================================================
 * 基础数组与矩阵工具
 * --------------------------------------------------------------------------
 * 这一组是 Algorithm 层最底层的拷贝、残差、注入和矩阵整理工具，供 Kalman、Observer 等模块复用。
 * ========================================================================== */
uint8_t common_arm_ok(arm_status status)
{
    return (status == ARM_MATH_SUCCESS) ? 1U : 0U;
}

uint8_t common_add_size(uint32_t *size, uint32_t value)
{
    if (size == NULL)
    {
        return 0U;
    }

    if (*size > (UINT32_MAX - value))
    {
        return 0U;
    }

    *size += value;
    return 1U;
}

void common_copy(float32_t *dst, const float32_t *src, uint32_t len)
{
    uint32_t i;

    if ((dst == NULL) || (src == NULL))
    {
        return;
    }

    for (i = 0U; i < len; ++i)
    {
        dst[i] = src[i];
    }
}

void common_add_to(float32_t *dst, const float32_t *src, uint32_t len)
{
    uint32_t i;

    if ((dst == NULL) || (src == NULL))
    {
        return;
    }

    for (i = 0U; i < len; ++i)
    {
        dst[i] += src[i];
    }
}

void common_sub_from(float32_t *dst, const float32_t *src, uint32_t len)
{
    uint32_t i;

    if ((dst == NULL) || (src == NULL))
    {
        return;
    }

    for (i = 0U; i < len; ++i)
    {
        dst[i] -= src[i];
    }
}

void common_identity(arm_matrix_instance_f32 *matrix)
{
    uint16_t row;
    uint16_t col;
    uint16_t dim;

    if ((matrix == NULL) || (matrix->pData == NULL) || (matrix->numRows != matrix->numCols))
    {
        return;
    }

    dim = matrix->numRows;
    for (row = 0U; row < dim; ++row)
    {
        for (col = 0U; col < dim; ++col)
        {
            matrix->pData[((uint32_t)row * dim) + col] = (row == col) ? 1.0F : 0.0F;
        }
    }
}

void common_symmetrize(arm_matrix_instance_f32 *matrix)
{
    uint16_t row;
    uint16_t col;
    uint16_t dim;
    float32_t avg;

    if ((matrix == NULL) || (matrix->pData == NULL) || (matrix->numRows != matrix->numCols))
    {
        return;
    }

    dim = matrix->numRows;
    for (row = 0U; row < dim; ++row)
    {
        for (col = (uint16_t)(row + 1U); col < dim; ++col)
        {
            avg = 0.5F * (matrix->pData[((uint32_t)row * dim) + col] +
                          matrix->pData[((uint32_t)col * dim) + row]);
            matrix->pData[((uint32_t)row * dim) + col] = avg;
            matrix->pData[((uint32_t)col * dim) + row] = avg;
        }
    }
}

void common_residual(float32_t *y,
                     const float32_t *z,
                     const float32_t *pred,
                     uint32_t len)
{
    uint32_t i;

    if ((y == NULL) || (z == NULL) || (pred == NULL))
    {
        return;
    }

    for (i = 0U; i < len; ++i)
    {
        y[i] = z[i] - pred[i];
    }
}

void common_inject(float32_t *out,
                   const float32_t *x,
                   const float32_t *dx,
                   uint32_t len)
{
    uint32_t i;

    if ((out == NULL) || (x == NULL) || (dx == NULL))
    {
        return;
    }

    for (i = 0U; i < len; ++i)
    {
        out[i] = x[i] + dx[i];
    }
}

/**
 * @brief 数值零阈值。
 *
 * 用于归一化、奇异判断和除零保护。
 */
#define COMMON_EPS (1.0e-8F)

/**
 * @brief 非法矩阵检查返回的大误差。
 */
#define COMMON_BAD_ERR (1.0e30F)

/* ==========================================================================
 * 内部数值校验
 * --------------------------------------------------------------------------
 * 这一组只做有限值、向量、旋转矩阵和四元数的局部检查，不承载业务语义。
 * ========================================================================== */
/**
 * @brief 判断浮点数是否有限。
 *
 * @param x 待检查数值。
 * @return 1 表示有限，0 表示 NaN 或 Inf。
 */
static uint8_t common_finite(float32_t x)
{
    return (uint8_t)(isfinite((double)x) != 0);
}

/**
 * @brief 检查二维向量是否可用。
 *
 * @param v 二维向量。
 * @return 1 表示可用，0 表示不可用。
 */
static uint8_t common_vec2_ok(const Vec2 *v)
{
    return (uint8_t)((v != NULL) &&
                     (common_finite(v->x) != 0U) &&
                     (common_finite(v->y) != 0U));
}

/**
 * @brief 检查三维向量是否可用。
 *
 * @param v 三维向量。
 * @return 1 表示可用，0 表示不可用。
 */
static uint8_t common_vec3_ok(const Vec3 *v)
{
    return (uint8_t)((v != NULL) &&
                     (common_finite(v->x) != 0U) &&
                     (common_finite(v->y) != 0U) &&
                     (common_finite(v->z) != 0U));
}

/**
 * @brief 检查四元数是否可用。
 *
 * @param q 四元数。
 * @return 1 表示可用，0 表示不可用。
 */
static uint8_t common_quat_ok(const Quat *q)
{
    return (uint8_t)((q != NULL) &&
                     (common_finite(q->w) != 0U) &&
                     (common_finite(q->x) != 0U) &&
                     (common_finite(q->y) != 0U) &&
                     (common_finite(q->z) != 0U));
}

/**
 * @brief 检查二维旋转矩阵是否可用。
 *
 * @param r 二维旋转矩阵。
 * @return 1 表示可用，0 表示不可用。
 */
static uint8_t common_rot2_ok(const RotMat2 *r)
{
    uint16_t i;

    if (r == NULL)
    {
        return 0U;
    }

    for (i = 0U; i < 4U; ++i)
    {
        if (common_finite(r->m[i]) == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief 检查三维旋转矩阵是否可用。
 *
 * @param r 三维旋转矩阵。
 * @return 1 表示可用，0 表示不可用。
 */
static uint8_t common_rot3_ok(const RotMat3 *r)
{
    uint16_t i;

    if (r == NULL)
    {
        return 0U;
    }

    for (i = 0U; i < 9U; ++i)
    {
        if (common_finite(r->m[i]) == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief 限制数值到指定范围。
 *
 * @param x 输入值。
 * @param min_value 最小值。
 * @param max_value 最大值。
 * @return 限幅后的值。
 */
static float32_t common_clamp(float32_t x, float32_t min_value, float32_t max_value)
{
    if (x < min_value)
    {
        return min_value;
    }

    if (x > max_value)
    {
        return max_value;
    }

    return x;
}

/**
 * @brief 原始四元数乘法，不做归一化。
 *
 * @param a 左四元数。
 * @param b 右四元数。
 * @param out 输出四元数。
 */
static void common_quat_mul_raw(const Quat *a, const Quat *b, Quat *out)
{
    out->w = (a->w * b->w) - (a->x * b->x) - (a->y * b->y) - (a->z * b->z);
    out->x = (a->w * b->x) + (a->x * b->w) + (a->y * b->z) - (a->z * b->y);
    out->y = (a->w * b->y) - (a->x * b->z) + (a->y * b->w) + (a->z * b->x);
    out->z = (a->w * b->z) + (a->x * b->y) - (a->y * b->x) + (a->z * b->w);
}

/**
 * @brief 对三维向量做交叉乘积，不检查输入。
 *
 * @param a 左向量。
 * @param b 右向量。
 * @param out 输出向量。
 */
static void common_vec3_cross_raw(const Vec3 *a, const Vec3 *b, Vec3 *out)
{
    out->x = (a->y * b->z) - (a->z * b->y);
    out->y = (a->z * b->x) - (a->x * b->z);
    out->z = (a->x * b->y) - (a->y * b->x);
}

/**
 * @brief 用 ZYX 欧拉角计算四元数，不调用 euler_wrap。
 *
 * @param e 欧拉角。
 * @param out 输出四元数。
 * @return CommonOk 表示成功。
 */
static CommonStatus common_quat_from_euler_raw(const Euler *e, Quat *out)
{
    float32_t cy;
    float32_t sy;
    float32_t cp;
    float32_t sp;
    float32_t cr;
    float32_t sr;
    Quat q;

    if ((e == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if ((common_finite(e->roll) == 0U) ||
        (common_finite(e->pitch) == 0U) ||
        (common_finite(e->yaw) == 0U))
    {
        return CommonValue;
    }

    cy = cosf(0.5F * e->yaw);
    sy = sinf(0.5F * e->yaw);
    cp = cosf(0.5F * e->pitch);
    sp = sinf(0.5F * e->pitch);
    cr = cosf(0.5F * e->roll);
    sr = sinf(0.5F * e->roll);

    q.w = (cy * cp * cr) + (sy * sp * sr);
    q.x = (cy * cp * sr) - (sy * sp * cr);
    q.y = (sy * cp * sr) + (cy * sp * cr);
    q.z = (sy * cp * cr) - (cy * sp * sr);

    return quat_normalize(&q, out);
}

/* ==========================================================================
 * 角度工具
 * --------------------------------------------------------------------------
 * 角度 wrap 与角差统一放在这里，避免不同控制器各自实现符号不一致的环绕逻辑。
 * ========================================================================== */
float32_t angle_wrap_pi(float32_t angle)
{
    float32_t wrapped;

    if (common_finite(angle) == 0U)
    {
        return 0.0F;
    }

    wrapped = fmodf(angle + COMMON_PI, COMMON_2PI);
    if (wrapped < 0.0F)
    {
        wrapped += COMMON_2PI;
    }

    return wrapped - COMMON_PI;
}

float32_t angle_wrap_2pi(float32_t angle)
{
    float32_t wrapped;

    if (common_finite(angle) == 0U)
    {
        return 0.0F;
    }

    wrapped = fmodf(angle, COMMON_2PI);
    if (wrapped < 0.0F)
    {
        wrapped += COMMON_2PI;
    }

    return wrapped;
}

float32_t angle_diff(float32_t target, float32_t current)
{
    if ((common_finite(target) == 0U) || (common_finite(current) == 0U))
    {
        return 0.0F;
    }

    return angle_wrap_pi(target - current);
}

/* ==========================================================================
 * 二维向量工具
 * --------------------------------------------------------------------------
 * 二维向量只提供点积、归一化和投影等基础操作，不引入动态分配或外部状态。
 * ========================================================================== */
float32_t vec2_dot(const Vec2 *a, const Vec2 *b)
{
    if ((common_vec2_ok(a) == 0U) || (common_vec2_ok(b) == 0U))
    {
        return 0.0F;
    }

    return (a->x * b->x) + (a->y * b->y);
}

float32_t vec2_norm(const Vec2 *v)
{
    if (common_vec2_ok(v) == 0U)
    {
        return 0.0F;
    }

    return sqrtf((v->x * v->x) + (v->y * v->y));
}

CommonStatus vec2_normalize(const Vec2 *v, Vec2 *out)
{
    float32_t norm;

    if ((v == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if (common_vec2_ok(v) == 0U)
    {
        return CommonValue;
    }

    norm = vec2_norm(v);
    if (norm <= COMMON_EPS)
    {
        return CommonSingular;
    }

    out->x = v->x / norm;
    out->y = v->y / norm;

    return CommonOk;
}

CommonStatus vec2_project(const Vec2 *v, const Vec2 *dir, Vec2 *out)
{
    float32_t den;
    float32_t scale;

    if ((v == NULL) || (dir == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if ((common_vec2_ok(v) == 0U) || (common_vec2_ok(dir) == 0U))
    {
        return CommonValue;
    }

    den = vec2_dot(dir, dir);
    if (den <= COMMON_EPS)
    {
        return CommonSingular;
    }

    scale = vec2_dot(v, dir) / den;
    out->x = scale * dir->x;
    out->y = scale * dir->y;

    return CommonOk;
}

/* ==========================================================================
 * 三维向量工具
 * --------------------------------------------------------------------------
 * 三维向量是 IMU、云台几何和弹道计算的公共语言，输入无效时返回明确状态。
 * ========================================================================== */
float32_t vec3_dot(const Vec3 *a, const Vec3 *b)
{
    if ((common_vec3_ok(a) == 0U) || (common_vec3_ok(b) == 0U))
    {
        return 0.0F;
    }

    return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

CommonStatus vec3_cross(const Vec3 *a, const Vec3 *b, Vec3 *out)
{
    if ((a == NULL) || (b == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if ((common_vec3_ok(a) == 0U) || (common_vec3_ok(b) == 0U))
    {
        return CommonValue;
    }

    common_vec3_cross_raw(a, b, out);

    return CommonOk;
}

float32_t vec3_norm(const Vec3 *v)
{
    if (common_vec3_ok(v) == 0U)
    {
        return 0.0F;
    }

    return sqrtf((v->x * v->x) + (v->y * v->y) + (v->z * v->z));
}

CommonStatus vec3_normalize(const Vec3 *v, Vec3 *out)
{
    float32_t norm;

    if ((v == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if (common_vec3_ok(v) == 0U)
    {
        return CommonValue;
    }

    norm = vec3_norm(v);
    if (norm <= COMMON_EPS)
    {
        return CommonSingular;
    }

    out->x = v->x / norm;
    out->y = v->y / norm;
    out->z = v->z / norm;

    return CommonOk;
}

CommonStatus vec3_project(const Vec3 *v, const Vec3 *dir, Vec3 *out)
{
    float32_t den;
    float32_t scale;

    if ((v == NULL) || (dir == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if ((common_vec3_ok(v) == 0U) || (common_vec3_ok(dir) == 0U))
    {
        return CommonValue;
    }

    den = vec3_dot(dir, dir);
    if (den <= COMMON_EPS)
    {
        return CommonSingular;
    }

    scale = vec3_dot(v, dir) / den;
    out->x = scale * dir->x;
    out->y = scale * dir->y;
    out->z = scale * dir->z;

    return CommonOk;
}

/* ==========================================================================
 * SO(2) 与平面旋转
 * --------------------------------------------------------------------------
 * 平面旋转矩阵和 SO(2) boxplus/boxminus 用于二维位姿和角度增量表达。
 * ========================================================================== */
CommonStatus rot2_from_angle(float32_t yaw, RotMat2 *out)
{
    float32_t c;
    float32_t s;

    if (out == NULL)
    {
        return CommonNull;
    }

    if (common_finite(yaw) == 0U)
    {
        return CommonValue;
    }

    c = cosf(yaw);
    s = sinf(yaw);

    out->m[0] = c;
    out->m[1] = -s;
    out->m[2] = s;
    out->m[3] = c;

    return CommonOk;
}

CommonStatus rot2_apply(const RotMat2 *r, const Vec2 *v, Vec2 *out)
{
    Vec2 y;

    if ((r == NULL) || (v == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if ((common_rot2_ok(r) == 0U) || (common_vec2_ok(v) == 0U))
    {
        return CommonValue;
    }

    y.x = (r->m[0] * v->x) + (r->m[1] * v->y);
    y.y = (r->m[2] * v->x) + (r->m[3] * v->y);
    *out = y;

    return CommonOk;
}

CommonStatus rot2_inv_apply(const RotMat2 *r, const Vec2 *v, Vec2 *out)
{
    Vec2 y;

    if ((r == NULL) || (v == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if ((common_rot2_ok(r) == 0U) || (common_vec2_ok(v) == 0U))
    {
        return CommonValue;
    }

    /* 旋转矩阵逆等于转置，避免显式求逆。 */
    y.x = (r->m[0] * v->x) + (r->m[2] * v->y);
    y.y = (r->m[1] * v->x) + (r->m[3] * v->y);
    *out = y;

    return CommonOk;
}

float32_t so2_boxplus(float32_t angle, float32_t delta)
{
    return angle_wrap_pi(angle + delta);
}

float32_t so2_boxminus(float32_t a, float32_t b)
{
    return angle_diff(a, b);
}

/* ==========================================================================
 * SO(3) 四元数工具
 * --------------------------------------------------------------------------
 * 四元数相关接口负责三维旋转组合、指数映射和姿态误差表达，是 Mahony 与几何工具的事实来源。
 * ========================================================================== */
CommonStatus quat_identity(Quat *out)
{
    if (out == NULL)
    {
        return CommonNull;
    }

    out->w = 1.0F;
    out->x = 0.0F;
    out->y = 0.0F;
    out->z = 0.0F;

    return CommonOk;
}

float32_t quat_norm(const Quat *q)
{
    if (common_quat_ok(q) == 0U)
    {
        return 0.0F;
    }

    return sqrtf((q->w * q->w) + (q->x * q->x) + (q->y * q->y) + (q->z * q->z));
}

CommonStatus quat_normalize(const Quat *q, Quat *out)
{
    float32_t norm;

    if ((q == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if (common_quat_ok(q) == 0U)
    {
        return CommonValue;
    }

    norm = quat_norm(q);
    if (norm <= COMMON_EPS)
    {
        return CommonSingular;
    }

    out->w = q->w / norm;
    out->x = q->x / norm;
    out->y = q->y / norm;
    out->z = q->z / norm;

    return CommonOk;
}

CommonStatus quat_conj(const Quat *q, Quat *out)
{
    if ((q == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if (common_quat_ok(q) == 0U)
    {
        return CommonValue;
    }

    out->w = q->w;
    out->x = -q->x;
    out->y = -q->y;
    out->z = -q->z;

    return CommonOk;
}

CommonStatus quat_mul(const Quat *a, const Quat *b, Quat *out)
{
    Quat raw;

    if ((a == NULL) || (b == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if ((common_quat_ok(a) == 0U) || (common_quat_ok(b) == 0U))
    {
        return CommonValue;
    }

    common_quat_mul_raw(a, b, &raw);

    return quat_normalize(&raw, out);
}

CommonStatus quat_rotate(const Quat *q, const Vec3 *v, Vec3 *out)
{
    Quat qn;
    Vec3 qv;
    Vec3 t;
    Vec3 cross2;
    Vec3 y;
    CommonStatus status;

    if ((q == NULL) || (v == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if ((common_quat_ok(q) == 0U) || (common_vec3_ok(v) == 0U))
    {
        return CommonValue;
    }

    status = quat_normalize(q, &qn);
    if (status != CommonOk)
    {
        return status;
    }

    qv.x = qn.x;
    qv.y = qn.y;
    qv.z = qn.z;

    common_vec3_cross_raw(&qv, v, &t);
    t.x *= 2.0F;
    t.y *= 2.0F;
    t.z *= 2.0F;

    common_vec3_cross_raw(&qv, &t, &cross2);

    y.x = v->x + (qn.w * t.x) + cross2.x;
    y.y = v->y + (qn.w * t.y) + cross2.y;
    y.z = v->z + (qn.w * t.z) + cross2.z;
    *out = y;

    return CommonOk;
}

CommonStatus quat_from_axis(const Vec3 *axis, float32_t angle, Quat *out)
{
    Vec3 unit;
    float32_t half;
    float32_t s;
    CommonStatus status;

    if ((axis == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if (common_finite(angle) == 0U)
    {
        return CommonValue;
    }

    if (fabsf(angle) <= COMMON_EPS)
    {
        return quat_identity(out);
    }

    if (common_vec3_ok(axis) == 0U)
    {
        return CommonValue;
    }

    status = vec3_normalize(axis, &unit);
    if (status != CommonOk)
    {
        return status;
    }

    half = 0.5F * angle;
    s = sinf(half);

    out->w = cosf(half);
    out->x = unit.x * s;
    out->y = unit.y * s;
    out->z = unit.z * s;

    return quat_normalize(out, out);
}

CommonStatus quat_from_euler(const Euler *e, Quat *out)
{
    return common_quat_from_euler_raw(e, out);
}

CommonStatus quat_to_euler(const Quat *q, Euler *out)
{
    Quat qn;
    float32_t sinr;
    float32_t cosr;
    float32_t sinp;
    float32_t siny;
    float32_t cosy;
    CommonStatus status;

    if ((q == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if (common_quat_ok(q) == 0U)
    {
        return CommonValue;
    }

    status = quat_normalize(q, &qn);
    if (status != CommonOk)
    {
        return status;
    }

    sinr = 2.0F * ((qn.w * qn.x) + (qn.y * qn.z));
    cosr = 1.0F - (2.0F * ((qn.x * qn.x) + (qn.y * qn.y)));

    sinp = 2.0F * ((qn.w * qn.y) - (qn.z * qn.x));

    siny = 2.0F * ((qn.w * qn.z) + (qn.x * qn.y));
    cosy = 1.0F - (2.0F * ((qn.y * qn.y) + (qn.z * qn.z)));

    out->roll = angle_wrap_pi(atan2f(sinr, cosr));
    out->pitch = asinf(common_clamp(sinp, -1.0F, 1.0F));
    out->yaw = angle_wrap_pi(atan2f(siny, cosy));

    return CommonOk;
}

CommonStatus so3_exp(const Vec3 *rotvec, Quat *out)
{
    float32_t theta;
    float32_t half;
    float32_t scale;

    if ((rotvec == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if (common_vec3_ok(rotvec) == 0U)
    {
        return CommonValue;
    }

    theta = vec3_norm(rotvec);
    if (theta <= COMMON_EPS)
    {
        /* 小角度时 sin(theta/2)/theta 约等于 1/2，避免 0 除。 */
        out->w = 1.0F;
        out->x = 0.5F * rotvec->x;
        out->y = 0.5F * rotvec->y;
        out->z = 0.5F * rotvec->z;
        return quat_normalize(out, out);
    }

    half = 0.5F * theta;
    scale = sinf(half) / theta;

    out->w = cosf(half);
    out->x = rotvec->x * scale;
    out->y = rotvec->y * scale;
    out->z = rotvec->z * scale;

    return quat_normalize(out, out);
}

CommonStatus so3_log(const Quat *q, Vec3 *out)
{
    Quat qn;
    float32_t v_norm;
    float32_t angle;
    float32_t scale;
    CommonStatus status;

    if ((q == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if (common_quat_ok(q) == 0U)
    {
        return CommonValue;
    }

    status = quat_normalize(q, &qn);
    if (status != CommonOk)
    {
        return status;
    }

    /* q 和 -q 表示同一姿态。取 w>=0 的代表元得到最短旋转向量。 */
    if (qn.w < 0.0F)
    {
        qn.w = -qn.w;
        qn.x = -qn.x;
        qn.y = -qn.y;
        qn.z = -qn.z;
    }

    v_norm = sqrtf((qn.x * qn.x) + (qn.y * qn.y) + (qn.z * qn.z));
    if (v_norm <= COMMON_EPS)
    {
        out->x = 2.0F * qn.x;
        out->y = 2.0F * qn.y;
        out->z = 2.0F * qn.z;
        return CommonOk;
    }

    angle = 2.0F * atan2f(v_norm, qn.w);
    scale = angle / v_norm;

    out->x = qn.x * scale;
    out->y = qn.y * scale;
    out->z = qn.z * scale;

    return CommonOk;
}

CommonStatus so3_boxplus(const Quat *q, const Vec3 *delta, Quat *out)
{
    Quat qn;
    Quat dq;
    CommonStatus status;

    if ((q == NULL) || (delta == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if ((common_quat_ok(q) == 0U) || (common_vec3_ok(delta) == 0U))
    {
        return CommonValue;
    }

    status = quat_normalize(q, &qn);
    if (status != CommonOk)
    {
        return status;
    }

    status = so3_exp(delta, &dq);
    if (status != CommonOk)
    {
        return status;
    }

    return quat_mul(&qn, &dq, out);
}

CommonStatus so3_boxminus(const Quat *target, const Quat *current, Vec3 *out)
{
    Quat target_n;
    Quat current_n;
    Quat inv_current;
    Quat err;
    CommonStatus status;

    if ((target == NULL) || (current == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if ((common_quat_ok(target) == 0U) || (common_quat_ok(current) == 0U))
    {
        return CommonValue;
    }

    status = quat_normalize(target, &target_n);
    if (status != CommonOk)
    {
        return status;
    }

    status = quat_normalize(current, &current_n);
    if (status != CommonOk)
    {
        return status;
    }

    status = quat_conj(&current_n, &inv_current);
    if (status != CommonOk)
    {
        return status;
    }

    /*
     * so3_boxplus 使用右扰动 q * Exp(delta)，因此 boxminus 必须使用
     * inverse(current) * target，保证 boxminus(boxplus(q, d), q) ≈ d。
     */
    status = quat_mul(&inv_current, &target_n, &err);
    if (status != CommonOk)
    {
        return status;
    }

    return so3_log(&err, out);
}

/* ==========================================================================
 * 三维旋转矩阵工具
 * --------------------------------------------------------------------------
 * 旋转矩阵接口用于显式坐标变换和正交性检查，调用方需要关注矩阵方向约定。
 * ========================================================================== */
CommonStatus rot3_identity(RotMat3 *out)
{
    if (out == NULL)
    {
        return CommonNull;
    }

    out->m[0] = 1.0F;
    out->m[1] = 0.0F;
    out->m[2] = 0.0F;
    out->m[3] = 0.0F;
    out->m[4] = 1.0F;
    out->m[5] = 0.0F;
    out->m[6] = 0.0F;
    out->m[7] = 0.0F;
    out->m[8] = 1.0F;

    return CommonOk;
}

CommonStatus rot3_transpose(const RotMat3 *r, RotMat3 *out)
{
    RotMat3 t;

    if ((r == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if (common_rot3_ok(r) == 0U)
    {
        return CommonValue;
    }

    t.m[0] = r->m[0];
    t.m[1] = r->m[3];
    t.m[2] = r->m[6];
    t.m[3] = r->m[1];
    t.m[4] = r->m[4];
    t.m[5] = r->m[7];
    t.m[6] = r->m[2];
    t.m[7] = r->m[5];
    t.m[8] = r->m[8];
    *out = t;

    return CommonOk;
}

CommonStatus rot3_mul(const RotMat3 *a, const RotMat3 *b, RotMat3 *out)
{
    RotMat3 y;
    uint16_t row;
    uint16_t col;
    uint16_t k;
    float32_t acc;

    if ((a == NULL) || (b == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if ((common_rot3_ok(a) == 0U) || (common_rot3_ok(b) == 0U))
    {
        return CommonValue;
    }

    for (row = 0U; row < 3U; ++row)
    {
        for (col = 0U; col < 3U; ++col)
        {
            acc = 0.0F;
            for (k = 0U; k < 3U; ++k)
            {
                acc += a->m[(row * 3U) + k] * b->m[(k * 3U) + col];
            }
            y.m[(row * 3U) + col] = acc;
        }
    }

    *out = y;

    return CommonOk;
}

CommonStatus rot3_apply(const RotMat3 *r, const Vec3 *v, Vec3 *out)
{
    Vec3 y;

    if ((r == NULL) || (v == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if ((common_rot3_ok(r) == 0U) || (common_vec3_ok(v) == 0U))
    {
        return CommonValue;
    }

    y.x = (r->m[0] * v->x) + (r->m[1] * v->y) + (r->m[2] * v->z);
    y.y = (r->m[3] * v->x) + (r->m[4] * v->y) + (r->m[5] * v->z);
    y.z = (r->m[6] * v->x) + (r->m[7] * v->y) + (r->m[8] * v->z);
    *out = y;

    return CommonOk;
}

CommonStatus rot3_inv_apply(const RotMat3 *r, const Vec3 *v, Vec3 *out)
{
    Vec3 y;

    if ((r == NULL) || (v == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if ((common_rot3_ok(r) == 0U) || (common_vec3_ok(v) == 0U))
    {
        return CommonValue;
    }

    /* 旋转矩阵逆等于转置，控制路径中避免通用矩阵求逆。 */
    y.x = (r->m[0] * v->x) + (r->m[3] * v->y) + (r->m[6] * v->z);
    y.y = (r->m[1] * v->x) + (r->m[4] * v->y) + (r->m[7] * v->z);
    y.z = (r->m[2] * v->x) + (r->m[5] * v->y) + (r->m[8] * v->z);
    *out = y;

    return CommonOk;
}

CommonStatus rot3_from_quat(const Quat *q, RotMat3 *out)
{
    Quat qn;
    float32_t xx;
    float32_t yy;
    float32_t zz;
    float32_t xy;
    float32_t xz;
    float32_t yz;
    float32_t wx;
    float32_t wy;
    float32_t wz;
    CommonStatus status;

    if ((q == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if (common_quat_ok(q) == 0U)
    {
        return CommonValue;
    }

    status = quat_normalize(q, &qn);
    if (status != CommonOk)
    {
        return status;
    }

    xx = qn.x * qn.x;
    yy = qn.y * qn.y;
    zz = qn.z * qn.z;
    xy = qn.x * qn.y;
    xz = qn.x * qn.z;
    yz = qn.y * qn.z;
    wx = qn.w * qn.x;
    wy = qn.w * qn.y;
    wz = qn.w * qn.z;

    out->m[0] = 1.0F - (2.0F * (yy + zz));
    out->m[1] = 2.0F * (xy - wz);
    out->m[2] = 2.0F * (xz + wy);
    out->m[3] = 2.0F * (xy + wz);
    out->m[4] = 1.0F - (2.0F * (xx + zz));
    out->m[5] = 2.0F * (yz - wx);
    out->m[6] = 2.0F * (xz - wy);
    out->m[7] = 2.0F * (yz + wx);
    out->m[8] = 1.0F - (2.0F * (xx + yy));

    return CommonOk;
}

CommonStatus rot3_to_quat(const RotMat3 *r, Quat *out)
{
    float32_t trace;
    float32_t s;
    Quat q;

    if ((r == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if (common_rot3_ok(r) == 0U)
    {
        return CommonValue;
    }

    trace = r->m[0] + r->m[4] + r->m[8];
    if (trace > 0.0F)
    {
        s = sqrtf(trace + 1.0F) * 2.0F;
        if (s <= COMMON_EPS)
        {
            return CommonSingular;
        }
        q.w = 0.25F * s;
        q.x = (r->m[7] - r->m[5]) / s;
        q.y = (r->m[2] - r->m[6]) / s;
        q.z = (r->m[3] - r->m[1]) / s;
    }
    else if ((r->m[0] > r->m[4]) && (r->m[0] > r->m[8]))
    {
        s = sqrtf(1.0F + r->m[0] - r->m[4] - r->m[8]) * 2.0F;
        if (s <= COMMON_EPS)
        {
            return CommonSingular;
        }
        q.w = (r->m[7] - r->m[5]) / s;
        q.x = 0.25F * s;
        q.y = (r->m[1] + r->m[3]) / s;
        q.z = (r->m[2] + r->m[6]) / s;
    }
    else if (r->m[4] > r->m[8])
    {
        s = sqrtf(1.0F + r->m[4] - r->m[0] - r->m[8]) * 2.0F;
        if (s <= COMMON_EPS)
        {
            return CommonSingular;
        }
        q.w = (r->m[2] - r->m[6]) / s;
        q.x = (r->m[1] + r->m[3]) / s;
        q.y = 0.25F * s;
        q.z = (r->m[5] + r->m[7]) / s;
    }
    else
    {
        s = sqrtf(1.0F + r->m[8] - r->m[0] - r->m[4]) * 2.0F;
        if (s <= COMMON_EPS)
        {
            return CommonSingular;
        }
        q.w = (r->m[3] - r->m[1]) / s;
        q.x = (r->m[2] + r->m[6]) / s;
        q.y = (r->m[5] + r->m[7]) / s;
        q.z = 0.25F * s;
    }

    return quat_normalize(&q, out);
}

CommonStatus rot3_from_euler(const Euler *e, RotMat3 *out)
{
    Quat q;
    CommonStatus status;

    if ((e == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    status = quat_from_euler(e, &q);
    if (status != CommonOk)
    {
        return status;
    }

    return rot3_from_quat(&q, out);
}

CommonStatus rot3_to_euler(const RotMat3 *r, Euler *out)
{
    Quat q;
    CommonStatus status;

    if ((r == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    status = rot3_to_quat(r, &q);
    if (status != CommonOk)
    {
        return status;
    }

    return quat_to_euler(&q, out);
}

float32_t rot3_det(const RotMat3 *r)
{
    float32_t det;

    if (common_rot3_ok(r) == 0U)
    {
        return 0.0F;
    }

    det = (r->m[0] * ((r->m[4] * r->m[8]) - (r->m[5] * r->m[7]))) -
          (r->m[1] * ((r->m[3] * r->m[8]) - (r->m[5] * r->m[6]))) +
          (r->m[2] * ((r->m[3] * r->m[7]) - (r->m[4] * r->m[6])));

    return (common_finite(det) != 0U) ? det : 0.0F;
}

float32_t rot3_orthogonality_error(const RotMat3 *r)
{
    uint16_t row;
    uint16_t col;
    uint16_t k;
    float32_t acc;
    float32_t expected;
    float32_t err;
    float32_t max_err;

    if (common_rot3_ok(r) == 0U)
    {
        return COMMON_BAD_ERR;
    }

    max_err = 0.0F;
    for (row = 0U; row < 3U; ++row)
    {
        for (col = 0U; col < 3U; ++col)
        {
            acc = 0.0F;
            for (k = 0U; k < 3U; ++k)
            {
                acc += r->m[(k * 3U) + row] * r->m[(k * 3U) + col];
            }

            expected = (row == col) ? 1.0F : 0.0F;
            err = fabsf(acc - expected);
            if (err > max_err)
            {
                max_err = err;
            }
        }
    }

    return (common_finite(max_err) != 0U) ? max_err : COMMON_BAD_ERR;
}

CommonStatus rot3_check(const RotMat3 *r, float32_t tol)
{
    float32_t det;
    float32_t orth_err;

    if (r == NULL)
    {
        return CommonNull;
    }

    if ((common_finite(tol) == 0U) || (tol <= 0.0F))
    {
        return CommonSize;
    }

    if (common_rot3_ok(r) == 0U)
    {
        return CommonValue;
    }

    det = rot3_det(r);
    orth_err = rot3_orthogonality_error(r);
    if ((common_finite(det) == 0U) || (common_finite(orth_err) == 0U))
    {
        return CommonValue;
    }

    if ((fabsf(det - 1.0F) > tol) || (orth_err > tol))
    {
        return CommonSingular;
    }

    return CommonOk;
}

/* ==========================================================================
 * Euler 与 yaw-pitch-roll
 * --------------------------------------------------------------------------
 * Euler 角只作为边界和调试输出语言使用，核心状态估计仍优先使用四元数或旋转矩阵。
 * ========================================================================== */
CommonStatus euler_wrap(const Euler *e, Euler *out)
{
    Euler y;

    if ((e == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if ((common_finite(e->roll) == 0U) ||
        (common_finite(e->pitch) == 0U) ||
        (common_finite(e->yaw) == 0U))
    {
        return CommonValue;
    }

    y.roll = angle_wrap_pi(e->roll);
    y.pitch = angle_wrap_pi(e->pitch);
    y.yaw = angle_wrap_pi(e->yaw);

    /*
     * ZYX 欧拉角规范化到 pitch in [-pi/2, pi/2]。
     * 超出范围时需要同步翻转 roll/yaw，保持等价姿态。
     */
    if (y.pitch > (0.5F * COMMON_PI))
    {
        y.pitch = COMMON_PI - y.pitch;
        y.roll = angle_wrap_pi(y.roll + COMMON_PI);
        y.yaw = angle_wrap_pi(y.yaw + COMMON_PI);
    }
    else if (y.pitch < (-0.5F * COMMON_PI))
    {
        y.pitch = -COMMON_PI - y.pitch;
        y.roll = angle_wrap_pi(y.roll + COMMON_PI);
        y.yaw = angle_wrap_pi(y.yaw + COMMON_PI);
    }

    *out = y;

    return CommonOk;
}

CommonStatus ypr_to_rot3(float32_t yaw, float32_t pitch, float32_t roll, RotMat3 *out)
{
    Euler e;

    if (out == NULL)
    {
        return CommonNull;
    }

    if ((common_finite(yaw) == 0U) || (common_finite(pitch) == 0U) || (common_finite(roll) == 0U))
    {
        return CommonValue;
    }

    e.roll = roll;
    e.pitch = pitch;
    e.yaw = yaw;

    return rot3_from_euler(&e, out);
}

/* ==========================================================================
 * 位姿变换
 * --------------------------------------------------------------------------
 * Pose2/Pose3 只表达刚体位姿对点的正反向变换，不包含路径规划或控制逻辑。
 * ========================================================================== */
CommonStatus pose2_apply(const Pose2 *pose, const Vec2 *point, Vec2 *out)
{
    RotMat2 r;
    Vec2 y;
    CommonStatus status;

    if ((pose == NULL) || (point == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if ((common_vec2_ok(&pose->p) == 0U) || (common_finite(pose->yaw) == 0U) || (common_vec2_ok(point) == 0U))
    {
        return CommonValue;
    }

    status = rot2_from_angle(pose->yaw, &r);
    if (status != CommonOk)
    {
        return status;
    }

    status = rot2_apply(&r, point, &y);
    if (status != CommonOk)
    {
        return status;
    }

    out->x = y.x + pose->p.x;
    out->y = y.y + pose->p.y;

    return CommonOk;
}

CommonStatus pose2_inv_apply(const Pose2 *pose, const Vec2 *point, Vec2 *out)
{
    RotMat2 r;
    Vec2 diff;
    CommonStatus status;

    if ((pose == NULL) || (point == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if ((common_vec2_ok(&pose->p) == 0U) || (common_finite(pose->yaw) == 0U) || (common_vec2_ok(point) == 0U))
    {
        return CommonValue;
    }

    status = rot2_from_angle(pose->yaw, &r);
    if (status != CommonOk)
    {
        return status;
    }

    diff.x = point->x - pose->p.x;
    diff.y = point->y - pose->p.y;

    return rot2_inv_apply(&r, &diff, out);
}

CommonStatus pose3_apply(const Pose3 *pose, const Vec3 *point, Vec3 *out)
{
    Vec3 y;
    CommonStatus status;

    if ((pose == NULL) || (point == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if ((common_vec3_ok(&pose->p) == 0U) || (common_quat_ok(&pose->q) == 0U) || (common_vec3_ok(point) == 0U))
    {
        return CommonValue;
    }

    status = quat_rotate(&pose->q, point, &y);
    if (status != CommonOk)
    {
        return status;
    }

    out->x = y.x + pose->p.x;
    out->y = y.y + pose->p.y;
    out->z = y.z + pose->p.z;

    return CommonOk;
}

CommonStatus pose3_inv_apply(const Pose3 *pose, const Vec3 *point, Vec3 *out)
{
    Quat qn;
    Quat qc;
    Vec3 diff;
    CommonStatus status;

    if ((pose == NULL) || (point == NULL) || (out == NULL))
    {
        return CommonNull;
    }

    if ((common_vec3_ok(&pose->p) == 0U) || (common_quat_ok(&pose->q) == 0U) || (common_vec3_ok(point) == 0U))
    {
        return CommonValue;
    }

    status = quat_normalize(&pose->q, &qn);
    if (status != CommonOk)
    {
        return status;
    }

    status = quat_conj(&qn, &qc);
    if (status != CommonOk)
    {
        return status;
    }

    diff.x = point->x - pose->p.x;
    diff.y = point->y - pose->p.y;
    diff.z = point->z - pose->p.z;

    return quat_rotate(&qc, &diff, out);
}

/* ==========================================================================
 * IMU 与云台几何
 * --------------------------------------------------------------------------
 * 这一组封装 IMU 坐标、机体系和云台朝向之间的纯几何关系，避免 App 层重复写三角函数。
 * ========================================================================== */
CommonStatus imu_to_body(const RotMat3 *imu_to_body, const Vec3 *imu_vec, Vec3 *body_vec)
{
    return rot3_apply(imu_to_body, imu_vec, body_vec);
}

CommonStatus body_to_imu(const RotMat3 *imu_to_body, const Vec3 *body_vec, Vec3 *imu_vec)
{
    return rot3_inv_apply(imu_to_body, body_vec, imu_vec);
}

CommonStatus gimbal_forward(float32_t yaw, float32_t pitch, Vec3 *out)
{
    RotMat3 r;
    Vec3 forward;
    CommonStatus status;

    if (out == NULL)
    {
        return CommonNull;
    }

    status = gimbal_rot(yaw, pitch, &r);
    if (status != CommonOk)
    {
        return status;
    }

    forward.x = 1.0F;
    forward.y = 0.0F;
    forward.z = 0.0F;

    return rot3_apply(&r, &forward, out);
}

CommonStatus gimbal_rot(float32_t yaw, float32_t pitch, RotMat3 *out)
{
    if (out == NULL)
    {
        return CommonNull;
    }

    if ((common_finite(yaw) == 0U) || (common_finite(pitch) == 0U))
    {
        return CommonValue;
    }

    /*
     * 通用 YPR 采用右手 Ry(pitch)。云台接口单独约定正 pitch 为抬头，
     * 因此这里取 -pitch，使局部 +X 发射轴获得正 Z 分量。
     */
    return ypr_to_rot3(yaw, -pitch, 0.0F, out);
}

/* ==========================================================================
 * 弹道辅助与状态字符串
 * --------------------------------------------------------------------------
 * 保留轻量弹道辅助和公共状态转字符串能力，复杂弹道模型放在 ballistic 模块中维护。
 * ========================================================================== */
CommonStatus ballistic_solve(float32_t horizontal,
                                float32_t vertical,
                                float32_t speed,
                                float32_t gravity,
                                BallisticSolution *out)
{
    float32_t v2;
    float32_t disc;
    float32_t root;
    float32_t tan_low;
    float32_t tan_high;
    float32_t cos_low;
    float32_t cos_high;
    float32_t eps_disc;

    if (out == NULL)
    {
        return CommonNull;
    }

    out->low_pitch = 0.0F;
    out->high_pitch = 0.0F;
    out->low_time = 0.0F;
    out->high_time = 0.0F;
    out->reachable = 0U;

    if ((common_finite(horizontal) == 0U) ||
        (common_finite(vertical) == 0U) ||
        (common_finite(speed) == 0U) ||
        (common_finite(gravity) == 0U))
    {
        return CommonValue;
    }

    if ((horizontal <= COMMON_EPS) || (speed <= COMMON_EPS) || (gravity <= COMMON_EPS))
    {
        return CommonSize;
    }

    v2 = speed * speed;
    disc = (v2 * v2) - (gravity * ((gravity * horizontal * horizontal) + (2.0F * vertical * v2)));

    if (disc < 0.0F)
    {
        eps_disc = 1.0e-6F * v2 * v2;
        if (disc >= -eps_disc)
        {
            disc = 0.0F;
        }
        else
        {
            return CommonSingular;
        }
    }

    root = sqrtf(disc);
    tan_low = (v2 - root) / (gravity * horizontal);
    tan_high = (v2 + root) / (gravity * horizontal);

    out->low_pitch = atanf(tan_low);
    out->high_pitch = atanf(tan_high);

    cos_low = cosf(out->low_pitch);
    cos_high = cosf(out->high_pitch);
    if ((fabsf(cos_low) <= COMMON_EPS) || (fabsf(cos_high) <= COMMON_EPS))
    {
        out->reachable = 0U;
        return CommonSingular;
    }

    out->low_time = horizontal / (speed * cos_low);
    out->high_time = horizontal / (speed * cos_high);
    out->reachable = 1U;

    return CommonOk;
}

const char *common_status_str(CommonStatus status)
{
    switch (status)
    {
    case CommonOk:
        return "CommonOk";
    case CommonNull:
        return "CommonNull";
    case CommonSize:
        return "CommonSize";
    case CommonValue:
        return "CommonValue";
    case CommonSingular:
        return "CommonSingular";
    default:
        return "CommonUnknown";
    }
}

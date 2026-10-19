/**
 ******************************************************************************
 * @file    common.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   算法层通用数学接口。
 *
 * @note    本文件集中放置算法层共享的矩阵、数组、向量、姿态和几何数学工具。
 *
 * @note    矩阵工具使用 CMSIS-DSP 的 arm_matrix_instance_f32 视图；
 *          几何工具使用右手坐标系、弧度制和行主序旋转矩阵。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_ALGORITHM_COMMON_H
#define PRINTK_ALGORITHM_COMMON_H

#include <stdint.h>
#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 基础数组与矩阵工具
 * --------------------------------------------------------------------------
 * 本组声明最底层的数组和矩阵辅助接口，主要服务 Kalman 工作区和公共数值代码。
 * ========================================================================== */
/**
 * @brief 计算两个维度的 uint32_t 乘积。
 *
 * 该宏统一将较小整型提升到 uint32_t 后再计算，避免 16 位维度参与乘法时发生中间溢出。
 */
#define COMMON_MUL(a, b) ((uint32_t)(a) * (uint32_t)(b))

/**
 * @brief 判断 CMSIS-DSP 返回值是否表示成功。
 *
 * @param status CMSIS-DSP 返回状态。
 * @return 非 0 表示成功，0 表示失败。
 */
uint8_t common_arm_ok(arm_status status);

/**
 * @brief 向尺寸计数器安全累加元素数。
 *
 * @param size 当前累计尺寸指针。
 * @param value 本次需要累加的元素数。
 * @return 非 0 表示累加成功，0 表示发生 uint32_t 回绕风险。
 */
uint8_t common_add_size(uint32_t *size, uint32_t value);

/**
 * @brief 复制 float32_t 数组。
 *
 * @param dst 目标数组。
 * @param src 源数组。
 * @param len 元素数量。
 */
void common_copy(float32_t *dst, const float32_t *src, uint32_t len);

/**
 * @brief 执行 dst = dst + src。
 *
 * @param dst 目标数组，同时保存累加结果。
 * @param src 源数组。
 * @param len 元素数量。
 */
void common_add_to(float32_t *dst, const float32_t *src, uint32_t len);

/**
 * @brief 执行 dst = dst - src。
 *
 * @param dst 目标数组，同时保存相减结果。
 * @param src 源数组。
 * @param len 元素数量。
 */
void common_sub_from(float32_t *dst, const float32_t *src, uint32_t len);

/**
 * @brief 将方阵设置为单位矩阵。
 *
 * @param matrix 方阵矩阵视图。
 */
void common_identity(arm_matrix_instance_f32 *matrix);

/**
 * @brief 对方阵执行原地对称化。
 *
 * @param matrix 方阵矩阵视图。
 */
void common_symmetrize(arm_matrix_instance_f32 *matrix);

/**
 * @brief 计算测量残差 y = z - pred。
 *
 * @param y 残差输出数组。
 * @param z 实际测量数组。
 * @param pred 预测测量数组。
 * @param len 元素数量。
 */
void common_residual(float32_t *y,
                     const float32_t *z,
                     const float32_t *pred,
                     uint32_t len);

/**
 * @brief 执行欧式状态注入 out = x + dx。
 *
 * @param out 更新后的状态输出数组。
 * @param x 原状态数组。
 * @param dx 状态增量数组。
 * @param len 元素数量。
 */
void common_inject(float32_t *out,
                   const float32_t *x,
                   const float32_t *dx,
                   uint32_t len);

/* ==========================================================================
 * 数学常量与公共类型
 * --------------------------------------------------------------------------
 * 常量、状态码和几何类型在这里统一定义，避免各模块重复定义单位和坐标语言。
 * ========================================================================== */
/**
 * @brief 圆周率常量。
 */
#define COMMON_PI (3.14159265358979323846F)

/**
 * @brief 2*pi 常量。
 */
#define COMMON_2PI (6.28318530717958647692F)

/**
 * @brief 角度转弧度系数。
 */
#define COMMON_DEG2RAD (0.01745329251994329577F)

/**
 * @brief 弧度转角度系数。
 */
#define COMMON_RAD2DEG (57.2957795130823208768F)

/**
 * @brief 通用数学模块执行状态。
 */
typedef enum
{
    /** 执行成功。 */
    CommonOk = 0,

    /** 输入指针为空。 */
    CommonNull = 1,

    /** 参数范围或尺寸非法。 */
    CommonSize = 2,

    /** 输入或中间结果包含 NaN/Inf。 */
    CommonValue = 3,

    /** 归一化、求解或几何计算遇到奇异情况。 */
    CommonSingular = 4
} CommonStatus;

/**
 * @brief 二维向量。
 */
typedef struct
{
    /** x 分量。 */
    float32_t x;

    /** y 分量。 */
    float32_t y;
} Vec2;

/**
 * @brief 三维向量。
 */
typedef struct
{
    /** x 分量。 */
    float32_t x;

    /** y 分量。 */
    float32_t y;

    /** z 分量。 */
    float32_t z;
} Vec3;

/**
 * @brief 单位四元数。
 *
 * 约定 q = w + x*i + y*j + z*k，用于表示主动旋转。
 */
typedef struct
{
    /** 标量部。 */
    float32_t w;

    /** x 虚部。 */
    float32_t x;

    /** y 虚部。 */
    float32_t y;

    /** z 虚部。 */
    float32_t z;
} Quat;

/**
 * @brief ZYX yaw-pitch-roll 欧拉角。
 */
typedef struct
{
    /** 绕 x 轴 roll，单位 rad。 */
    float32_t roll;

    /** 绕 y 轴 pitch，单位 rad。 */
    float32_t pitch;

    /** 绕 z 轴 yaw，单位 rad。 */
    float32_t yaw;
} Euler;

/**
 * @brief 二维旋转矩阵。
 *
 * 行主序矩阵：[ c -s; s c ]。
 */
typedef struct
{
    /** 行主序 2x2 矩阵。 */
    float32_t m[4];
} RotMat2;

/**
 * @brief 三维旋转矩阵。
 *
 * 行主序 3x3 矩阵。
 */
typedef struct
{
    /** 行主序 3x3 矩阵。 */
    float32_t m[9];
} RotMat3;

/**
 * @brief 二维刚体位姿。
 *
 * p 表示平移，yaw 表示从局部坐标系到父坐标系的旋转。
 */
typedef struct
{
    /** 平移。 */
    Vec2 p;

    /** 航向角，单位 rad。 */
    float32_t yaw;
} Pose2;

/**
 * @brief 三维刚体位姿。
 *
 * p 表示平移，q 表示从局部坐标系到父坐标系的旋转。
 */
typedef struct
{
    /** 平移。 */
    Vec3 p;

    /** 姿态四元数。 */
    Quat q;
} Pose3;

/**
 * @brief 无空气阻力弹道解。
 */
typedef struct
{
    /** 低抛弹道俯仰角，单位 rad。 */
    float32_t low_pitch;

    /** 高抛弹道俯仰角，单位 rad。 */
    float32_t high_pitch;

    /** 低抛飞行时间，单位秒。 */
    float32_t low_time;

    /** 高抛飞行时间，单位秒。 */
    float32_t high_time;

    /** 非 0 表示目标在给定速度下可达。 */
    uint8_t reachable;
} BallisticSolution;

/* ==========================================================================
 * 角度、向量与 SO(2)
 * --------------------------------------------------------------------------
 * 这一组接口处理角度环绕、二维/三维向量和二维旋转，是控制器最常用的基础几何工具。
 * ========================================================================== */
/**
 * @brief 将角度 wrap 到 [-pi, pi)。
 *
 * @param angle 输入角度，单位 rad。
 * @return wrap 后角度。
 */
float32_t angle_wrap_pi(float32_t angle);

/**
 * @brief 将角度 wrap 到 [0, 2*pi)。
 *
 * @param angle 输入角度，单位 rad。
 * @return wrap 后角度。
 */
float32_t angle_wrap_2pi(float32_t angle);

/**
 * @brief 计算目标角到当前角的最短残差。
 *
 * @param target 目标角，单位 rad。
 * @param current 当前角，单位 rad。
 * @return wrap 到 [-pi, pi) 的 target-current。
 */
float32_t angle_diff(float32_t target, float32_t current);

/**
 * @brief 二维向量点积。
 *
 * @param a 向量 a。
 * @param b 向量 b。
 * @return 点积。
 */
float32_t vec2_dot(const Vec2 *a, const Vec2 *b);

/**
 * @brief 二维向量模长。
 *
 * @param v 输入向量。
 * @return 模长；输入为空或非法时返回 0。
 */
float32_t vec2_norm(const Vec2 *v);

/**
 * @brief 归一化二维向量。
 *
 * @param v 输入向量。
 * @param out 输出单位向量。
 * @return 执行状态。
 */
CommonStatus vec2_normalize(const Vec2 *v, Vec2 *out);

/**
 * @brief 将二维向量投影到指定方向。
 *
 * @param v 输入向量。
 * @param dir 投影方向，不要求单位化。
 * @param out 投影结果。
 * @return 执行状态。
 */
CommonStatus vec2_project(const Vec2 *v, const Vec2 *dir, Vec2 *out);

/**
 * @brief 三维向量点积。
 *
 * @param a 向量 a。
 * @param b 向量 b。
 * @return 点积。
 */
float32_t vec3_dot(const Vec3 *a, const Vec3 *b);

/**
 * @brief 三维向量叉积。
 *
 * @param a 向量 a。
 * @param b 向量 b。
 * @param out 输出 a x b。
 * @return 执行状态。
 */
CommonStatus vec3_cross(const Vec3 *a, const Vec3 *b, Vec3 *out);

/**
 * @brief 三维向量模长。
 *
 * @param v 输入向量。
 * @return 模长；输入为空或非法时返回 0。
 */
float32_t vec3_norm(const Vec3 *v);

/**
 * @brief 归一化三维向量。
 *
 * @param v 输入向量。
 * @param out 输出单位向量。
 * @return 执行状态。
 */
CommonStatus vec3_normalize(const Vec3 *v, Vec3 *out);

/**
 * @brief 将三维向量投影到指定方向。
 *
 * @param v 输入向量。
 * @param dir 投影方向，不要求单位化。
 * @param out 投影结果。
 * @return 执行状态。
 */
CommonStatus vec3_project(const Vec3 *v, const Vec3 *dir, Vec3 *out);

/**
 * @brief 由角度生成二维旋转矩阵。
 *
 * @param yaw 旋转角，单位 rad。
 * @param out 输出旋转矩阵。
 * @return 执行状态。
 */
CommonStatus rot2_from_angle(float32_t yaw, RotMat2 *out);

/**
 * @brief 二维旋转作用于向量。
 *
 * @param r 旋转矩阵。
 * @param v 输入向量。
 * @param out 输出向量。
 * @return 执行状态。
 */
CommonStatus rot2_apply(const RotMat2 *r, const Vec2 *v, Vec2 *out);

/**
 * @brief 二维逆旋转作用于向量。
 *
 * @param r 旋转矩阵。
 * @param v 输入向量。
 * @param out 输出向量。
 * @return 执行状态。
 */
CommonStatus rot2_inv_apply(const RotMat2 *r, const Vec2 *v, Vec2 *out);

/**
 * @brief SO(2) boxplus。
 *
 * @param angle 当前角。
 * @param delta 增量角。
 * @return wrap 后的 angle + delta。
 */
float32_t so2_boxplus(float32_t angle, float32_t delta);

/**
 * @brief SO(2) boxminus。
 *
 * @param a 目标角。
 * @param b 当前角。
 * @return wrap 后的 a - b。
 */
float32_t so2_boxminus(float32_t a, float32_t b);

/* ==========================================================================
 * SO(3) 姿态工具
 * --------------------------------------------------------------------------
 * 四元数、旋转矩阵和 Euler 角转换统一从这里获取，调用方不要在业务层手写不同约定的姿态公式。
 * ========================================================================== */
/**
 * @brief 生成单位四元数。
 *
 * @param out 输出四元数。
 * @return 执行状态。
 */
CommonStatus quat_identity(Quat *out);

/**
 * @brief 计算四元数模长。
 *
 * @param q 输入四元数。
 * @return 模长；输入为空或非法时返回 0。
 */
float32_t quat_norm(const Quat *q);

/**
 * @brief 归一化四元数。
 *
 * @param q 输入四元数。
 * @param out 输出单位四元数。
 * @return 执行状态。
 */
CommonStatus quat_normalize(const Quat *q, Quat *out);

/**
 * @brief 四元数共轭。
 *
 * @param q 输入四元数。
 * @param out 输出共轭四元数。
 * @return 执行状态。
 */
CommonStatus quat_conj(const Quat *q, Quat *out);

/**
 * @brief 四元数乘法。
 *
 * @param a 左四元数。
 * @param b 右四元数。
 * @param out 输出 a*b。
 * @return 执行状态。
 */
CommonStatus quat_mul(const Quat *a, const Quat *b, Quat *out);

/**
 * @brief 使用四元数旋转三维向量。
 *
 * @param q 单位四元数。
 * @param v 输入向量。
 * @param out 输出向量。
 * @return 执行状态。
 */
CommonStatus quat_rotate(const Quat *q, const Vec3 *v, Vec3 *out);

/**
 * @brief 由轴角生成四元数。
 *
 * @param axis 旋转轴，不要求单位化。
 * @param angle 旋转角，单位 rad。
 * @param out 输出单位四元数。
 * @return 执行状态。
 */
CommonStatus quat_from_axis(const Vec3 *axis, float32_t angle, Quat *out);

/**
 * @brief 由 ZYX Euler 角生成四元数。
 *
 * @param e Euler 角。
 * @param out 输出单位四元数。
 * @return 执行状态。
 */
CommonStatus quat_from_euler(const Euler *e, Quat *out);

/**
 * @brief 四元数转 ZYX Euler 角。
 *
 * @param q 输入四元数。
 * @param out 输出 Euler 角。
 * @return 执行状态。
 */
CommonStatus quat_to_euler(const Quat *q, Euler *out);

/**
 * @brief SO(3) 指数映射，旋转向量转四元数。
 *
 * @param rotvec 旋转向量，方向为轴，模长为角度 rad。
 * @param out 输出单位四元数。
 * @return 执行状态。
 */
CommonStatus so3_exp(const Vec3 *rotvec, Quat *out);

/**
 * @brief SO(3) 对数映射，四元数转旋转向量。
 *
 * @param q 输入单位四元数。
 * @param out 输出旋转向量。
 * @return 执行状态。
 */
CommonStatus so3_log(const Quat *q, Vec3 *out);

/**
 * @brief SO(3) boxplus。
 *
 * out = q * Exp(delta)。
 *
 * @param q 当前姿态。
 * @param delta 小旋转向量。
 * @param out 输出姿态。
 * @return 执行状态。
 */
CommonStatus so3_boxplus(const Quat *q, const Vec3 *delta, Quat *out);

/**
 * @brief SO(3) boxminus。
 *
 * out = Log(inverse(current) * target)，与 so3_boxplus 的右扰动约定保持一致。
 *
 * @param target 目标姿态。
 * @param current 当前姿态。
 * @param out 输出旋转误差向量。
 * @return 执行状态。
 */
CommonStatus so3_boxminus(const Quat *target, const Quat *current, Vec3 *out);

/**
 * @brief 生成三维单位旋转矩阵。
 *
 * @param out 输出旋转矩阵。
 * @return 执行状态。
 */
CommonStatus rot3_identity(RotMat3 *out);

/**
 * @brief 三维旋转矩阵转置。
 *
 * @param r 输入旋转矩阵。
 * @param out 输出转置矩阵。
 * @return 执行状态。
 */
CommonStatus rot3_transpose(const RotMat3 *r, RotMat3 *out);

/**
 * @brief 三维旋转矩阵乘法。
 *
 * @param a 左矩阵。
 * @param b 右矩阵。
 * @param out 输出 a*b。
 * @return 执行状态。
 */
CommonStatus rot3_mul(const RotMat3 *a, const RotMat3 *b, RotMat3 *out);

/**
 * @brief 三维旋转作用于向量。
 *
 * @param r 旋转矩阵。
 * @param v 输入向量。
 * @param out 输出向量。
 * @return 执行状态。
 */
CommonStatus rot3_apply(const RotMat3 *r, const Vec3 *v, Vec3 *out);

/**
 * @brief 三维逆旋转作用于向量。
 *
 * @param r 旋转矩阵。
 * @param v 输入向量。
 * @param out 输出向量。
 * @return 执行状态。
 */
CommonStatus rot3_inv_apply(const RotMat3 *r, const Vec3 *v, Vec3 *out);

/**
 * @brief 四元数转三维旋转矩阵。
 *
 * @param q 输入四元数。
 * @param out 输出旋转矩阵。
 * @return 执行状态。
 */
CommonStatus rot3_from_quat(const Quat *q, RotMat3 *out);

/**
 * @brief 三维旋转矩阵转四元数。
 *
 * @param r 输入旋转矩阵。
 * @param out 输出单位四元数。
 * @return 执行状态。
 */
CommonStatus rot3_to_quat(const RotMat3 *r, Quat *out);

/**
 * @brief Euler 角转三维旋转矩阵。
 *
 * @param e 输入 Euler 角。
 * @param out 输出旋转矩阵。
 * @return 执行状态。
 */
CommonStatus rot3_from_euler(const Euler *e, RotMat3 *out);

/**
 * @brief 三维旋转矩阵转 Euler 角。
 *
 * @param r 输入旋转矩阵。
 * @param out 输出 Euler 角。
 * @return 执行状态。
 */
CommonStatus rot3_to_euler(const RotMat3 *r, Euler *out);

/**
 * @brief 计算三维矩阵行列式。
 *
 * @param r 输入 3x3 矩阵。
 * @return 行列式；输入为空或非法时返回 0。
 */
float32_t rot3_det(const RotMat3 *r);

/**
 * @brief 计算旋转矩阵正交性误差。
 *
 * 返回 R^T*R 与单位矩阵的最大绝对元素误差。该函数用于初始化和调试检查，
 * 不建议在高频控制路径中每次调用。
 *
 * @param r 输入 3x3 矩阵。
 * @return 正交性误差；输入为空或非法时返回较大值。
 */
float32_t rot3_orthogonality_error(const RotMat3 *r);

/**
 * @brief 检查矩阵是否近似 SO(3) 旋转矩阵。
 *
 * 检查 finite、R^T*R≈I 以及 det(R)≈1。tol 必须大于 0。
 *
 * @param r 输入 3x3 矩阵。
 * @param tol 允许误差。
 * @return 执行状态。
 */
CommonStatus rot3_check(const RotMat3 *r, float32_t tol);

/**
 * @brief wrap Euler 角到常用范围。
 *
 * roll/yaw wrap 到 [-pi, pi)，pitch 限制在 [-pi/2, pi/2] 附近的等价表示。
 *
 * @param e 输入 Euler 角。
 * @param out 输出 Euler 角。
 * @return 执行状态。
 */
CommonStatus euler_wrap(const Euler *e, Euler *out);

/**
 * @brief yaw-pitch-roll 转旋转矩阵。
 *
 * @param yaw yaw，单位 rad。
 * @param pitch pitch，单位 rad。
 * @param roll roll，单位 rad。
 * @param out 输出旋转矩阵。
 * @return 执行状态。
 */
CommonStatus ypr_to_rot3(float32_t yaw,
                            float32_t pitch,
                            float32_t roll,
                            RotMat3 *out);

/* ==========================================================================
 * 位姿与机构几何
 * --------------------------------------------------------------------------
 * 位姿、IMU 坐标转换、云台朝向和弹道辅助作为高层几何接口提供给 Component 与 Apps。
 * ========================================================================== */
/**
 * @brief 二维位姿作用于点。
 *
 * @param pose 位姿。
 * @param point 局部点。
 * @param out 父坐标系点。
 * @return 执行状态。
 */
CommonStatus pose2_apply(const Pose2 *pose, const Vec2 *point, Vec2 *out);

/**
 * @brief 二维位姿逆作用于点。
 *
 * @param pose 位姿。
 * @param point 父坐标系点。
 * @param out 局部坐标系点。
 * @return 执行状态。
 */
CommonStatus pose2_inv_apply(const Pose2 *pose, const Vec2 *point, Vec2 *out);

/**
 * @brief 三维位姿作用于点。
 *
 * @param pose 位姿。
 * @param point 局部点。
 * @param out 父坐标系点。
 * @return 执行状态。
 */
CommonStatus pose3_apply(const Pose3 *pose, const Vec3 *point, Vec3 *out);

/**
 * @brief 三维位姿逆作用于点。
 *
 * @param pose 位姿。
 * @param point 父坐标系点。
 * @param out 局部坐标系点。
 * @return 执行状态。
 */
CommonStatus pose3_inv_apply(const Pose3 *pose, const Vec3 *point, Vec3 *out);

/**
 * @brief IMU 坐标系向 body 坐标系转换。
 *
 * @param imu_to_body IMU 到 body 的旋转。
 * @param imu_vec IMU 坐标系向量。
 * @param body_vec body 坐标系向量。
 * @return 执行状态。
 */
CommonStatus imu_to_body(const RotMat3 *imu_to_body,
                            const Vec3 *imu_vec,
                            Vec3 *body_vec);

/**
 * @brief body 坐标系向 IMU 坐标系转换。
 *
 * @param imu_to_body IMU 到 body 的旋转。
 * @param body_vec body 坐标系向量。
 * @param imu_vec IMU 坐标系向量。
 * @return 执行状态。
 */
CommonStatus body_to_imu(const RotMat3 *imu_to_body,
                            const Vec3 *body_vec,
                            Vec3 *imu_vec);

/**
 * @brief 由云台 yaw/pitch 生成发射方向。
 *
 * 默认发射轴为局部 +X。云台约定为正 pitch 抬头，即输出方向具有正 Z 分量；
 * 内部等效旋转为 Rz(yaw)*Ry(-pitch)。
 *
 * @param yaw 云台 yaw，单位 rad。
 * @param pitch 云台 pitch，单位 rad。
 * @param out 世界或底盘坐标系中的方向单位向量。
 * @return 执行状态。
 */
CommonStatus gimbal_forward(float32_t yaw, float32_t pitch, Vec3 *out);

/**
 * @brief 由云台 yaw/pitch 生成旋转矩阵。
 *
 * 云台约定为正 pitch 抬头，内部等效旋转为 Rz(yaw)*Ry(-pitch)，roll 固定为 0。
 *
 * @param yaw 云台 yaw，单位 rad。
 * @param pitch 云台 pitch，单位 rad。
 * @param out 输出旋转矩阵。
 * @return 执行状态。
 */
CommonStatus gimbal_rot(float32_t yaw, float32_t pitch, RotMat3 *out);

/**
 * @brief 无空气阻力弹道俯仰角求解。
 *
 * horizontal 必须大于 0，speed 必须大于 0，gravity 必须大于 0。
 * vertical 为目标相对发射点的高度差，向上为正。
 *
 * @param horizontal 水平距离。
 * @param vertical 高度差。
 * @param speed 弹丸初速度。
 * @param gravity 重力加速度。
 * @param out 弹道解。
 * @return 求解状态。
 */
CommonStatus ballistic_solve(float32_t horizontal,
                                float32_t vertical,
                                float32_t speed,
                                float32_t gravity,
                                BallisticSolution *out);

/**
 * @brief 获取状态码对应的只读字符串。
 *
 * @param status 状态码。
 * @return 状态码字符串。
 */
const char *common_status_str(CommonStatus status);
#ifdef __cplusplus
}
#endif

#endif // PRINTK_ALGORITHM_COMMON_H

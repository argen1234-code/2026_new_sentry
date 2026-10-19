/**
 ******************************************************************************
 * @file    extended_kalman.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   扩展 Kalman 滤波器接口。
 *
 * @note    非线性状态方程、测量方程和雅可比矩阵由外部函数注入。
 *
 * @note    模块内部不保存隐式模型，不申请动态内存，只操作调用者提供的缓冲区。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_ALGORITHM_EKF_H
#define PRINTK_ALGORITHM_EKF_H

#include <stdint.h>
#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 类型与矩阵配置
 * --------------------------------------------------------------------------
 * 扩展 Kalman 的上下文只保存矩阵视图和维度，实际存储由外部静态工作区提供。
 * ========================================================================== */
/**
 * @brief 扩展 Kalman 滤波器执行状态。
 */
typedef enum
{
    EKF_OK = 0,           /**< 执行成功。 */
    EKF_ERR_ARG = 1,      /**< 输入指针无效。 */
    EKF_ERR_SIZE = 2,     /**< 维度或缓冲区长度无效。 */
    EKF_ERR_MATH = 3,     /**< CMSIS-DSP 矩阵运算失败。 */
    EKF_ERR_SINGULAR = 4, /**< 创新协方差矩阵不可逆。 */
    EKF_ERR_FUNC = 5      /**< 外部模型或雅可比回调执行失败。 */
} ekf_status_t;

/**
 * @brief 扩展 Kalman 协方差更新模式。
 */
typedef enum
{
    EKF_COV_COMPACT = 0,   /**< 使用 P = P - K*H*P，计算量最小。 */
    EKF_COV_SYMMETRIC = 1, /**< 使用紧凑形式后执行 P = 0.5*(P + P^T)。 */
    EKF_COV_JOSEPH = 2     /**< 使用 Joseph 形式，数值稳定性更好。 */
} ekf_cov_t;

/**
 * @brief 非线性状态方程回调。
 *
 * @param x_out 状态预测输出向量，长度为 x_dim。
 * @param x 当前状态向量，长度为 x_dim。
 * @param u 控制向量，长度为 u_dim，u_dim 为 0 时为空。
 * @param x_dim 状态向量维度。
 * @param u_dim 控制向量维度。
 * @param arg 用户注入的状态模型上下文。
 * @return 回调执行状态。
 */
typedef ekf_status_t (*ekf_state_fn)(float32_t *x_out,
                                                             const float32_t *x,
                                                             const float32_t *u,
                                                             uint16_t x_dim,
                                                             uint16_t u_dim,
                                                             void *arg);

/**
 * @brief 状态方程雅可比矩阵回调。
 *
 * @param f_out 状态雅可比矩阵 F 输出缓冲区，长度为 x_dim*x_dim。
 * @param x 当前状态向量，长度为 x_dim。
 * @param u 控制向量，长度为 u_dim，u_dim 为 0 时为空。
 * @param x_dim 状态向量维度。
 * @param u_dim 控制向量维度。
 * @param arg 用户注入的状态模型上下文。
 * @return 回调执行状态。
 */
typedef ekf_status_t (*ekf_jac_fn)(float32_t *f_out,
                                                           const float32_t *x,
                                                           const float32_t *u,
                                                           uint16_t x_dim,
                                                           uint16_t u_dim,
                                                           void *arg);

/**
 * @brief 非线性测量方程回调。
 *
 * @param z_out 测量预测输出向量，长度为 z_dim。
 * @param x 当前状态向量，长度为 x_dim。
 * @param x_dim 状态向量维度。
 * @param z_dim 测量向量维度。
 * @param arg 用户注入的测量模型上下文。
 * @return 回调执行状态。
 */
typedef ekf_status_t (*ekf_meas_fn)(float32_t *z_out,
                                                            const float32_t *x,
                                                            uint16_t x_dim,
                                                            uint16_t z_dim,
                                                            void *arg);

/**
 * @brief 测量方程雅可比矩阵回调。
 *
 * @param h_out 测量雅可比矩阵 H 输出缓冲区，长度为 z_dim*x_dim。
 * @param x 当前状态向量，长度为 x_dim。
 * @param x_dim 状态向量维度。
 * @param z_dim 测量向量维度。
 * @param arg 用户注入的测量模型上下文。
 * @return 回调执行状态。
 */
typedef ekf_status_t (*ekf_hjac_fn)(float32_t *h_out,
                                                            const float32_t *x,
                                                            uint16_t x_dim,
                                                            uint16_t z_dim,
                                                            void *arg);

/**
 * @brief 测量残差回调。
 *
 * 当测量空间包含角度或李群变量时，可用该回调替代默认的 y = z - z_pred。
 *
 * @param y_out 残差输出向量，长度为 z_dim。
 * @param z 实际测量向量，长度为 z_dim。
 * @param pred 预测测量向量，长度为 z_dim。
 * @param z_dim 测量向量维度。
 * @param arg 用户注入的测量模型上下文。
 * @return 回调执行状态。
 */
typedef ekf_status_t (*ekf_res_fn)(float32_t *y_out,
                                                           const float32_t *z,
                                                           const float32_t *pred,
                                                           uint16_t z_dim,
                                                           void *arg);

/**
 * @brief 状态注入回调。
 *
 * 当状态空间不适合普通欧式加法时，可用该回调替代默认的 x_out = x + dx。
 *
 * @param x_out 更新后的状态输出向量，长度为 x_dim。
 * @param x 当前状态向量，长度为 x_dim。
 * @param dx 状态增量向量，长度为 x_dim。
 * @param x_dim 状态向量维度。
 * @param arg 用户注入的状态模型上下文。
 * @return 回调执行状态。
 */
typedef ekf_status_t (*ekf_inject_fn)(float32_t *x_out,
                                                              const float32_t *x,
                                                              const float32_t *dx,
                                                              uint16_t x_dim,
                                                              void *arg);

/**
 * @brief 扩展 Kalman 滤波器配置。
 */
typedef struct
{
    uint16_t x_dim;                 /**< 状态向量维度 n。 */
    uint16_t z_dim;                 /**< 测量向量维度 m。 */
    uint16_t u_dim;                 /**< 控制向量维度 c，无控制输入时填 0。 */
    float32_t *x;                   /**< 状态向量 x，长度 n。 */
    float32_t *p;                   /**< 状态协方差 P，长度 n*n。 */
    float32_t *q;                   /**< 过程噪声协方差 Q，长度 n*n。 */
    float32_t *r;                   /**< 测量噪声协方差 R，长度 m*m。 */
    float32_t *f;                   /**< 状态雅可比矩阵 F，长度 n*n。 */
    float32_t *h;                   /**< 测量雅可比矩阵 H，长度 m*n。 */
    float32_t *u;                   /**< 控制向量 u，长度 c，u_dim 为 0 时可为空。 */
    float32_t *k;                   /**< Kalman 增益 K，长度 n*m。 */
    float32_t *y;                   /**< 创新向量 y，长度 m。 */
    float32_t *s;                   /**< 创新协方差 S，长度 m*m。 */
    ekf_cov_t cov;      /**< 协方差更新模式。 */
    ekf_state_fn state; /**< 非线性状态方程回调。 */
    ekf_jac_fn jac;     /**< 状态方程雅可比矩阵回调。 */
    ekf_meas_fn meas;   /**< 非线性测量方程回调。 */
    ekf_hjac_fn hjac;   /**< 测量方程雅可比矩阵回调。 */
    ekf_res_fn res;     /**< 测量残差回调，为空时使用默认残差。 */
    ekf_inject_fn inject; /**< 状态注入回调，为空时使用默认注入。 */
    void *state_arg;                /**< 状态模型用户上下文。 */
    void *meas_arg;                 /**< 测量模型用户上下文。 */
} ekf_cfg_t;

/**
 * @brief 扩展 Kalman 滤波器上下文。
 */
typedef struct
{
    uint16_t x_dim;                 /**< 状态向量维度 n。 */
    uint16_t z_dim;                 /**< 测量向量维度 m。 */
    uint16_t u_dim;                 /**< 控制向量维度 c。 */
    uint8_t has_ctrl;               /**< 非 0 表示启用控制输入。 */
    ekf_cov_t cov;      /**< 协方差更新模式。 */
    arm_matrix_instance_f32 x;      /**< 状态向量 x。 */
    arm_matrix_instance_f32 p;      /**< 状态协方差 P。 */
    arm_matrix_instance_f32 q;      /**< 过程噪声协方差 Q。 */
    arm_matrix_instance_f32 r;      /**< 测量噪声协方差 R。 */
    arm_matrix_instance_f32 f;      /**< 状态雅可比矩阵 F。 */
    arm_matrix_instance_f32 h;      /**< 测量雅可比矩阵 H。 */
    arm_matrix_instance_f32 u;      /**< 控制向量 u。 */
    arm_matrix_instance_f32 k;      /**< Kalman 增益 K。 */
    arm_matrix_instance_f32 y;      /**< 创新向量 y。 */
    arm_matrix_instance_f32 s;      /**< 创新协方差 S。 */
    ekf_state_fn state; /**< 非线性状态方程回调。 */
    ekf_jac_fn jac;     /**< 状态方程雅可比矩阵回调。 */
    ekf_meas_fn meas;   /**< 非线性测量方程回调。 */
    ekf_hjac_fn hjac;   /**< 测量方程雅可比矩阵回调。 */
    ekf_res_fn res;     /**< 测量残差回调。 */
    ekf_inject_fn inject; /**< 状态注入回调。 */
    void *state_arg;                /**< 状态模型用户上下文。 */
    void *meas_arg;                 /**< 测量模型用户上下文。 */
} ekf_t;

/**
 * @brief 扩展 Kalman 滤波器工作区。
 */
typedef struct
{
    float32_t *buf;              /**< 工作区原始缓冲区首地址。 */
    uint32_t len;                /**< 工作区原始缓冲区长度，单位为 float32_t 元素。 */
    arm_matrix_instance_f32 x1;  /**< 临时 n*1 向量。 */
    arm_matrix_instance_f32 z1;  /**< 临时 m*1 向量。 */
    arm_matrix_instance_f32 nn1; /**< 临时 n*n 矩阵。 */
    arm_matrix_instance_f32 nn2; /**< 临时 n*n 矩阵。 */
    arm_matrix_instance_f32 nn3; /**< 临时 n*n 矩阵。 */
    arm_matrix_instance_f32 nm1; /**< 临时 n*m 矩阵。 */
    arm_matrix_instance_f32 mn1; /**< 临时 m*n 矩阵。 */
    arm_matrix_instance_f32 mm1; /**< 临时 m*m 矩阵。 */
    arm_matrix_instance_f32 mm2; /**< 临时 m*m 矩阵。 */
} ekf_work_t;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按工作区计算、初始化、predict、correct、step 的顺序排列，方便新人按数据流阅读。
 * ========================================================================== */
/**
 * @brief 计算工作区所需的最小元素数量。
 *
 * @param x_dim 状态向量维度 n。
 * @param z_dim 测量向量维度 m。
 * @return 所需 float32_t 元素数量，参数非法时返回 0。
 */
uint32_t ekf_work_size(uint16_t x_dim, uint16_t z_dim);

/**
 * @brief 初始化扩展 Kalman 滤波器上下文。
 *
 * @param kalman 扩展 Kalman 滤波器上下文指针。
 * @param cfg 外部注入的配置、矩阵缓冲区和回调。
 * @return 初始化状态。
 */
ekf_status_t ekf_init(ekf_t *kalman,
                                              const ekf_cfg_t *cfg);

/**
 * @brief 初始化扩展 Kalman 滤波器工作区。
 *
 * @param work 工作区结构体指针。
 * @param buf 外部提供的连续 float32_t 缓冲区。
 * @param len buf 的元素数量。
 * @param x_dim 状态向量维度 n。
 * @param z_dim 测量向量维度 m。
 * @return 初始化状态。
 */
ekf_status_t ekf_work_init(ekf_work_t *work,
                                                   float32_t *buf,
                                                   uint32_t len,
                                                   uint16_t x_dim,
                                                   uint16_t z_dim);

/**
 * @brief 执行扩展 Kalman 预测步骤。
 *
 * @param kalman 已初始化的滤波器上下文。
 * @param work 已初始化且维度匹配的工作区。
 * @return 预测状态。
 */
ekf_status_t ekf_predict(ekf_t *kalman,
                                                 ekf_work_t *work);

/**
 * @brief 执行扩展 Kalman 校正步骤。
 *
 * @param kalman 已初始化的滤波器上下文。
 * @param work 已初始化且维度匹配的工作区。
 * @param z 测量向量，长度为 z_dim。
 * @return 校正状态。
 */
ekf_status_t ekf_correct(ekf_t *kalman,
                                                 ekf_work_t *work,
                                                 const float32_t *z);

/**
 * @brief 执行一次完整扩展 Kalman 预测和校正。
 *
 * @param kalman 已初始化的滤波器上下文。
 * @param work 已初始化且维度匹配的工作区。
 * @param z 测量向量，长度为 z_dim。
 * @return 执行状态。
 */
ekf_status_t ekf_step(ekf_t *kalman,
                                              ekf_work_t *work,
                                              const float32_t *z);

/**
 * @brief 获取状态码对应的只读字符串。
 *
 * @param status 扩展 Kalman 状态码。
 * @return 状态码对应的只读字符串。
 */
const char *ekf_status_str(ekf_status_t status);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_ALGORITHM_EKF_H

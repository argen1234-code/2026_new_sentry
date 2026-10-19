/**
 ******************************************************************************
 * @file    kalman.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   线性 Kalman 滤波器接口。
 *
 * @note    本模块仅依赖标准整型和 CMSIS-DSP 矩阵接口，不申请动态内存。
 *
 * @note    状态、模型、噪声、输出和临时工作区均由调用者注入，便于任务层静态分配。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_ALGORITHM_KF_H
#define PRINTK_ALGORITHM_KF_H

#include <stdint.h>
#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 类型与矩阵配置
 * --------------------------------------------------------------------------
 * 线性 Kalman 的上下文只保存矩阵视图和维度，实际存储由外部静态工作区提供。
 * ========================================================================== */
/**
 * @brief 线性 Kalman 滤波器执行状态。
 */
typedef enum
{
    KF_OK = 0,           /**< 执行成功。 */
    KF_ERR_ARG = 1,      /**< 输入指针无效。 */
    KF_ERR_SIZE = 2,     /**< 维度或缓冲区长度无效。 */
    KF_ERR_MATH = 3,     /**< CMSIS-DSP 矩阵运算失败。 */
    KF_ERR_SINGULAR = 4  /**< 创新协方差矩阵不可逆。 */
} kf_status_t;

/**
 * @brief 线性 Kalman 协方差更新模式。
 */
typedef enum
{
    KF_COV_COMPACT = 0,   /**< 使用 P = P - K*H*P，计算量最小。 */
    KF_COV_SYMMETRIC = 1, /**< 使用紧凑形式后执行 P = 0.5*(P + P^T)。 */
    KF_COV_JOSEPH = 2     /**< 使用 Joseph 形式，数值稳定性更好。 */
} kf_cov_t;

/**
 * @brief 线性 Kalman 滤波器配置。
 *
 * 所有矩阵均为行主序缓冲区。函数只记录这些缓冲区地址，不清零也不修改初值。
 */
typedef struct
{
    uint16_t x_dim;      /**< 状态向量维度 n。 */
    uint16_t z_dim;      /**< 测量向量维度 m。 */
    uint16_t u_dim;      /**< 控制向量维度 c，无控制输入时填 0。 */
    float32_t *x;        /**< 状态向量 x，长度 n。 */
    float32_t *p;        /**< 状态协方差 P，长度 n*n。 */
    float32_t *f;        /**< 状态转移矩阵 F，长度 n*n。 */
    float32_t *q;        /**< 过程噪声协方差 Q，长度 n*n。 */
    float32_t *h;        /**< 测量矩阵 H，长度 m*n。 */
    float32_t *r;        /**< 测量噪声协方差 R，长度 m*m。 */
    float32_t *b;        /**< 控制矩阵 B，长度 n*c，u_dim 为 0 时可为空。 */
    float32_t *u;        /**< 控制向量 u，长度 c，u_dim 为 0 时可为空。 */
    float32_t *k;        /**< Kalman 增益 K，长度 n*m，可在校正后读取。 */
    float32_t *y;        /**< 创新向量 y，长度 m，可在校正后读取。 */
    float32_t *s;        /**< 创新协方差 S，长度 m*m，可在校正后读取。 */
    kf_cov_t cov;    /**< 协方差更新模式。 */
} kf_cfg_t;

/**
 * @brief 线性 Kalman 滤波器上下文。
 *
 * 该结构体只保存矩阵视图和维度信息，不拥有任何内存。
 */
typedef struct
{
    uint16_t x_dim;          /**< 状态向量维度 n。 */
    uint16_t z_dim;          /**< 测量向量维度 m。 */
    uint16_t u_dim;          /**< 控制向量维度 c。 */
    uint8_t has_ctrl;        /**< 非 0 表示启用控制输入。 */
    kf_cov_t cov;        /**< 协方差更新模式。 */
    arm_matrix_instance_f32 x; /**< 状态向量 x。 */
    arm_matrix_instance_f32 p; /**< 状态协方差 P。 */
    arm_matrix_instance_f32 f; /**< 状态转移矩阵 F。 */
    arm_matrix_instance_f32 q; /**< 过程噪声协方差 Q。 */
    arm_matrix_instance_f32 h; /**< 测量矩阵 H。 */
    arm_matrix_instance_f32 r; /**< 测量噪声协方差 R。 */
    arm_matrix_instance_f32 b; /**< 控制矩阵 B。 */
    arm_matrix_instance_f32 u; /**< 控制向量 u。 */
    arm_matrix_instance_f32 k; /**< Kalman 增益 K。 */
    arm_matrix_instance_f32 y; /**< 创新向量 y。 */
    arm_matrix_instance_f32 s; /**< 创新协方差 S。 */
} kf_t;

/**
 * @brief 线性 Kalman 滤波器工作区。
 *
 * 工作区由外部连续 float32_t 缓冲区切分而来，用于保存临时矩阵视图。
 */
typedef struct
{
    float32_t *buf;          /**< 工作区原始缓冲区首地址。 */
    uint32_t len;            /**< 工作区原始缓冲区长度，单位为 float32_t 元素。 */
    arm_matrix_instance_f32 x1;  /**< 临时 n*1 向量。 */
    arm_matrix_instance_f32 x2;  /**< 临时 n*1 向量。 */
    arm_matrix_instance_f32 nn1; /**< 临时 n*n 矩阵。 */
    arm_matrix_instance_f32 nn2; /**< 临时 n*n 矩阵。 */
    arm_matrix_instance_f32 nn3; /**< 临时 n*n 矩阵。 */
    arm_matrix_instance_f32 nm1; /**< 临时 n*m 矩阵。 */
    arm_matrix_instance_f32 mn1; /**< 临时 m*n 矩阵。 */
    arm_matrix_instance_f32 mm1; /**< 临时 m*m 矩阵。 */
    arm_matrix_instance_f32 mm2; /**< 临时 m*m 矩阵。 */
} kf_work_t;

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
uint32_t kf_work_size(uint16_t x_dim, uint16_t z_dim);

/**
 * @brief 初始化线性 Kalman 滤波器上下文。
 *
 * @param kalman 线性 Kalman 滤波器上下文指针。
 * @param cfg 外部注入的配置和矩阵缓冲区。
 * @return 初始化状态。
 */
kf_status_t kf_init(kf_t *kalman, const kf_cfg_t *cfg);

/**
 * @brief 初始化线性 Kalman 滤波器工作区。
 *
 * @param work 工作区结构体指针。
 * @param buf 外部提供的连续 float32_t 缓冲区。
 * @param len buf 的元素数量。
 * @param x_dim 状态向量维度 n。
 * @param z_dim 测量向量维度 m。
 * @return 初始化状态。
 */
kf_status_t kf_work_init(kf_work_t *work,
                                 float32_t *buf,
                                 uint32_t len,
                                 uint16_t x_dim,
                                 uint16_t z_dim);

/**
 * @brief 执行线性 Kalman 预测步骤。
 *
 * @param kalman 已初始化的滤波器上下文。
 * @param work 已初始化且维度匹配的工作区。
 * @return 预测状态。
 */
kf_status_t kf_predict(kf_t *kalman, kf_work_t *work);

/**
 * @brief 执行线性 Kalman 校正步骤。
 *
 * @param kalman 已初始化的滤波器上下文。
 * @param work 已初始化且维度匹配的工作区。
 * @param z 测量向量，长度为 z_dim。
 * @return 校正状态。
 */
kf_status_t kf_correct(kf_t *kalman,
                               kf_work_t *work,
                               const float32_t *z);

/**
 * @brief 执行一次完整线性 Kalman 预测和校正。
 *
 * @param kalman 已初始化的滤波器上下文。
 * @param work 已初始化且维度匹配的工作区。
 * @param z 测量向量，长度为 z_dim。
 * @return 执行状态。
 */
kf_status_t kf_step(kf_t *kalman,
                            kf_work_t *work,
                            const float32_t *z);

/**
 * @brief 获取状态码对应的只读字符串。
 *
 * @param status 线性 Kalman 状态码。
 * @return 状态码对应的只读字符串。
 */
const char *kf_status_str(kf_status_t status);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_ALGORITHM_KF_H

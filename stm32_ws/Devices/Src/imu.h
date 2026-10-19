/**
 ******************************************************************************
 * @file    imu.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   IMU 设备公共接口。
 *
 * @note    本文件只定义 IMU 设备抽象接口，上层不依赖具体 IMU 型号。
 * @note    当前板卡实际 IMU 型号、SPI/GPIO 绑定和默认标定由 imu.c 内部处理。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_IMU_H
#define PRINTK_DEVICE_IMU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 设备头文件描述协议事实、配置结构、状态和诊断，具体接口绑定由 Robot 配置展开。
 * ========================================================================== */

/**
 * @brief IMU 设备状态。
 */
typedef enum
{
    IMU_OK = 0,             /**< 操作成功。 */
    IMU_ERR = -1,           /**< 通用错误。 */
    IMU_ERR_ARG = -2,       /**< 输入参数无效。 */
    IMU_ERR_IO = -3,        /**< 底层通信或端口错误。 */
    IMU_ERR_NOT_READY = -4, /**< IMU 尚未完成初始化。 */
    IMU_ERR_CALI = -5       /**< IMU 标定失败。 */
} imu_status_t;

/**
 * @brief IMU 最近一次采样值。
 */
typedef struct
{
    float accel[3];     /**< 加速度，顺序为 x、y、z。 */
    float gyro[3];      /**< 角速度，顺序为 x、y、z。 */
    float temperature;  /**< 当前温度，单位摄氏度。 */
    float cal_temp;     /**< 默认标定温度，单位摄氏度。 */
} imu_sample_t;

/**
 * @brief IMU 标定参数。
 */
typedef struct
{
    float gyro_offset[3];         /**< 三轴陀螺仪零偏，单位 rad/s。 */
    float accel_offset[3];        /**< 三轴加速度计零偏，单位 m/s^2。 */
    float g_norm;                 /**< 标定重力模长，单位 m/s^2。 */
    float temp_when_cali;         /**< 标定时温度，单位摄氏度。 */
    uint8_t accel_offset_enabled; /**< 非 0 表示启用加速度零偏补偿。 */
} ImuCali;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口只表达设备能做什么，不把遥控器、裁判系统或电机解释成具体机器人控制动作。
 * ========================================================================== */

/**
 * @brief 初始化板载 IMU。
 *
 * @param calibrate 非 0 时初始化后执行在线标定；0 时使用板级默认标定。
 * @return 返回 IMU 设备状态。
 */
imu_status_t imu_init(uint8_t calibrate);

/**
 * @brief 读取并更新板载 IMU 测量值。
 *
 * @return 返回 IMU 设备状态。
 */
imu_status_t imu_read(void);

/**
 * @brief 对板载 IMU 执行在线标定。
 *
 * @return 返回 IMU 设备状态。
 */
imu_status_t imu_calibrate(void);

/**
 * @brief 应用外部保存的 IMU 标定参数。
 *
 * 该函数可在 imu_init() 前调用，后续初始化将使用该参数；也可在 IMU
 * 已 ready 后调用，立即更新当前设备对象。
 *
 * @param cali 标定参数。
 * @return 返回 IMU 设备状态。
 */
imu_status_t imu_set_cali(const ImuCali *cali);

/**
 * @brief 读取当前 IMU 标定参数。
 *
 * @param out 标定参数输出。
 * @return 返回 IMU 设备状态。
 */
imu_status_t imu_get_cali(ImuCali *out);

/**
 * @brief 获取最近一次 IMU 采样值。
 *
 * @param out 采样值输出。
 * @return 返回 IMU 设备状态。
 */
imu_status_t imu_sample(imu_sample_t *out);

/**
 * @brief 获取 IMU 默认标定温度。
 *
 * @return 默认标定温度，单位摄氏度。
 */
float imu_cal_temp(void);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_DEVICE_IMU_H

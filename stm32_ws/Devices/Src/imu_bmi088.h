/**
 ******************************************************************************
 * @file    imu_bmi088.h
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   BMI088 IMU 设备驱动对外接口。
 *
 * @note    本文件定义 BMI088 端口抽象、设备对象、标定参数、错误码和
 *          对外初始化/读数/标定接口。
 * @note    驱动不直接依赖 HAL 或板级 GPIO/SPI 资源，调用方需通过
 *          bmi088_port_t 注入底层操作。
 * @note    差异: 旧版 BMI088driver.h 暴露 HAL SPI 句柄绑定、全局 BMI088 数据对象
 *          和板级默认零偏；新版只暴露器件驱动接口，板级资源与标定参数由
 *          外部适配层组合。
 * @note    差异: 新版保留旧版常用量程、错误码和读数单位，但初始化、读数和
 *          标定接口改为返回状态码，不再通过 void 接口或死循环隐藏异常。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_BMI088_H
#define PRINTK_DEVICE_BMI088_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 设备头文件描述协议事实、配置结构、状态和诊断，具体接口绑定由 Robot 配置展开。
 * ========================================================================== */
#define BMI088_TEMP_FACTOR 0.125f                                         /**< BMI088 温度寄存器换算系数，单位为摄氏度每 LSB。 */
#define BMI088_TEMP_OFFSET 23.0f                                          /**< BMI088 温度寄存器换算偏移量，单位为摄氏度。 */
#define BMI088_ACCEL_3G_SEN 0.0008974358974f                              /**< BMI088 默认加速度计量程为 3G 时的灵敏度，单位为 m/s^2/LSB。 */
#define BMI088_ACCEL_6G_SEN 0.00179443359375f                             /**< BMI088 默认加速度计量程为 6G 时的灵敏度，单位为 m/s^2/LSB。 */
#define BMI088_ACCEL_12G_SEN 0.0035888671875f                             /**< BMI088 默认加速度计量程为 12G 时的灵敏度，单位为 m/s^2/LSB。 */
#define BMI088_ACCEL_24G_SEN 0.007177734375f                              /**< BMI088 默认加速度计量程为 24G 时的灵敏度，单位为 m/s^2/LSB。 */
#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f          /**< BMI088 默认陀螺仪量程为 2000dps 时的灵敏度，单位为 rad/s/LSB。 */
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f       /**< BMI088 默认陀螺仪量程为 1000dps 时的灵敏度，单位为 rad/s/LSB。 */
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f        /**< BMI088 默认陀螺仪量程为 500dps 时的灵敏度，单位为 rad/s/LSB。 */
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f        /**< BMI088 默认陀螺仪量程为 250dps 时的灵敏度，单位为 rad/s/LSB。 */
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f       /**< BMI088 默认陀螺仪量程为 125dps 时的灵敏度，单位为 rad/s/LSB。 */
#define BMI088_DEFAULT_G_NORM 9.81f                                       /**< 未提供板级标定参数时使用的标准重力模长，单位为 m/s^2。 */
#define BMI088_LONG_DELAY_TIME 80U                                        /**< 初始化前等待传感器复位稳定的长延时，单位为 ms。 */
#define BMI088_CALI_GYRO_TIMES 6000U                                      /**< 陀螺仪零偏标定采样次数。 */
#define BMI088_CALI_ACCEL_TIMES 6000U                                     /**< 加速度零偏标定采样次数。 */
#define BMI088_CALI_TIMEOUT_S 10.0f                                       /**< 标定超时时间，单位为 s。 */
#define BMI088_CALI_G_NORM_DIFF_THRES 0.5f                                /**< 标定阶段允许的重力模长极差阈值，单位为 m/s^2。 */
#define BMI088_CALI_G_NORM_ABS_THRES 0.5f                                 /**< 标定阶段允许的目标重力模长偏差阈值，单位为 m/s^2。 */
#define BMI088_CALI_GYRO_DIFF_THRES 0.15f                                 /**< 标定阶段允许的单轴陀螺仪极差阈值，单位为 rad/s。 */
#define BMI088_CALI_GYRO_OFFSET_ABS_THRES 0.01f                           /**< 标定结果允许的单轴陀螺仪零偏绝对值阈值，单位为 rad/s。 */
#define BMI088_STATUS_OK 0                                                /**< 驱动接口调用成功。 */
#define BMI088_STATUS_INVALID_ARG (-1)                                    /**< 输入参数为空或无效。 */
#define BMI088_STATUS_PORT_ERROR (-2)                                     /**< 端口回调未提供或不完整。 */
#define BMI088_STATUS_CALI_TIMEOUT (-3)                                   /**< 标定在规定时间内未收敛。 */
#define BMI088_STATUS_CALI_UNSTABLE (-4)                                  /**< 标定采样过程检测到 IMU 不稳定。 */
#define BMI088_NO_ERROR 0x00                                              /**< BMI088 无错误。 */
#define BMI088_ACC_PWR_CTRL_ERROR 0x01                                    /**< BMI088 加速度计电源控制寄存器配置错误。 */
#define BMI088_ACC_PWR_CONF_ERROR 0x02                                    /**< BMI088 加速度计电源模式寄存器配置错误。 */
#define BMI088_ACC_CONF_ERROR 0x03                                        /**< BMI088 加速度计配置寄存器错误。 */
#define BMI088_ACC_SELF_TEST_ERROR 0x04                                   /**< BMI088 加速度计自检错误。 */
#define BMI088_ACC_RANGE_ERROR 0x05                                       /**< BMI088 加速度计量程寄存器错误。 */
#define BMI088_INT1_IO_CTRL_ERROR 0x06                                    /**< BMI088 加速度计中断引脚配置错误。 */
#define BMI088_INT_MAP_DATA_ERROR 0x07                                    /**< BMI088 加速度计中断映射错误。 */
#define BMI088_GYRO_RANGE_ERROR 0x08                                      /**< BMI088 陀螺仪量程寄存器错误。 */
#define BMI088_GYRO_BANDWIDTH_ERROR 0x09                                  /**< BMI088 陀螺仪带宽寄存器错误。 */
#define BMI088_GYRO_LPM1_ERROR 0x0A                                       /**< BMI088 陀螺仪功耗模式寄存器错误。 */
#define BMI088_GYRO_CTRL_ERROR 0x0B                                       /**< BMI088 陀螺仪控制寄存器错误。 */
#define BMI088_GYRO_INT3_INT4_IO_CONF_ERROR 0x0C                          /**< BMI088 陀螺仪中断引脚配置错误。 */
#define BMI088_GYRO_INT3_INT4_IO_MAP_ERROR 0x0D                           /**< BMI088 陀螺仪中断映射错误。 */
#define BMI088_SELF_TEST_ACCEL_ERROR 0x80                                 /**< BMI088 加速度计自检失败。 */
#define BMI088_SELF_TEST_GYRO_ERROR 0x40                                  /**< BMI088 陀螺仪自检失败。 */
#define BMI088_NO_SENSOR 0xFF                                             /**< 未检测到 BMI088 传感器。 */

/**
 * @brief BMI088 底层端口抽象。
 *
 * 调用方需通过回调提供 SPI 字节收发、片选控制、延时与时间基准。
 * `spi_transfer` 为可选 burst 传输接口，返回 0 表示成功；传入 NULL 时驱动会退回到
 * `spi_transfer_byte`。
 *
 * @note 差异: 旧版由 BMI088Middleware 直接绑定 HAL SPI 和 GPIO；新版通过该结构
 *       注入端口能力，使 BMI088 设备驱动不感知具体 SPI 实例或片选引脚。
 */
typedef struct {
    void *user;                                                                     /**< 用户上下文指针。 */
    void (*accel_cs)(void *user, uint8_t level);                                    /**< 加速度计片选控制回调。 */
    void (*gyro_cs)(void *user, uint8_t level);                                     /**< 陀螺仪片选控制回调。 */
    uint8_t (*spi_transfer_byte)(void *user, uint8_t tx);                           /**< SPI 单字节收发回调。 */
    int (*spi_transfer)(void *user, const uint8_t *tx, uint8_t *rx, uint16_t len);  /**< SPI 批量收发回调，可为 NULL。 */
    void (*delay_ms)(void *user, uint32_t ms);                                      /**< 毫秒延时回调。 */
    void (*delay_us)(void *user, uint32_t us);                                      /**< 微秒延时回调。 */
    float (*get_time_s)(void *user);                                                /**< 秒级时间基准回调。 */
} bmi088_port_t;

/**
 * @brief BMI088 标定参数。
 *
 * 该结构应由 board/device 适配层提供。传入 NULL 时，驱动仅使用零偏为 0、
 * 重力模长为 9.81 m/s^2 的空标定。
 *
 * @note 差异: 旧版默认零偏是特定板卡/特定 IMU 的实测值；新版不在通用
 *       设备头文件中保存这类板级数据。
 */
typedef struct {
    float gyro_offset[3];                                                           /**< 三轴陀螺仪零偏，单位 rad/s。 */
    float accel_offset[3];                                                          /**< 三轴加速度计零偏，单位 m/s^2。 */
    float g_norm;                                                                   /**< 标定重力模长，单位 m/s^2。 */
    float temp_when_cali;                                                           /**< 标定时温度，单位摄氏度。 */
    uint8_t accel_offset_enabled;                                                   /**< 非 0 时启用加速度零偏补偿。 */
} bmi088_calibration_t;

/**
 * @brief BMI088 设备对象。
 *
 * 该对象由调用方持有，既保存端口回调，也保存当前测量值和标定参数。
 */
typedef struct {
    bmi088_port_t port;                                                             /**< 底层端口回调集合。 */
    float accel[3];                                                                 /**< 三轴加速度，单位 m/s^2。 */
    float gyro[3];                                                                  /**< 三轴角速度，单位 rad/s。 */
    float temperature;                                                              /**< 当前温度，单位摄氏度。 */
    float temp_when_cali;                                                           /**< 标定时温度，单位摄氏度。 */
    float accel_scale;                                                              /**< 加速度重力模长缩放系数。 */
    float accel_sen;                                                                /**< 加速度计灵敏度，单位 m/s^2/LSB。 */
    float gyro_sen;                                                                 /**< 陀螺仪灵敏度，单位 rad/s/LSB。 */
    float gyro_offset[3];                                                           /**< 三轴陀螺仪零偏，单位 rad/s。 */
    float accel_offset[3];                                                          /**< 三轴加速度计零偏，单位 m/s^2。 */
    float g_norm;                                                                   /**< 标定重力模长，单位 m/s^2。 */
    uint8_t accel_offset_enabled;                                                   /**< 非 0 时启用加速度零偏补偿。 */
} bmi088_t;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口只表达设备能做什么，不把遥控器、裁判系统或电机解释成具体机器人控制动作。
 * ========================================================================== */

/**
 * @brief 初始化 BMI088 设备对象。
 *
 * 该函数会复制端口回调与默认标定参数，随后按原工程流程完成加速度计和陀螺仪初始化。
 * 当 `calibrate` 非 0 时，会继续执行一次零偏标定。
 *
 * @note 差异: 旧版初始化失败时可由外层 `BMI088_Init` 死循环重试；新版仅返回
 *       状态码，重试策略由上层决定。
 * @param dev 待初始化的设备对象。
 * @param port 端口回调集合。
 * @param calibration 板级标定参数；传入 `NULL` 时使用驱动内置空标定。
 * @param calibrate 是否在初始化后立即执行标定，非 0 表示执行。
 * @return `BMI088_STATUS_INVALID_ARG`、`BMI088_STATUS_PORT_ERROR` 或 BMI088 初始化错误码。
 */
int bmi088_init(bmi088_t *dev, const bmi088_port_t *port, const bmi088_calibration_t *calibration, uint8_t calibrate);

/**
 * @brief 初始化 BMI088 加速度计。
 *
 * @param dev 设备对象。
 * @return `BMI088_STATUS_INVALID_ARG`、`BMI088_STATUS_PORT_ERROR` 或 BMI088 初始化错误码。
 */
int bmi088_accel_init(bmi088_t *dev);

/**
 * @brief 初始化 BMI088 陀螺仪。
 *
 * @param dev 设备对象。
 * @return `BMI088_STATUS_INVALID_ARG`、`BMI088_STATUS_PORT_ERROR` 或 BMI088 初始化错误码。
 */
int bmi088_gyro_init(bmi088_t *dev);

/**
 * @brief 执行 BMI088 零偏与重力模长标定。
 *
 * 标定期间需保证 IMU 处于水平静止状态。标定完成后会更新 `gyro_offset`、
 * `accel_offset`、`g_norm`、`accel_scale` 和 `temp_when_cali`。
 *
 * @note 差异: 旧版标定超时后仍返回成功；新版超时或采样不稳定会返回显式错误码。
 * @param dev 设备对象。
 * @return `BMI088_STATUS_INVALID_ARG`、`BMI088_STATUS_PORT_ERROR`、标定错误码或 `BMI088_STATUS_OK`。
 */
int bmi088_calibrate(bmi088_t *dev);

/**
 * @brief 读取 BMI088 当前测量值。
 *
 * 该函数会更新设备对象中的三轴加速度、三轴角速度与温度结果。
 * 若启用了加速度零偏补偿，则仅对 X/Y 轴执行补偿，Z 轴保持原工程逻辑不扣除零偏。
 *
 * @param dev 设备对象。
 * @return `BMI088_STATUS_INVALID_ARG`、`BMI088_STATUS_PORT_ERROR`、`BMI088_NO_SENSOR` 或 `BMI088_STATUS_OK`。
 */
int bmi088_read(bmi088_t *dev);

/**
 * @brief 读取 BMI088 当前测量值，可选择是否读取温度。
 *
 * @param dev 设备对象。
 * @param read_temp 非 0 表示本次读取温度；0 表示复用设备对象中的上次温度。
 * @return `BMI088_STATUS_INVALID_ARG`、`BMI088_STATUS_PORT_ERROR`、`BMI088_NO_SENSOR` 或 `BMI088_STATUS_OK`。
 */
int bmi088_read_ex(bmi088_t *dev, uint8_t read_temp);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_DEVICE_BMI088_H

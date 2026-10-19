/**
 ******************************************************************************
 * @file    tfmini.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   TFmini 测距设备接口。
 *
 * @note    本文件定义当前板载 TFmini 的 UART 接收、帧解析和测量值读取接口。
 * @note    底层 UART 资源仍由 BSP UART 提供，本层只保留 TFmini 设备语义。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_TFMINI_H
#define PRINTK_DEVICE_TFMINI_H

#include "robot_board.h"

#include <stdbool.h>
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
 * @brief TFmini 设备状态。
 */
typedef enum
{
    TFMINI_OK = 0,             /**< 操作成功。 */
    TFMINI_ERR = -1,           /**< 通用错误。 */
    TFMINI_ERR_ARG = -2,       /**< 输入参数无效。 */
    TFMINI_ERR_IO = -3,        /**< 底层 UART 操作失败。 */
    TFMINI_ERR_NOT_READY = -4  /**< TFmini 尚未初始化。 */
} tfmini_status_t;

/**
 * @brief TFmini 测距数据。
 */
typedef struct
{
    uint16_t distance_cm;      /**< 距离，单位 cm。 */
    uint16_t strength;         /**< 信号强度。 */
    int16_t temp_cdeg;         /**< 温度，单位 0.01 摄氏度。 */
    uint8_t valid;             /**< 非 0 表示信号强度有效。 */
    uint64_t timestamp_us;     /**< 采样时间戳，单位 us。 */
} tfmini_meas_t;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口只表达设备能做什么，不把遥控器、裁判系统或电机解释成具体机器人控制动作。
 * ========================================================================== */

/**
 * @brief 初始化 TFmini 接收和解析流程。
 *
 * @return 返回 TFmini 设备状态。
 */
tfmini_status_t tfmini_init(void);

/**
 * @brief 判断 TFmini 是否已经完成初始化。
 *
 * @return 已初始化返回 true，否则返回 false。
 */
bool tfmini_ready(void);

/**
 * @brief 判断 TFmini 是否在线。
 *
 * @param timeout_ms 离线超时时间，单位 ms。
 * @return 超时时间内收到过有效帧返回 true，否则返回 false。
 */
bool tfmini_online(uint32_t timeout_ms);

/**
 * @brief 读取一帧新的 TFmini 测量值。
 *
 * @param out 测量值输出。
 * @param io_seq 调用方保存的序号，读取成功后会更新为最新序号。
 * @return 有新数据返回 true，否则返回 false。
 */
bool tfmini_read(tfmini_meas_t *out, uint32_t *io_seq);

/**
 * @brief 获取最近一次 TFmini 测量值。
 *
 * @param out 测量值输出。
 * @return 已有测量值返回 true，否则返回 false。
 */
bool tfmini_peek(tfmini_meas_t *out);

/**
 * @brief 重置 TFmini 帧解析状态。
 */
void tfmini_reset(void);

/**
 * @brief 发送 TFmini 原始字节。
 *
 * @param data 发送数据。
 * @param length 发送长度，单位 byte。
 * @return 返回 TFmini 设备状态。
 */
tfmini_status_t tfmini_raw(const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_DEVICE_TFMINI_H

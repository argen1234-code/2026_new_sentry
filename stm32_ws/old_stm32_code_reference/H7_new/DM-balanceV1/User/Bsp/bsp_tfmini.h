/**
  ****************************(C) COPYRIGHT 2025****************************
  * @file       bsp_tfmini.h
  * @brief      TFmini Plus BSP driver (UART/IIC).
  *             TFmini Plus BSP驱动（UART/IIC）
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-21-2025     Codex           1. created
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025****************************
  */
#ifndef BSP_TFMINI_H
#define BSP_TFMINI_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= 配置项 / Config ========================= */
#ifndef TFMINI_IIC_ENABLE
#define TFMINI_IIC_ENABLE 0   /* 1=启用软件IIC接口, 0=关闭 */
#endif

#ifndef TFMINI_IIC_ADDR
#define TFMINI_IIC_ADDR 0x10u /* 默认7位地址, 需根据手册确认 */
#endif

#ifndef TFMINI_USE_TIM24_TIME
#define TFMINI_USE_TIM24_TIME 1 /* 1=use TIM24 timebase, 0=disable */
#endif

#ifndef TFMINI_USE_DWT_TIME
#define TFMINI_USE_DWT_TIME 0 /* 1=use DWT timebase, 0=disable */
#endif

/* ========================= 常量定义 / Constants ========================= */
#define TFMINI_FRAME_HEAD 0x59u
#define TFMINI_FRAME_LEN  9u

/* ========================= 数据结构 / Data Types ========================= */
typedef struct {
    uint16_t distance_cm; /**< 距离值, 单位: cm (默认输出单位) */
    uint16_t strength;    /**< 信号强度 */
    int16_t  temp_cdeg;   /**< 温度值, 单位: 0.01℃ */
    uint8_t  valid;       /**< 数据有效标志(1=有效,0=无效) */
    uint64_t ts_us;       /**< 时间戳(微秒) */
} tfmini_measurement_t;

/* ========================= UART 接口 / UART API ========================= */
/**
  * @brief          TFmini UART初始化
  * @param[in]      huart: UART句柄（建议USART10）
  * @retval         none
  */
/**
  * @brief          TFmini UART init
  * @param[in]      huart: UART handle (USART10 recommended)
  * @retval         none
  */
void tfmini_uart_init(UART_HandleTypeDef *huart);

/**
  * @brief          UART接收完成回调(请在HAL回调中调用)
  * @param[in]      huart: UART句柄
  * @retval         none
  */
/**
  * @brief          UART Rx complete callback (call it in HAL callback)
  * @param[in]      huart: UART handle
  * @retval         none
  */
void tfmini_uart_rx_cplt_callback(UART_HandleTypeDef *huart);

/**
  * @brief          UART错误回调(请在HAL错误回调中调用)
  * @param[in]      huart: UART句柄
  * @retval         none
  */
/**
  * @brief          UART error callback (call it in HAL error callback)
  * @param[in]      huart: UART handle
  * @retval         none
  */
void tfmini_uart_error_callback(UART_HandleTypeDef *huart);

/**
  * @brief          读取最新测距数据(版本号机制)
  * @param[out]     out: 输出测距数据
  * @param[in,out]  io_seq: 输入上次序号, 输出最新序号
  * @retval         true=有新数据, false=无新数据或参数无效
  */
/**
  * @brief          Read latest measurement with sequence check
  * @param[out]     out: output measurement
  * @param[in,out]  io_seq: input last seq, output new seq
  * @retval         true=new data, false=no data or invalid
  */
bool tfmini_read(tfmini_measurement_t *out, uint32_t *io_seq);

/**
  * @brief          获取最近一次测距数据(无新旧判断)
  * @param[out]     out: 输出测距数据
  * @retval         true=有可用数据, false=暂无数据
  */
/**
  * @brief          Peek last measurement (no sequence check)
  * @param[out]     out: output measurement
  * @retval         true=available, false=not ready
  */
bool tfmini_peek(tfmini_measurement_t *out);

/**
  * @brief          清空解析器状态
  * @retval         none
  */
/**
  * @brief          Reset parser state
  * @retval         none
  */
void tfmini_reset_parser(void);

/**
  * @brief          UART发送原始命令或数据
  * @param[in]      data: 数据缓冲区
  * @param[in]      len: 数据长度
  * @retval         true=成功, false=失败
  */
/**
  * @brief          Send raw bytes via UART
  * @param[in]      data: data buffer
  * @param[in]      len: length
  * @retval         true=ok, false=failed
  */
bool tfmini_uart_send_raw(const uint8_t *data, uint16_t len);

#if (TFMINI_IIC_ENABLE == 1)
/* ========================= 软件IIC接口 / Software IIC API ========================= */
/**
  * @brief          软件IIC初始化(需提前配置GPIO为开漏输出+上拉)
  * @retval         true=成功, false=失败
  */
/**
  * @brief          Software IIC init (GPIO should be open-drain + pull-up)
  * @retval         true=ok, false=failed
  */
bool tfmini_iic_init(void);

/**
  * @brief          通过IIC读取测距数据(9字节标准帧)
  * @param[out]     out: 输出测距数据
  * @retval         true=成功, false=失败
  */
/**
  * @brief          Read measurement via IIC (9-byte standard frame)
  * @param[out]     out: output measurement
  * @retval         true=ok, false=failed
  */
bool tfmini_iic_read_measurement(tfmini_measurement_t *out);
#endif

#ifdef __cplusplus
}
#endif

#endif /* BSP_TFMINI_H */

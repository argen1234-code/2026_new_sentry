/**
 ******************************************************************************
 * @file    bsp_adc.h
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   ADC 控制器 BSP 抽象接口。
 *
 * @note    本文件只提供 ADC 句柄获取、HAL ADC 操作包装和 HAL ADC
 *          回调分发。
 * @note    板级通道用途、数值换算、滤波和业务状态应放在设备驱动或
 *          应用模块，不应放在 ADC BSP。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef PRINTK_BSP_ADC_H
#define PRINTK_BSP_ADC_H

#include "adc.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * BSP 头文件描述板级外设能力、ID 和轻量配置，不描述设备用途或机器人装配语义。
 * ========================================================================== */

/**
 * @brief 板上可由 BSP 管理的 ADC 标识。
 *
 * 新增或删除 ADC 实例时，需要同步维护本枚举和 bsp_adc.c 中的 bsp_adc_slots[]。
 */
typedef enum
{
    BSP_ADC_ID_ADC1 = 0,
    BSP_ADC_ID_ADC3,
    BSP_ADC_ID_COUNT
} bsp_adc_id_t;

/**
 * @brief ADC 启停模式。
 */
typedef enum
{
    BSP_ADC_MODE_POLL = 0,    /**< 普通阻塞/轮询转换。 */
    BSP_ADC_MODE_IT,          /**< 中断转换。 */
    BSP_ADC_MODE_DMA          /**< DMA 转换。 */
} bsp_adc_mode_t;

/**
 * @brief ADC 事件类型。
 */
typedef enum
{
    BSP_ADC_EVENT_CONV_CPLT = 0,
    BSP_ADC_EVENT_CONV_HALF,
    BSP_ADC_EVENT_ERROR,
    BSP_ADC_EVENT_LEVEL_OUT_OF_WINDOW,
    BSP_ADC_EVENT_END_OF_SAMPLING,
    BSP_ADC_EVENT_COUNT
} bsp_adc_event_t;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按初始化、查询、收发和回调辅助排列，上层不应直接访问 CubeMX 句柄。
 * ========================================================================== */

/**
 * @brief ADC 事件回调函数类型。
 *
 * @param hadc 触发事件的 ADC 句柄。
 * @param user 注册回调时传入的用户上下文。
 */
typedef void (*bsp_adc_event_callback_t)(ADC_HandleTypeDef *hadc, void *user);

/**
 * @brief ADC 启动事务。
 */
typedef struct
{
    bsp_adc_mode_t mode;    /**< 启动模式。 */
    uint32_t *data;         /**< DMA 目标缓冲区；非 DMA 模式忽略。 */
    uint32_t length;        /**< DMA 传输长度；非 DMA 模式忽略。 */
} bsp_adc_start_t;

/**
 * @brief 初始化 ADC BSP 回调表。
 *
 * @note 差异: 新版 ADC BSP 只清空回调注册状态，不启动转换，也不维护采样缓冲。
 */
void bsp_adc_init(void);

/**
 * @brief 根据 BSP ADC 标识获取 ADC 句柄。
 *
 * @param id BSP ADC 标识。
 * @return 匹配成功返回对应句柄，否则返回 `NULL`。
 */
ADC_HandleTypeDef *bsp_adc_handle(bsp_adc_id_t id);

/**
 * @brief 根据外设实例地址获取 ADC 句柄。
 *
 * @param instance ADC 外设实例，例如 `ADC1`、`ADC3`。
 * @return 匹配成功返回对应句柄，否则返回 `NULL`。
 */
ADC_HandleTypeDef *bsp_adc_from_instance(ADC_TypeDef *instance);

/**
 * @brief 判断 ADC 句柄是否由本 BSP 管理。
 *
 * @param hadc 待检查的 ADC 句柄。
 * @return 受管理返回 1，否则返回 0。
 */
uint8_t bsp_adc_is_supported(ADC_HandleTypeDef *hadc);

/**
 * @brief 启动 ADC 自校准。
 *
 * @param hadc 目标 ADC 句柄。
 * @param calibration_mode HAL 定义的校准模式。
 * @param single_diff HAL 定义的单端/差分模式。
 * @return 返回 HAL ADC 状态码。
 */
HAL_StatusTypeDef bsp_adc_calibration_start(ADC_HandleTypeDef *hadc,
                                            uint32_t calibration_mode,
                                            uint32_t single_diff);

/**
 * @brief 启动 ADC 普通转换。
 *
 * @param hadc 目标 ADC 句柄。
 * @param start ADC 启动事务配置。
 * @return 返回 HAL ADC 状态码。
 */
HAL_StatusTypeDef bsp_adc_start(ADC_HandleTypeDef *hadc, const bsp_adc_start_t *start);

/**
 * @brief 停止 ADC 普通转换。
 *
 * @param hadc 目标 ADC 句柄。
 * @param mode 需要停止的 ADC 模式。
 * @return 返回 HAL ADC 状态码。
 */
HAL_StatusTypeDef bsp_adc_stop(ADC_HandleTypeDef *hadc, bsp_adc_mode_t mode);

/**
 * @brief 等待 ADC 普通转换完成。
 *
 * @param hadc 目标 ADC 句柄。
 * @param timeout 超时时间，单位为毫秒。
 * @return 返回 HAL ADC 状态码。
 */
HAL_StatusTypeDef bsp_adc_poll_for_conversion(ADC_HandleTypeDef *hadc, uint32_t timeout);

/**
 * @brief 获取 ADC 当前转换值。
 *
 * @param hadc 目标 ADC 句柄。
 * @return 当前转换值；入参非法时返回 0。
 */
uint32_t bsp_adc_get_value(ADC_HandleTypeDef *hadc);

/**
 * @brief 配置 ADC 普通通道。
 *
 * @param hadc 目标 ADC 句柄。
 * @param config HAL ADC 通道配置。
 * @return 返回 HAL ADC 状态码。
 */
HAL_StatusTypeDef bsp_adc_config_channel(ADC_HandleTypeDef *hadc,
                                         const ADC_ChannelConfTypeDef *config);

/**
 * @brief 绑定 ADC 事件回调。
 *
 * @param hadc 目标 ADC 句柄。
 * @param event ADC 事件类型。
 * @param callback 事件回调，传入 `NULL` 表示取消绑定。
 * @param user 用户上下文。
 * @return 返回 HAL ADC 状态码。
 */
HAL_StatusTypeDef bsp_adc_on(ADC_HandleTypeDef *hadc,
                             bsp_adc_event_t event,
                             bsp_adc_event_callback_t callback,
                             void *user);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_BSP_ADC_H

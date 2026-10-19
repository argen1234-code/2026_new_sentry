/**
 ******************************************************************************
 * @file    bsp_dts.h
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   DTS 控制器 BSP 抽象接口。
 * @note    差异: 本文件只封装 DTS 控制器句柄、HAL 操作包装和事件分发；
 *          不维护温度业务缓存，也不绑定具体热管理策略。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef PRINTK_BSP_DTS_H
#define PRINTK_BSP_DTS_H

#include "dts.h"

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
 * @brief 板上可由 BSP 管理的 DTS 标识。
 *
 * 新增或删除 DTS 实例时，需要同步维护本枚举和 bsp_dts.c 中的 bsp_dts_slots[]。
 */
typedef enum
{
    BSP_DTS_ID_DTS = 0,
    BSP_DTS_ID_COUNT
} bsp_dts_id_t;

/**
 * @brief DTS 事件类型。
 */
typedef enum
{
    BSP_DTS_EVENT_END = 0,       /**< 同步转换结束事件。 */
    BSP_DTS_EVENT_LOW,           /**< 同步低温阈值事件。 */
    BSP_DTS_EVENT_HIGH,          /**< 同步高温阈值事件。 */
    BSP_DTS_EVENT_ASYNC_END,     /**< 异步转换结束事件。 */
    BSP_DTS_EVENT_ASYNC_LOW,     /**< 异步低温阈值事件。 */
    BSP_DTS_EVENT_ASYNC_HIGH,    /**< 异步高温阈值事件。 */
    BSP_DTS_EVENT_COUNT
} bsp_dts_event_t;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按初始化、查询、收发和回调辅助排列，上层不应直接访问 CubeMX 句柄。
 * ========================================================================== */

/**
 * @brief DTS 事件回调类型。
 * @param hdts 触发事件的 HAL DTS 句柄。
 * @param user 用户上下文。
 */
typedef void (*bsp_dts_event_callback_t)(DTS_HandleTypeDef *hdts, void *user);

/**
 * @brief 初始化 DTS BSP 内部状态。
 * @note 该函数不替代 CubeMX 生成的 MX_DTS_Init()。
 */
void bsp_dts_init(void);

/**
 * @brief 获取指定 DTS 句柄。
 * @param id DTS 标识。
 * @return 匹配的 HAL DTS 句柄；未找到时返回 NULL。
 */
DTS_HandleTypeDef *bsp_dts_handle(bsp_dts_id_t id);

/**
 * @brief 根据 DTS 外设实例反查 HAL DTS 句柄。
 * @param instance DTS 外设实例地址。
 * @return 匹配的 HAL DTS 句柄；未找到时返回 NULL。
 */
DTS_HandleTypeDef *bsp_dts_from_instance(DTS_TypeDef *instance);

/**
 * @brief 判断 DTS 句柄是否由本 BSP 管理。
 * @param hdts HAL DTS 句柄。
 * @return 1 表示受管；0 表示不受管。
 */
uint8_t bsp_dts_is_supported(DTS_HandleTypeDef *hdts);

/**
 * @brief 启动 DTS 同步测量。
 * @param hdts HAL DTS 句柄。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_dts_start(DTS_HandleTypeDef *hdts);

/**
 * @brief 停止 DTS 同步测量。
 * @param hdts HAL DTS 句柄。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_dts_stop(DTS_HandleTypeDef *hdts);

/**
 * @brief 读取 DTS 原始温度结果。
 * @param hdts HAL DTS 句柄。
 * @param temperature 输出温度值指针。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_dts_get_temperature(DTS_HandleTypeDef *hdts, int32_t *temperature);

/**
 * @brief 以中断方式启动 DTS 测量。
 * @param hdts HAL DTS 句柄。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_dts_start_it(DTS_HandleTypeDef *hdts);

/**
 * @brief 停止 DTS 中断测量。
 * @param hdts HAL DTS 句柄。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_dts_stop_it(DTS_HandleTypeDef *hdts);

/**
 * @brief 转发 DTS 中断处理。
 * @param hdts HAL DTS 句柄。
 */
void bsp_dts_irq_handler(DTS_HandleTypeDef *hdts);

/**
 * @brief 获取 DTS HAL 状态。
 * @param hdts HAL DTS 句柄。
 * @return HAL DTS 状态；句柄非法时返回 HAL_DTS_STATE_RESET。
 */
HAL_DTS_StateTypeDef bsp_dts_get_state(DTS_HandleTypeDef *hdts);

/**
 * @brief 绑定 DTS 事件回调。
 * @param hdts HAL DTS 句柄。
 * @param event DTS 事件类型。
 * @param callback 事件回调；传入 NULL 可清空该事件绑定。
 * @param user 用户上下文。
 * @return HAL_OK 表示绑定成功；HAL_ERROR 表示参数非法或句柄不受管。
 */
HAL_StatusTypeDef bsp_dts_on(DTS_HandleTypeDef *hdts,
                             bsp_dts_event_t event,
                             bsp_dts_event_callback_t callback,
                             void *user);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_BSP_DTS_H

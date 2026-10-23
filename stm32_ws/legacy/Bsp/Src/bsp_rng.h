/**
 ******************************************************************************
 * @file    bsp_rng.h
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   RNG 控制器 BSP 抽象接口。
 * @note    差异: 本文件只封装 RNG 句柄、HAL 操作包装、状态读取和事件分发；
 *          不实现随机数池、业务级 nonce 管理或协议安全策略。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef PRINTK_BSP_RNG_H
#define PRINTK_BSP_RNG_H

#include "rng.h"

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
 * @brief 板上可由 BSP 管理的 RNG 标识。
 *
 * 新增或删除 RNG 实例时，需要同步维护本枚举和 bsp_rng.c 中的 bsp_rng_slots[]。
 */
typedef enum
{
    BSP_RNG_ID_RNG = 0,
    BSP_RNG_ID_COUNT
} bsp_rng_id_t;

/**
 * @brief RNG 事件类型。
 */
typedef enum
{
    BSP_RNG_EVENT_READY = 0,    /**< 随机数就绪事件。 */
    BSP_RNG_EVENT_ERROR,        /**< RNG 错误事件。 */
    BSP_RNG_EVENT_COUNT
} bsp_rng_event_t;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按初始化、查询、收发和回调辅助排列，上层不应直接访问 CubeMX 句柄。
 * ========================================================================== */

/**
 * @brief RNG 普通事件回调类型。
 * @param hrng 触发事件的 HAL RNG 句柄。
 * @param user 用户上下文。
 */
typedef void (*bsp_rng_event_callback_t)(RNG_HandleTypeDef *hrng, void *user);

/**
 * @brief RNG 随机数就绪事件回调类型。
 * @param hrng 触发事件的 HAL RNG 句柄。
 * @param random32bit HAL 回调传入的 32 位随机数。
 * @param user 用户上下文。
 */
typedef void (*bsp_rng_ready_callback_t)(RNG_HandleTypeDef *hrng, uint32_t random32bit, void *user);

/**
 * @brief 初始化 RNG BSP 内部状态。
 * @note 该函数不替代 CubeMX 生成的 MX_RNG_Init()。
 */
void bsp_rng_init(void);

/**
 * @brief 获取指定 RNG 句柄。
 * @param id RNG 标识。
 * @return 匹配的 HAL RNG 句柄；未找到时返回 NULL。
 */
RNG_HandleTypeDef *bsp_rng_handle(bsp_rng_id_t id);

/**
 * @brief 根据 RNG 外设实例反查 HAL RNG 句柄。
 * @param instance RNG 外设实例地址。
 * @return 匹配的 HAL RNG 句柄；未找到时返回 NULL。
 */
RNG_HandleTypeDef *bsp_rng_from_instance(RNG_TypeDef *instance);

/**
 * @brief 判断 RNG 句柄是否由本 BSP 管理。
 * @param hrng HAL RNG 句柄。
 * @return 1 表示受管；0 表示不受管。
 */
uint8_t bsp_rng_is_supported(RNG_HandleTypeDef *hrng);

/**
 * @brief 轮询生成一个 32 位随机数。
 * @param hrng HAL RNG 句柄。
 * @param random32bit 输出随机数指针。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_rng_generate(RNG_HandleTypeDef *hrng, uint32_t *random32bit);

/**
 * @brief 以中断方式启动随机数生成。
 * @param hrng HAL RNG 句柄。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_rng_generate_it(RNG_HandleTypeDef *hrng);

/**
 * @brief 读取最近一次 RNG 数据寄存器值。
 * @param hrng HAL RNG 句柄。
 * @return 最近一次随机数；句柄非法时返回 0。
 */
uint32_t bsp_rng_read_last(RNG_HandleTypeDef *hrng);

/**
 * @brief 转发 RNG 中断处理。
 * @param hrng HAL RNG 句柄。
 */
void bsp_rng_irq_handler(RNG_HandleTypeDef *hrng);

/**
 * @brief 获取 RNG HAL 状态。
 * @param hrng HAL RNG 句柄。
 * @return HAL RNG 状态；句柄非法时返回 HAL_RNG_STATE_RESET。
 */
HAL_RNG_StateTypeDef bsp_rng_get_state(RNG_HandleTypeDef *hrng);

/**
 * @brief 获取 RNG 错误码。
 * @param hrng HAL RNG 句柄。
 * @return HAL RNG 错误码；句柄非法时返回 0。
 */
uint32_t bsp_rng_get_error(RNG_HandleTypeDef *hrng);

/**
 * @brief 尝试恢复 RNG seed error。
 * @param hrng HAL RNG 句柄。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_rng_recover_seed_error(RNG_HandleTypeDef *hrng);

/**
 * @brief 设置 RNG 配置。
 * @param hrng HAL RNG 句柄。
 * @param config RNG 配置指针。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_rng_set_config(RNG_HandleTypeDef *hrng, const RNG_ConfigTypeDef *config);

/**
 * @brief 获取 RNG 配置。
 * @param hrng HAL RNG 句柄。
 * @param config RNG 配置输出指针。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_rng_get_config(RNG_HandleTypeDef *hrng, RNG_ConfigTypeDef *config);

/**
 * @brief 锁定 RNG 配置。
 * @param hrng HAL RNG 句柄。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_rng_lock_config(RNG_HandleTypeDef *hrng);

/**
 * @brief 绑定 RNG 事件回调。
 * @param hrng HAL RNG 句柄。
 * @param event RNG 事件类型。
 * @param callback 事件回调；READY 事件使用 bsp_rng_ready_callback_t，其它事件使用 bsp_rng_event_callback_t。
 * @param user 用户上下文。
 * @return HAL_OK 表示绑定成功；HAL_ERROR 表示参数非法或句柄不受管。
 */
HAL_StatusTypeDef bsp_rng_on(RNG_HandleTypeDef *hrng, bsp_rng_event_t event, void *callback, void *user);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_BSP_RNG_H

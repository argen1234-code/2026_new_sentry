/**
 ******************************************************************************
 * @file    bsp_crc.h
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   CRC 控制器 BSP 抽象接口。
 * @note    差异: 本文件只提供 CRC 句柄获取、HAL CRC 操作包装和状态读取；
 *          协议帧校验、查表兼容、字段拼包和业务完整性策略应放在协议模块。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef PRINTK_BSP_CRC_H
#define PRINTK_BSP_CRC_H

#include "crc.h"

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
 * @brief 板上可由 BSP 管理的 CRC 标识。
 *
 * 新增或删除 CRC 实例时，需要同步维护本枚举和 bsp_crc.c 中的 bsp_crc_slots[]。
 */
typedef enum
{
    BSP_CRC_ID_CRC = 0,
    BSP_CRC_ID_COUNT
} bsp_crc_id_t;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按初始化、查询、收发和回调辅助排列，上层不应直接访问 CubeMX 句柄。
 * ========================================================================== */

/**
 * @brief 初始化 CRC BSP 内部状态。
 * @note 该函数不替代 CubeMX 生成的 MX_CRC_Init()。
 */
void bsp_crc_init(void);

/**
 * @brief 获取指定 CRC 句柄。
 * @param id CRC 标识。
 * @return 匹配的 HAL CRC 句柄；未找到时返回 NULL。
 */
CRC_HandleTypeDef *bsp_crc_handle(bsp_crc_id_t id);

/**
 * @brief 根据 CRC 外设实例反查 HAL CRC 句柄。
 * @param instance CRC 外设实例地址。
 * @return 匹配的 HAL CRC 句柄；未找到时返回 NULL。
 */
CRC_HandleTypeDef *bsp_crc_from_instance(CRC_TypeDef *instance);

/**
 * @brief 判断 CRC 句柄是否由本 BSP 管理。
 * @param hcrc HAL CRC 句柄。
 * @return 1 表示受管；0 表示不受管。
 */
uint8_t bsp_crc_is_supported(CRC_HandleTypeDef *hcrc);

/**
 * @brief 计算一段 32 位数据的 CRC。
 * @param hcrc HAL CRC 句柄。
 * @param data 输入数据指针。
 * @param length 输入数据长度，单位为 32 bit word。
 * @return CRC 结果；参数非法时返回 0。
 */
uint32_t bsp_crc_calculate(CRC_HandleTypeDef *hcrc, const uint32_t *data, uint32_t length);

/**
 * @brief 在当前 CRC 状态上继续累加计算。
 * @param hcrc HAL CRC 句柄。
 * @param data 输入数据指针。
 * @param length 输入数据长度，单位为 32 bit word。
 * @return CRC 结果；参数非法时返回 0。
 */
uint32_t bsp_crc_accumulate(CRC_HandleTypeDef *hcrc, const uint32_t *data, uint32_t length);

/**
 * @brief 复位 CRC 计算单元。
 * @param hcrc HAL CRC 句柄。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_crc_reset(CRC_HandleTypeDef *hcrc);

/**
 * @brief 设置 CRC 初始值。
 * @param hcrc HAL CRC 句柄。
 * @param init_value CRC 初始值。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_crc_set_initial_value(CRC_HandleTypeDef *hcrc, uint32_t init_value);

/**
 * @brief 设置 CRC 独立数据寄存器。
 * @param hcrc HAL CRC 句柄。
 * @param value 独立数据寄存器值。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_crc_set_independent_data(CRC_HandleTypeDef *hcrc, uint8_t value);

/**
 * @brief 读取 CRC 独立数据寄存器。
 * @param hcrc HAL CRC 句柄。
 * @return 独立数据寄存器值；句柄非法时返回 0。
 */
uint8_t bsp_crc_get_independent_data(CRC_HandleTypeDef *hcrc);

/**
 * @brief 设置 CRC 多项式。
 * @param hcrc HAL CRC 句柄。
 * @param polynomial CRC 多项式。
 * @param polynomial_length CRC 多项式长度配置。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_crc_set_polynomial(CRC_HandleTypeDef *hcrc,
                                         uint32_t polynomial,
                                         uint32_t polynomial_length);

/**
 * @brief 设置 CRC 输入数据反转模式。
 * @param hcrc HAL CRC 句柄。
 * @param reverse_mode 输入反转模式。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_crc_set_input_reverse(CRC_HandleTypeDef *hcrc, uint32_t reverse_mode);

/**
 * @brief 设置 CRC 输出数据反转模式。
 * @param hcrc HAL CRC 句柄。
 * @param reverse_mode 输出反转模式。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_crc_set_output_reverse(CRC_HandleTypeDef *hcrc, uint32_t reverse_mode);

/**
 * @brief 获取 CRC HAL 状态。
 * @param hcrc HAL CRC 句柄。
 * @return HAL CRC 状态；句柄非法时返回 HAL_CRC_STATE_RESET。
 */
HAL_CRC_StateTypeDef bsp_crc_get_state(CRC_HandleTypeDef *hcrc);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_BSP_CRC_H

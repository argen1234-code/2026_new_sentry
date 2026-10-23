/**
 ******************************************************************************
 * @file    bsp_crc.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-15
 * @brief   CRC BSP 的 C++ 编译期包装。
 *
 * @note    本文件只做 C++ 包装和模板绑定，运行时实现全部位于 Src/bsp_crc.c。
 * @note    C 文件应包含 Src/bsp_crc.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_BSP_CRC_HPP
#define PRINTK_BSP_CRC_HPP

#include "Src/bsp_crc.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只为 C BSP 接口提供命名空间内 inline 转发，不引入运行时抽象。
 * ========================================================================== */
namespace Bsp::Crc {

/**
 * @brief CRC 标识。
 */
enum class Id {
    Crc = BSP_CRC_ID_CRC,    /**< CRC。 */
};

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 初始化 CRC BSP。 */
inline void init()
{
    bsp_crc_init();
}

/** @brief 获取 CRC HAL 句柄。 */
inline CRC_HandleTypeDef *handle(Id id)
{
    return bsp_crc_handle(static_cast<bsp_crc_id_t>(id));
}

/** @brief 根据外设实例反查 CRC HAL 句柄。 */
inline CRC_HandleTypeDef *inst(CRC_TypeDef *instance)
{
    return bsp_crc_from_instance(instance);
}

/** @brief 判断 CRC 句柄是否由 BSP 管理。 */
inline bool supported(CRC_HandleTypeDef *hcrc)
{
    return bsp_crc_is_supported(hcrc) != 0U;
}

/** @brief 计算 CRC。 */
inline uint32_t calc(CRC_HandleTypeDef *hcrc, const uint32_t *data, uint32_t length)
{
    return bsp_crc_calculate(hcrc, data, length);
}

/** @brief 累加计算 CRC。 */
inline uint32_t accum(CRC_HandleTypeDef *hcrc, const uint32_t *data, uint32_t length)
{
    return bsp_crc_accumulate(hcrc, data, length);
}

/** @brief 复位 CRC 计算单元。 */
inline HAL_StatusTypeDef reset(CRC_HandleTypeDef *hcrc)
{
    return bsp_crc_reset(hcrc);
}

/** @brief 设置 CRC 初始值。 */
inline HAL_StatusTypeDef seed(CRC_HandleTypeDef *hcrc, uint32_t value)
{
    return bsp_crc_set_initial_value(hcrc, value);
}

/** @brief 设置 CRC 独立数据寄存器。 */
inline HAL_StatusTypeDef ind(CRC_HandleTypeDef *hcrc, uint8_t value)
{
    return bsp_crc_set_independent_data(hcrc, value);
}

/** @brief 读取 CRC 独立数据寄存器。 */
inline uint8_t ind(CRC_HandleTypeDef *hcrc)
{
    return bsp_crc_get_independent_data(hcrc);
}

/** @brief 设置 CRC 多项式。 */
inline HAL_StatusTypeDef poly(CRC_HandleTypeDef *hcrc, uint32_t polynomial, uint32_t length)
{
    return bsp_crc_set_polynomial(hcrc, polynomial, length);
}

/** @brief 设置 CRC 输入数据反转模式。 */
inline HAL_StatusTypeDef in_rev(CRC_HandleTypeDef *hcrc, uint32_t mode)
{
    return bsp_crc_set_input_reverse(hcrc, mode);
}

/** @brief 设置 CRC 输出数据反转模式。 */
inline HAL_StatusTypeDef out_rev(CRC_HandleTypeDef *hcrc, uint32_t mode)
{
    return bsp_crc_set_output_reverse(hcrc, mode);
}

/** @brief 获取 CRC HAL 状态。 */
inline HAL_CRC_StateTypeDef state(CRC_HandleTypeDef *hcrc)
{
    return bsp_crc_get_state(hcrc);
}

/**
 * @brief CRC 编译期端口。
 *
 * @tparam Unit CRC 标识。
 */
template<Id Unit>
struct Port;

/** @brief CRC 编译期端口。 */
template<>
struct Port<Id::Crc> {
    /** @brief 获取 CRC HAL 句柄。 */
    static CRC_HandleTypeDef *handle()
    {
        return Bsp::Crc::handle(Id::Crc);
    }
};

/** @brief 通过编译期端口计算 CRC。 */
template<Id Unit>
inline uint32_t calc(const uint32_t *data, uint32_t length)
{
    return calc(Port<Unit>::handle(), data, length);
}

/** @brief 通过编译期端口累加计算 CRC。 */
template<Id Unit>
inline uint32_t accum(const uint32_t *data, uint32_t length)
{
    return accum(Port<Unit>::handle(), data, length);
}

/** @brief 通过编译期端口复位 CRC。 */
template<Id Unit>
inline HAL_StatusTypeDef reset()
{
    return reset(Port<Unit>::handle());
}

} // namespace Bsp::Crc

#endif // PRINTK_BSP_CRC_HPP

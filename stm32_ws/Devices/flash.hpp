/**
 ******************************************************************************
 * @file    flash.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   外部 Flash 设备 C++ 包装。
 *
 * @note    本文件只做 C++ 薄包装，运行时实现全部位于 Src/flash.c。
 * @note    C 文件应包含 Src/flash.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_FLASH_HPP
#define PRINTK_DEVICE_FLASH_HPP

#include "Src/flash.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只包装设备层 C API，便于 C++ 调用侧保持零成本访问。
 * ========================================================================== */
namespace Device::Flash {

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief Flash 起始地址。 */
inline constexpr uint32_t Base = FLASH_DEVICE_BASE;

/** @brief Flash 总容量。 */
inline constexpr uint32_t Size = FLASH_DEVICE_SIZE;

/** @brief Flash 页大小。 */
inline constexpr uint32_t PageSize = FLASH_DEVICE_PAGE_SIZE;

/** @brief Flash 擦除粒度。 */
inline constexpr uint32_t EraseSize = FLASH_DEVICE_ERASE_SIZE;

/** @brief Flash 结束地址。 */
inline constexpr uint32_t End = FLASH_DEVICE_END;

/** @brief Flash 状态。 */
using Status = flash_status_t;

/** @brief Flash 几何信息。 */
using Info = flash_info_t;

/** @brief 初始化并探测 Flash。 */
inline Status init()
{
    return flash_init();
}

/** @brief 获取 Flash 几何信息。 */
inline const Info *info()
{
    return flash_info();
}

/** @brief 读取 Flash 数据。 */
inline Status read(uint32_t address, void *buffer, uint32_t length)
{
    return flash_read(address, buffer, length);
}

/** @brief 编程 Flash 数据。 */
inline Status prog(uint32_t address, const void *buffer, uint32_t length)
{
    return flash_prog(address, buffer, length);
}

/** @brief 擦除 Flash 区域。 */
inline Status erase(uint32_t address, uint32_t length)
{
    return flash_erase(address, length);
}

/** @brief 擦除覆盖区域后写入数据。 */
inline Status write(uint32_t address, const void *buffer, uint32_t length)
{
    return flash_write(address, buffer, length);
}

/** @brief 获取地址所在扇区编号。 */
inline uint32_t sector(uint32_t address)
{
    return flash_sector(address);
}

/** @brief 获取下一个扇区边界。 */
inline uint32_t next_sector(uint32_t address)
{
    return flash_next_sector(address);
}

} // namespace Device::Flash

#endif // PRINTK_DEVICE_FLASH_HPP

/**
 ******************************************************************************
 * @file    flash_w25q64.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   W25Q64 外部 Flash 实现接口。
 *
 * @note    本文件是 `flash.c` 使用的型号实现接口，不建议上层业务直接包含。
 * @note    公共 Flash 访问约定见 `flash.h`，C++ 包装见 `flash.hpp`。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_FLASH_W25Q64_H
#define PRINTK_DEVICE_FLASH_W25Q64_H

#include "flash.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口只表达设备能做什么，不把遥控器、裁判系统或电机解释成具体机器人控制动作。
 * ========================================================================== */

/**
 * @brief 初始化并探测 W25Q64。
 *
 * @return 返回 Flash 状态码。
 */
flash_status_t flash_w25q64_init(void);

/**
 * @brief 获取 W25Q64 几何信息。
 *
 * @return 返回只读 Flash 信息指针。
 */
const flash_info_t *flash_w25q64_info(void);

/**
 * @brief 从 W25Q64 读取 byte 数据。
 *
 * @param address 器件内部起始地址。
 * @param buffer 读取输出缓冲区。
 * @param length 读取长度，单位为 byte。
 * @return 返回 Flash 状态码。
 */
flash_status_t flash_w25q64_read(uint32_t address, void *buffer, uint32_t length);

/**
 * @brief 向 W25Q64 编程 byte 数据。
 *
 * 本接口只执行 page program，不会擦除目标区域；调用方必须保证目标区域已经擦除。
 *
 * @param address 器件内部起始地址。
 * @param buffer 待写入数据。
 * @param length 写入长度，单位为 byte。
 * @return 返回 Flash 状态码。
 */
flash_status_t flash_w25q64_prog(uint32_t address, const void *buffer, uint32_t length);

/**
 * @brief 按 4KB 扇区擦除 W25Q64。
 *
 * address 和 length 必须按 FLASH_DEVICE_ERASE_SIZE 对齐。
 *
 * @param address 器件内部起始地址。
 * @param length 擦除长度，单位为 byte。
 * @return 返回 Flash 状态码。
 */
flash_status_t flash_w25q64_erase(uint32_t address, uint32_t length);

/**
 * @brief 擦除覆盖目标范围的扇区后写入 byte 数据。
 *
 * @param address 器件内部起始地址。
 * @param buffer 待写入数据。
 * @param length 写入长度，单位为 byte。
 * @return 返回 Flash 状态码。
 */
flash_status_t flash_w25q64_write(uint32_t address, const void *buffer, uint32_t length);

/**
 * @brief 获取地址所在 4KB 扇区编号。
 *
 * @param address 器件内部地址。
 * @return 返回地址所在扇区编号。
 */
uint32_t flash_w25q64_sector(uint32_t address);

/**
 * @brief 获取地址所在扇区的下一个 4KB 扇区边界。
 *
 * @param address 器件内部地址。
 * @return 返回下一个扇区边界；越界时返回 FLASH_DEVICE_END。
 */
uint32_t flash_w25q64_next_sector(uint32_t address);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_DEVICE_FLASH_W25Q64_H

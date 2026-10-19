/**
 ******************************************************************************
 * @file    flash.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   外部 Flash 设备公共接口。
 *
 * @note    本文件定义当前工程对外部 Flash 的统一访问约定，具体芯片实现位于
 *          `flash_<model>.c/h`。
 * @note    当前板卡使用 W25Q64，公共接口由 `flash.c` 转发到 `flash_w25q64` 实现。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_FLASH_H
#define PRINTK_DEVICE_FLASH_H

#include "robot_board.h"

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
 * @brief 外部 Flash 状态码。
 */
typedef enum
{
    FLASH_OK = 0,              /**< 操作成功。 */
    FLASH_ERR = -1,            /**< 通用错误。 */
    FLASH_ERR_ARG = -2,        /**< 输入参数无效。 */
    FLASH_ERR_RANGE = -3,      /**< 地址范围越界。 */
    FLASH_ERR_ALIGN = -4,      /**< 地址或长度未满足对齐要求。 */
    FLASH_ERR_IO = -5,         /**< 底层总线或器件通信失败。 */
    FLASH_ERR_UNSUPPORTED = -6,/**< 当前实现不支持该操作。 */
    FLASH_ERR_NOT_READY = -7   /**< Flash 尚未初始化或未通过探测。 */
} flash_status_t;

/**
 * @brief 外部 Flash 几何信息。
 */
typedef struct
{
    uint32_t base;          /**< 器件内部起始地址。 */
    uint32_t size;          /**< 总容量，单位为 byte。 */
    uint32_t page_size;     /**< 页大小，单位为 byte。 */
    uint32_t erase_size;    /**< 最小擦除粒度，单位为 byte。 */
} flash_info_t;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口只表达设备能做什么，不把遥控器、裁判系统或电机解释成具体机器人控制动作。
 * ========================================================================== */

/**
 * @brief 初始化并探测外部 Flash。
 *
 * 本函数只校验外部 Flash 器件，不负责初始化 OSPI 外设；OSPI 仍由 CubeMX/HAL
 * 初始化流程完成。
 *
 * @return 返回 Flash 状态码。
 */
flash_status_t flash_init(void);

/**
 * @brief 获取外部 Flash 几何信息。
 *
 * @return 返回只读 Flash 信息指针。
 */
const flash_info_t *flash_info(void);

/**
 * @brief 从外部 Flash 读取 byte 数据。
 *
 * @param address 器件内部起始地址。
 * @param buffer 读取输出缓冲区。
 * @param length 读取长度，单位为 byte。
 * @return 返回 Flash 状态码。
 */
flash_status_t flash_read(uint32_t address, void *buffer, uint32_t length);

/**
 * @brief 向外部 Flash 编程 byte 数据。
 *
 * 本接口只执行 page program，不会擦除目标区域；调用方必须保证目标区域已经擦除。
 *
 * @param address 器件内部起始地址。
 * @param buffer 待写入数据。
 * @param length 写入长度，单位为 byte。
 * @return 返回 Flash 状态码。
 */
flash_status_t flash_prog(uint32_t address, const void *buffer, uint32_t length);

/**
 * @brief 按扇区擦除外部 Flash。
 *
 * address 和 length 必须按 FLASH_DEVICE_ERASE_SIZE 对齐，避免调用方误擦除相邻数据。
 *
 * @param address 器件内部起始地址。
 * @param length 擦除长度，单位为 byte。
 * @return 返回 Flash 状态码。
 */
flash_status_t flash_erase(uint32_t address, uint32_t length);

/**
 * @brief 擦除覆盖目标范围的扇区后写入 byte 数据。
 *
 * 这是危险操作：如果 address/length 未按扇区对齐，会擦除覆盖到的完整扇区，
 * 扇区内其他数据不会被保留。
 *
 * @param address 器件内部起始地址。
 * @param buffer 待写入数据。
 * @param length 写入长度，单位为 byte。
 * @return 返回 Flash 状态码。
 */
flash_status_t flash_write(uint32_t address, const void *buffer, uint32_t length);

/**
 * @brief 获取地址所在扇区编号。
 *
 * @param address 器件内部地址。
 * @return 返回地址所在扇区编号。
 */
uint32_t flash_sector(uint32_t address);

/**
 * @brief 获取地址所在扇区的下一个扇区边界。
 *
 * @param address 器件内部地址。
 * @return 返回下一个扇区边界；越界时返回 FLASH_DEVICE_END。
 */
uint32_t flash_next_sector(uint32_t address);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_DEVICE_FLASH_H

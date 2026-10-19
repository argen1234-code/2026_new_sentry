/**
 ******************************************************************************
 * @file    flash.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   外部 Flash 设备公共接口实现。
 *
 * @note    本层只负责把公共 Flash 接口绑定到当前板卡实际使用的 W25Q64 实现。
 * @note    芯片命令、OSPI 事务和容量校验细节保留在 `flash_w25q64.c` 内部。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "flash.h"
#include "flash_w25q64.h"

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置设备驱动的协议常量、默认参数和私有类型，硬件绑定来自 Robot board 配置。
 * ========================================================================== */

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只提供初始化、喂入、查询和发送入口，不保留旧 task/global 兼容接口。
 * ========================================================================== */
flash_status_t flash_init(void)
{
    return flash_w25q64_init();
}

const flash_info_t *flash_info(void)
{
    return flash_w25q64_info();
}

flash_status_t flash_read(uint32_t address, void *buffer, uint32_t length)
{
    return flash_w25q64_read(address, buffer, length);
}

flash_status_t flash_prog(uint32_t address, const void *buffer, uint32_t length)
{
    return flash_w25q64_prog(address, buffer, length);
}

flash_status_t flash_erase(uint32_t address, uint32_t length)
{
    return flash_w25q64_erase(address, length);
}

flash_status_t flash_write(uint32_t address, const void *buffer, uint32_t length)
{
    return flash_w25q64_write(address, buffer, length);
}

uint32_t flash_sector(uint32_t address)
{
    return flash_w25q64_sector(address);
}

uint32_t flash_next_sector(uint32_t address)
{
    return flash_w25q64_next_sector(address);
}

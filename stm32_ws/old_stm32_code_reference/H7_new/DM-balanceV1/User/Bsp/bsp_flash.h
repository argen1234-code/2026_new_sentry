#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include <stdint.h>

/* ---------- H723 单 Bank 1 MB 扇区地址 ---------- */
#define ADDR_FLASH_SECTOR_0     0x08000000UL
#define ADDR_FLASH_SECTOR_1     0x08008000UL
#define ADDR_FLASH_SECTOR_2     0x08010000UL
#define ADDR_FLASH_SECTOR_3     0x08018000UL
#define ADDR_FLASH_SECTOR_4     0x08020000UL
#define ADDR_FLASH_SECTOR_5     0x08040000UL
#define ADDR_SECTOR_6           0x08080000UL
#define ADDR_SECTOR_7           0x080C0000UL
#define FLASH_END_ADDR          0x08100000UL

/* 接口保持与原文件完全一致,地址已经乘上4了，所以 */
extern void flash_erase_address(uint32_t address, uint16_t len);
extern int8_t flash_write_single_address(uint32_t start_address, uint32_t *buf, uint32_t len);
extern int8_t flash_write_muli_address(uint32_t start_address, uint32_t end_address, uint32_t *buf, uint32_t len);
extern void flash_read(uint32_t address, uint32_t *buf, uint32_t len);
extern uint32_t ger_sector(uint32_t address);
extern uint32_t get_next_flash_address(uint32_t address);

#endif

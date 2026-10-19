#include "w25q64.h"          // ← 你用 CubeMX 生成的驱动

#define W25Q_BASE   0x000000UL
#define W25Q_SECTOR 4096               // 4 KB
#define W25Q_PAGE   256                // 256 B

/* 初始化：CubeMX 已配置好 OSPI 映射，这里只做 ID 校验 */
int8_t flash_init(void)
{
    return (OSPI_W25Qxx_Init() == OSPI_W25Qxx_OK) ? 0 : -1;
}

//int8_t flag = 0;
//void flash_read(uint32_t address, uint32_t *buf, uint32_t len)
//{
//	uint8_t *p = (uint8_t *)buf;   // 类型转换
//	flag = OSPI_W25Qxx_ReadBuffer(p, address,  len * 4);
//	if( flag !=0 ){
//		flag = 6;
//		Error_Handler();
//	}
//}
uint8_t tmp[256];
void flash_read(uint32_t address, uint32_t *buf, uint32_t len)
{
		uint32_t byteLen = len * 4;  // 计算字节长度

    // 进行4字节对齐检查
    if (((uint32_t)buf & 3) || (byteLen & 3)) 
    {
        uint32_t chunk;
        while (byteLen) 
        {
            chunk = (byteLen > 256) ? 256 : byteLen;  // 分批次读取，不超过256字节
            OSPI_W25Qxx_ReadBuffer((uint8_t *)tmp, address + (byteLen - chunk), chunk);
            memcpy((uint8_t *)buf + (byteLen - chunk), tmp, chunk);  // 拷贝数据到目标缓冲区
            byteLen -= chunk;
        }
    } 
    else 
    {
        OSPI_W25Qxx_ReadBuffer((uint8_t *)buf, address, byteLen);  // 直接读取
    }
}


/* 写：先擦 4 KB Sector，再按 256 B Page 写 */
int8_t flash_write_single_address(uint32_t address, uint32_t *buf, uint32_t len)
{
    uint32_t byte_len = len * 4;
    uint8_t *p8 = (uint8_t *)buf;
    uint32_t addr = W25Q_BASE + address;

    /* 1. 擦除包含目标范围的 4 KB Sector */
    uint32_t secFirst = address / W25Q_SECTOR;
    uint32_t secLast  = (address + byte_len - 1) / W25Q_SECTOR;
    for (uint32_t s = secFirst; s <= secLast; s++)
        if (OSPI_W25Qxx_SectorErase(W25Q_BASE + s * W25Q_SECTOR) != OSPI_W25Qxx_OK)
            return -1;

    /* 2. 按页写入（驱动已处理跨页） */
    return (OSPI_W25Qxx_WriteBuffer(p8, addr, byte_len) == OSPI_W25Qxx_OK) ? 0 : -1;
}

/* 扇区擦除：4 KB 为单位 */
void flash_erase_address(uint32_t address, uint16_t sector_cnt)
{
    for (uint16_t i = 0; i < sector_cnt; i++)
        OSPI_W25Qxx_SectorErase(W25Q_BASE + address + i * W25Q_SECTOR);
}

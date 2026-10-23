/**
 ******************************************************************************
 * @file    flash_w25q64.c
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   W25Q64 外部 Flash 实现。
 *
 * @note    本文件直接实现“W25Q64 over OSPI”的读、编程和擦除流程。
 * @note    这里不再引入独立的 W25Q64 port/command 中间层；当前板卡固定使用 W25Q64 + OSPI，
 *          因此直接调用 bsp_ospi 更清楚。
 * @note    差异: 旧版曾通过 W25Q64 device port 回调再转译到 OSPI；新版将 W25Q64 命令流程
 *          收进本文件的 static helper，减少无实际收益的抽象层。由于当前 OSPI BSP 尚无退出
 *          memory-mapped 的能力，本版暂不暴露 mmap 入口。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "flash_w25q64.h"

#include "bsp_ospi.h"

#include <stddef.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置设备驱动的协议常量、默认参数和私有类型，硬件绑定来自 Robot board 配置。
 * ========================================================================== */
#define W25Q64_JEDEC_ID 0xEF4017UL                  /**< W25Q64 JEDEC ID。 */
#define W25Q64_CMD_JEDEC_ID 0x9FU                   /**< W25Q64 读取 JEDEC ID 命令。 */
#define W25Q64_CMD_WRITE_ENABLE 0x06U               /**< W25Q64 写使能命令。 */
#define W25Q64_CMD_READ_STATUS_REG1 0x05U           /**< W25Q64 读取状态寄存器 1 命令。 */
#define W25Q64_CMD_SECTOR_ERASE 0x20U               /**< W25Q64 4KB 扇区擦除命令。 */
#define W25Q64_CMD_QUAD_INPUT_PAGE_PROGRAM 0x32U    /**< W25Q64 1-1-4 四线输入页编程命令。 */
#define W25Q64_CMD_FAST_READ_QUAD_IO 0xEBU          /**< W25Q64 1-4-4 四线 I/O 快速读取命令。 */
#define W25Q64_STATUS_BUSY 0x01U                    /**< W25Q64 状态寄存器 1 的 BUSY 位。 */
#define W25Q64_STATUS_WEL 0x02U                     /**< W25Q64 状态寄存器 1 的 WEL 位。 */
#define W25Q64_FAST_READ_DUMMY_CYCLES 6U            /**< W25Q64 快速读取 dummy cycle 数。 */
#define W25Q64_AUTOPOLLING_INTERVAL 0x10U           /**< W25Q64 自动轮询间隔，沿用原工程配置。 */

/* ==========================================================================
 * 状态缓存与工作区
 * --------------------------------------------------------------------------
 * 设备层只缓存原始设备状态、诊断和收发缓冲，不写机器人控制语义。
 * ========================================================================== */

/**
 * @brief 当前板载外部 Flash 几何信息。
 */
static const flash_info_t flash_info_data = {
    .base = FLASH_DEVICE_BASE,
    .size = FLASH_DEVICE_SIZE,
    .page_size = FLASH_DEVICE_PAGE_SIZE,
    .erase_size = FLASH_DEVICE_ERASE_SIZE,
};

/**
 * @brief Flash 是否已完成初始化和 ID 校验。
 */
static uint8_t flash_ready = 0U;

/**
 * @brief 获取板载外部 Flash 使用的 OSPI flash_bus。
 */
static bsp_ospi_bus_t *flash_bus(void)
{
    return bsp_ospi_bus(FLASH_OSPI_ID);
}

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责帧格式、数值范围、在线时间和底层返回值检查。
 * ========================================================================== */

/**
 * @brief 将 OSPI BSP 状态码转换为 Flash BSP 状态码。
 */
static flash_status_t conv_ospi(bsp_ospi_status_t status)
{
    return (status == BSP_OSPI_STATUS_OK) ? FLASH_OK : FLASH_ERR_IO;
}

/**
 * @brief 构造带通用默认值的 OSPI 命令。
 */
static bsp_ospi_cmd_t cmd_base(uint8_t instruction)
{
    bsp_ospi_cmd_t cmd = {0};

    cmd.operation_type = BSP_OSPI_OPTYPE_COMMON_CFG;
    cmd.flash_id = FLASH_OSPI_FLASH_ID;
    cmd.instruction = instruction;
    cmd.instruction_mode = BSP_OSPI_LINE_1;
    cmd.instruction_size = BSP_OSPI_SIZE_8_BITS;
    cmd.instruction_dtr = BSP_OSPI_DTR_DISABLE;
    cmd.address_mode = BSP_OSPI_LINE_NONE;
    cmd.address_size = BSP_OSPI_SIZE_24_BITS;
    cmd.address_dtr = BSP_OSPI_DTR_DISABLE;
    cmd.alternate_bytes_mode = BSP_OSPI_LINE_NONE;
    cmd.alternate_bytes_dtr = BSP_OSPI_DTR_DISABLE;
    cmd.data_mode = BSP_OSPI_LINE_NONE;
    cmd.data_dtr = BSP_OSPI_DTR_DISABLE;
    cmd.dqs_mode = BSP_OSPI_DQS_DISABLE;
    cmd.sioo_mode = BSP_OSPI_SIOO_INST_EVERY_CMD;

    return cmd;
}

/**
 * @brief 检查 Flash 地址范围是否位于板载外部 Flash 容量内。
 */
static int range_ok(uint32_t address, uint32_t length)
{
    if ((length == 0U) || (address >= FLASH_DEVICE_SIZE)) {
        return 0;
    }

    if (length > (FLASH_DEVICE_SIZE - address)) {
        return 0;
    }

    return 1;
}

/**
 * @brief 检查地址范围是否按擦除粒度对齐。
 */
static int erase_align(uint32_t address, uint32_t length)
{
    if (((address % FLASH_DEVICE_ERASE_SIZE) != 0U) || ((length % FLASH_DEVICE_ERASE_SIZE) != 0U)) {
        return 0;
    }

    return 1;
}

/**
 * @brief 检查页编程请求是否保持在同一个页内。
 */
static int page_ok(uint32_t address, uint16_t length)
{
    if ((length == 0U) || (length > FLASH_DEVICE_PAGE_SIZE) || (range_ok(address, length) == 0)) {
        return 0;
    }

    if (((address % FLASH_DEVICE_PAGE_SIZE) + (uint32_t)length) > FLASH_DEVICE_PAGE_SIZE) {
        return 0;
    }

    return 1;
}

/**
 * @brief 检查 Flash 是否已完成初始化。
 */
static flash_status_t ready(void)
{
    return (flash_ready != 0U) ? FLASH_OK : FLASH_ERR_NOT_READY;
}

/**
 * @brief 读取 W25Q64 JEDEC ID。
 */
static flash_status_t read_id(uint32_t *id)
{
    uint8_t rx[3] = {0U, 0U, 0U};
    bsp_ospi_cmd_t cmd;
    flash_status_t status;

    if (id == NULL) {
        return FLASH_ERR_ARG;
    }

    cmd = cmd_base(W25Q64_CMD_JEDEC_ID);
    cmd.data_mode = BSP_OSPI_LINE_1;
    cmd.data_length = sizeof(rx);

    status = conv_ospi(bsp_ospi_command(flash_bus(), &cmd, FLASH_CMD_TIMEOUT_MS));
    if (status != FLASH_OK) {
        return status;
    }

    status = conv_ospi(bsp_ospi_receive(flash_bus(), rx, sizeof(rx), FLASH_CMD_TIMEOUT_MS));
    if (status != FLASH_OK) {
        return status;
    }

    *id = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | (uint32_t)rx[2];
    return FLASH_OK;
}

/**
 * @brief 执行 W25Q64 状态寄存器自动轮询。
 */
static flash_status_t poll_status(uint32_t match, uint32_t mask, uint32_t timeout_ms)
{
    bsp_ospi_cmd_t cmd;
    bsp_ospi_polling_t polling;

    cmd = cmd_base(W25Q64_CMD_READ_STATUS_REG1);
    cmd.data_mode = BSP_OSPI_LINE_1;
    cmd.data_length = 1U;

    polling.match = match;
    polling.mask = mask;
    polling.match_mode = BSP_OSPI_MATCH_MODE_AND;
    polling.interval = W25Q64_AUTOPOLLING_INTERVAL;
    polling.automatic_stop = BSP_OSPI_AUTOMATIC_STOP_ENABLE;

    return conv_ospi(bsp_ospi_poll_command(flash_bus(), &cmd, &polling, timeout_ms));
}

/* ==========================================================================
 * 收发、解析与打包
 * --------------------------------------------------------------------------
 * 这里把字节流或寄存器数据转换成设备事实；控制含义由 Component/Input 或 Apps 解释。
 * ========================================================================== */

/**
 * @brief 等待 W25Q64 BUSY 位清零。
 */
static flash_status_t wait_ready(uint32_t timeout_ms)
{
    return poll_status(0U, W25Q64_STATUS_BUSY, timeout_ms);
}

/**
 * @brief 发送写使能并等待 WEL 置位。
 */
static flash_status_t write_enable(void)
{
    bsp_ospi_cmd_t cmd;
    flash_status_t status;

    status = wait_ready(FLASH_CMD_TIMEOUT_MS);
    if (status != FLASH_OK) {
        return status;
    }

    cmd = cmd_base(W25Q64_CMD_WRITE_ENABLE);

    status = conv_ospi(bsp_ospi_command(flash_bus(), &cmd, FLASH_CMD_TIMEOUT_MS));
    if (status != FLASH_OK) {
        return status;
    }

    return poll_status(W25Q64_STATUS_WEL, W25Q64_STATUS_WEL, FLASH_CMD_TIMEOUT_MS);
}

/**
 * @brief 擦除一个 4KB 扇区。
 */
static flash_status_t erase_4k(uint32_t address)
{
    bsp_ospi_cmd_t cmd;
    flash_status_t status;

    if (range_ok(address, FLASH_DEVICE_ERASE_SIZE) == 0) {
        return FLASH_ERR_RANGE;
    }

    if ((address % FLASH_DEVICE_ERASE_SIZE) != 0U) {
        return FLASH_ERR_ALIGN;
    }

    status = write_enable();
    if (status != FLASH_OK) {
        return status;
    }

    cmd = cmd_base(W25Q64_CMD_SECTOR_ERASE);
    cmd.address = address;
    cmd.address_mode = BSP_OSPI_LINE_1;

    status = conv_ospi(bsp_ospi_command(flash_bus(), &cmd, FLASH_CMD_TIMEOUT_MS));
    if (status != FLASH_OK) {
        return status;
    }

    return wait_ready(FLASH_CMD_TIMEOUT_MS);
}

/**
 * @brief 页内编程数据。
 */
static flash_status_t prog_page(uint32_t address, const uint8_t *data, uint16_t length)
{
    bsp_ospi_cmd_t cmd;
    flash_status_t status;

    if (data == NULL) {
        return FLASH_ERR_ARG;
    }

    if (page_ok(address, length) == 0) {
        return FLASH_ERR_RANGE;
    }

    status = write_enable();
    if (status != FLASH_OK) {
        return status;
    }

    cmd = cmd_base(W25Q64_CMD_QUAD_INPUT_PAGE_PROGRAM);
    cmd.address = address;
    cmd.address_mode = BSP_OSPI_LINE_1;
    cmd.data_mode = BSP_OSPI_LINE_4;
    cmd.data_length = length;

    status = conv_ospi(bsp_ospi_command(flash_bus(), &cmd, FLASH_CMD_TIMEOUT_MS));
    if (status != FLASH_OK) {
        return status;
    }

    status = conv_ospi(bsp_ospi_transmit(flash_bus(), data, length, FLASH_CMD_TIMEOUT_MS));
    if (status != FLASH_OK) {
        return status;
    }

    return wait_ready(FLASH_CMD_TIMEOUT_MS);
}

/**
 * @brief 四线 I/O 快速读取数据。
 */
static flash_status_t read_quad(uint32_t address, uint8_t *data, uint32_t length)
{
    bsp_ospi_cmd_t cmd;
    flash_status_t status;

    if (data == NULL) {
        return FLASH_ERR_ARG;
    }

    if (range_ok(address, length) == 0) {
        return FLASH_ERR_RANGE;
    }

    cmd = cmd_base(W25Q64_CMD_FAST_READ_QUAD_IO);
    cmd.address = address;
    cmd.address_mode = BSP_OSPI_LINE_4;
    cmd.data_mode = BSP_OSPI_LINE_4;
    cmd.data_length = length;
    cmd.dummy_cycles = W25Q64_FAST_READ_DUMMY_CYCLES;

    status = conv_ospi(bsp_ospi_command(flash_bus(), &cmd, FLASH_CMD_TIMEOUT_MS));
    if (status != FLASH_OK) {
        return status;
    }

    return conv_ospi(bsp_ospi_receive(flash_bus(), data, length, FLASH_CMD_TIMEOUT_MS));
}

/**
 * @brief 擦除覆盖指定地址范围的所有 4KB 扇区。
 */
static flash_status_t erase_range(uint32_t address, uint32_t length)
{
    uint32_t first_sector;
    uint32_t last_sector;
    uint32_t sector;
    flash_status_t status;

    if (range_ok(address, length) == 0) {
        return FLASH_ERR_RANGE;
    }

    first_sector = address / FLASH_DEVICE_ERASE_SIZE;
    last_sector = (address + length - 1U) / FLASH_DEVICE_ERASE_SIZE;

    for (sector = first_sector; sector <= last_sector; ++sector) {
        status = erase_4k(sector * FLASH_DEVICE_ERASE_SIZE);
        if (status != FLASH_OK) {
            return status;
        }
    }

    return FLASH_OK;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只提供初始化、喂入、查询和发送入口，不保留旧 task/global 兼容接口。
 * ========================================================================== */
flash_status_t flash_w25q64_init(void)
{
    uint32_t id = 0U;
    flash_status_t status;

    flash_ready = 0U;

    status = read_id(&id);
    if (status != FLASH_OK) {
        return status;
    }

    if (id != W25Q64_JEDEC_ID) {
        return FLASH_ERR_IO;
    }

    flash_ready = 1U;
    return FLASH_OK;
}

const flash_info_t *flash_w25q64_info(void)
{
    return &flash_info_data;
}

flash_status_t flash_w25q64_read(uint32_t address, void *buffer, uint32_t length)
{
    flash_status_t status;

    if (buffer == NULL) {
        return FLASH_ERR_ARG;
    }

    status = ready();
    if (status != FLASH_OK) {
        return status;
    }

    return read_quad(address, (uint8_t *)buffer, length);
}

flash_status_t flash_w25q64_prog(uint32_t address, const void *buffer, uint32_t length)
{
    uint32_t chunk;
    uint32_t current_address;
    uint32_t end_address;
    const uint8_t *current_data;
    flash_status_t status;

    if (buffer == NULL) {
        return FLASH_ERR_ARG;
    }

    status = ready();
    if (status != FLASH_OK) {
        return status;
    }

    if (range_ok(address, length) == 0) {
        return FLASH_ERR_RANGE;
    }

    current_address = address;
    end_address = address + length;
    current_data = (const uint8_t *)buffer;
    chunk = FLASH_DEVICE_PAGE_SIZE - (address % FLASH_DEVICE_PAGE_SIZE);
    if (chunk > length) {
        chunk = length;
    }

    while (current_address < end_address) {
        status = prog_page(current_address, current_data, (uint16_t)chunk);
        if (status != FLASH_OK) {
            return status;
        }

        current_address += chunk;
        current_data += chunk;
        chunk = ((current_address + FLASH_DEVICE_PAGE_SIZE) > end_address)
              ? (end_address - current_address)
              : FLASH_DEVICE_PAGE_SIZE;
    }

    return FLASH_OK;
}

flash_status_t flash_w25q64_erase(uint32_t address, uint32_t length)
{
    flash_status_t status;

    status = ready();
    if (status != FLASH_OK) {
        return status;
    }

    if (range_ok(address, length) == 0) {
        return FLASH_ERR_RANGE;
    }

    if (erase_align(address, length) == 0) {
        return FLASH_ERR_ALIGN;
    }

    return erase_range(address, length);
}

flash_status_t flash_w25q64_write(uint32_t address, const void *buffer, uint32_t length)
{
    flash_status_t status;

    if (buffer == NULL) {
        return FLASH_ERR_ARG;
    }

    status = ready();
    if (status != FLASH_OK) {
        return status;
    }

    if (range_ok(address, length) == 0) {
        return FLASH_ERR_RANGE;
    }

    status = erase_range(address, length);
    if (status != FLASH_OK) {
        return status;
    }

    return flash_w25q64_prog(address, buffer, length);
}

uint32_t flash_w25q64_sector(uint32_t address)
{
    return address / FLASH_DEVICE_ERASE_SIZE;
}

uint32_t flash_w25q64_next_sector(uint32_t address)
{
    uint32_t next_address;

    if (address >= FLASH_DEVICE_END) {
        return FLASH_DEVICE_END;
    }

    next_address = ((address / FLASH_DEVICE_ERASE_SIZE) + 1U) * FLASH_DEVICE_ERASE_SIZE;
    if (next_address > FLASH_DEVICE_END) {
        return FLASH_DEVICE_END;
    }

    return next_address;
}

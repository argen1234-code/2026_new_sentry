/**
 ******************************************************************************
 * @file    bsp_ospi.c
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   OSPI 控制器 BSP 抽象实现。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "bsp_ospi.h"

#include <stddef.h>

/**
 * @brief 板级外部 flash 使用的 OSPI bus。
 */
/* @diff: 旧版和早期新版直接写死 hospi2；此处改成 bus/context，默认实例仍然绑定 hospi2。 */
/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置板级外设封装需要的私有宏、局部表和轻量状态，不承载机器人业务语义。
 * ========================================================================== */

/* ==========================================================================
 * 状态缓存与工作区
 * --------------------------------------------------------------------------
 * BSP 层只缓存外设句柄、DMA 缓冲和硬件状态，具体设备用途由 Robot 配置决定。
 * ========================================================================== */
static bsp_ospi_bus_t bsp_ospi_flash_bus = {
    .hal = &hospi2,
    .last_data_length = 0U,
    .last_hal_status = HAL_OK,
};

bsp_ospi_bus_t *bsp_ospi_bus(bsp_ospi_id_t id)
{
    if (id == BSP_OSPI_ID_FLASH) {
        return &bsp_ospi_flash_bus;
    }

    return NULL;
}

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责索引、句柄、参数和 HAL 状态检查，避免上层直接碰 CubeMX 句柄细节。
 * ========================================================================== */

/**
 * @brief 将 HAL 返回值转换为 BSP OSPI 状态码。
 *
 * @param status HAL OSPI API 返回值。
 * @return BSP OSPI 状态码。
 */
static bsp_ospi_status_t status_from_hal(HAL_StatusTypeDef status)
{
    switch (status) {
    case HAL_OK:
        return BSP_OSPI_STATUS_OK;
    case HAL_BUSY:
        return BSP_OSPI_STATUS_BUSY;
    case HAL_TIMEOUT:
        return BSP_OSPI_STATUS_TIMEOUT;
    case HAL_ERROR:
    default:
        return BSP_OSPI_STATUS_ERROR;
    }
}

/**
 * @brief 映射 OSPI operation type 到 HAL_OSPI_OPTYPE_*。
 *
 * @param value BSP operation type。
 * @return HAL OSPI operation type。
 */
static uint32_t map_op_type(bsp_ospi_operation_type_t value)
{
    switch (value) {
    case BSP_OSPI_OPTYPE_READ_CFG:
        return HAL_OSPI_OPTYPE_READ_CFG;
    case BSP_OSPI_OPTYPE_WRITE_CFG:
        return HAL_OSPI_OPTYPE_WRITE_CFG;
    case BSP_OSPI_OPTYPE_WRAP_CFG:
        return HAL_OSPI_OPTYPE_WRAP_CFG;
    case BSP_OSPI_OPTYPE_COMMON_CFG:
    default:
        return HAL_OSPI_OPTYPE_COMMON_CFG;
    }
}

/**
 * @brief 映射 flash ID 到 HAL_OSPI_FLASH_ID_*。
 *
 * @param value BSP flash ID。
 * @return HAL OSPI flash ID。
 */
static uint32_t map_flash_id(bsp_ospi_flash_id_t value)
{
    return (value == BSP_OSPI_FLASH_ID_2) ? HAL_OSPI_FLASH_ID_2 : HAL_OSPI_FLASH_ID_1;
}

/**
 * @brief 映射指令线宽到 HAL_OSPI_INSTRUCTION_*。
 *
 * @param value BSP 线宽。
 * @return HAL OSPI instruction mode。
 */
static uint32_t map_line_mode(bsp_ospi_line_mode_t value)
{
    switch (value) {
    case BSP_OSPI_LINE_1:
        return HAL_OSPI_INSTRUCTION_1_LINE;
    case BSP_OSPI_LINE_2:
        return HAL_OSPI_INSTRUCTION_2_LINES;
    case BSP_OSPI_LINE_4:
        return HAL_OSPI_INSTRUCTION_4_LINES;
    case BSP_OSPI_LINE_8:
        return HAL_OSPI_INSTRUCTION_8_LINES;
    case BSP_OSPI_LINE_NONE:
    default:
        return HAL_OSPI_INSTRUCTION_NONE;
    }
}

/**
 * @brief 映射地址线宽到 HAL_OSPI_ADDRESS_*。
 *
 * @param value BSP 线宽。
 * @return HAL OSPI address mode。
 */
static uint32_t map_addr_mode(bsp_ospi_line_mode_t value)
{
    switch (value) {
    case BSP_OSPI_LINE_1:
        return HAL_OSPI_ADDRESS_1_LINE;
    case BSP_OSPI_LINE_2:
        return HAL_OSPI_ADDRESS_2_LINES;
    case BSP_OSPI_LINE_4:
        return HAL_OSPI_ADDRESS_4_LINES;
    case BSP_OSPI_LINE_8:
        return HAL_OSPI_ADDRESS_8_LINES;
    case BSP_OSPI_LINE_NONE:
    default:
        return HAL_OSPI_ADDRESS_NONE;
    }
}

/**
 * @brief 映射交替字节线宽到 HAL_OSPI_ALTERNATE_BYTES_*。
 *
 * @param value BSP 线宽。
 * @return HAL OSPI alternate bytes mode。
 */
static uint32_t map_alt_bytes_mode(bsp_ospi_line_mode_t value)
{
    switch (value) {
    case BSP_OSPI_LINE_1:
        return HAL_OSPI_ALTERNATE_BYTES_1_LINE;
    case BSP_OSPI_LINE_2:
        return HAL_OSPI_ALTERNATE_BYTES_2_LINES;
    case BSP_OSPI_LINE_4:
        return HAL_OSPI_ALTERNATE_BYTES_4_LINES;
    case BSP_OSPI_LINE_8:
        return HAL_OSPI_ALTERNATE_BYTES_8_LINES;
    case BSP_OSPI_LINE_NONE:
    default:
        return HAL_OSPI_ALTERNATE_BYTES_NONE;
    }
}

/**
 * @brief 映射数据线宽到 HAL_OSPI_DATA_*。
 *
 * @param value BSP 线宽。
 * @return HAL OSPI data mode。
 */
static uint32_t map_data_mode(bsp_ospi_line_mode_t value)
{
    switch (value) {
    case BSP_OSPI_LINE_1:
        return HAL_OSPI_DATA_1_LINE;
    case BSP_OSPI_LINE_2:
        return HAL_OSPI_DATA_2_LINES;
    case BSP_OSPI_LINE_4:
        return HAL_OSPI_DATA_4_LINES;
    case BSP_OSPI_LINE_8:
        return HAL_OSPI_DATA_8_LINES;
    case BSP_OSPI_LINE_NONE:
    default:
        return HAL_OSPI_DATA_NONE;
    }
}

/**
 * @brief 映射指令位宽到 HAL_OSPI_INSTRUCTION_*_BITS。
 *
 * @param value BSP 位宽。
 * @return HAL OSPI instruction size。
 */
static uint32_t map_bus_width(bsp_ospi_bus_width_t value)
{
    switch (value) {
    case BSP_OSPI_SIZE_16_BITS:
        return HAL_OSPI_INSTRUCTION_16_BITS;
    case BSP_OSPI_SIZE_24_BITS:
        return HAL_OSPI_INSTRUCTION_24_BITS;
    case BSP_OSPI_SIZE_32_BITS:
        return HAL_OSPI_INSTRUCTION_32_BITS;
    case BSP_OSPI_SIZE_8_BITS:
    default:
        return HAL_OSPI_INSTRUCTION_8_BITS;
    }
}

/**
 * @brief 映射地址位宽到 HAL_OSPI_ADDRESS_*_BITS。
 *
 * @param value BSP 位宽。
 * @return HAL OSPI address size。
 */
static uint32_t map_addr_width(bsp_ospi_bus_width_t value)
{
    switch (value) {
    case BSP_OSPI_SIZE_16_BITS:
        return HAL_OSPI_ADDRESS_16_BITS;
    case BSP_OSPI_SIZE_24_BITS:
        return HAL_OSPI_ADDRESS_24_BITS;
    case BSP_OSPI_SIZE_32_BITS:
        return HAL_OSPI_ADDRESS_32_BITS;
    case BSP_OSPI_SIZE_8_BITS:
    default:
        return HAL_OSPI_ADDRESS_8_BITS;
    }
}

/**
 * @brief 映射指令 DTR 模式到 HAL_OSPI_INSTRUCTION_DTR_*。
 *
 * @param value BSP DTR 模式。
 * @return HAL OSPI instruction DTR mode。
 */
static uint32_t map_dtr_mode(bsp_ospi_dtr_mode_t value)
{
    return (value == BSP_OSPI_DTR_ENABLE) ? HAL_OSPI_INSTRUCTION_DTR_ENABLE : HAL_OSPI_INSTRUCTION_DTR_DISABLE;
}

/**
 * @brief 映射地址 DTR 模式到 HAL_OSPI_ADDRESS_DTR_*。
 *
 * @param value BSP DTR 模式。
 * @return HAL OSPI address DTR mode。
 */
static uint32_t map_addr_dtr_mode(bsp_ospi_dtr_mode_t value)
{
    return (value == BSP_OSPI_DTR_ENABLE) ? HAL_OSPI_ADDRESS_DTR_ENABLE : HAL_OSPI_ADDRESS_DTR_DISABLE;
}

/**
 * @brief 映射交替字节 DTR 模式到 HAL_OSPI_ALTERNATE_BYTES_DTR_*。
 *
 * @param value BSP DTR 模式。
 * @return HAL OSPI alternate bytes DTR mode。
 */
static uint32_t map_alt_bytes_dtr(bsp_ospi_dtr_mode_t value)
{
    return (value == BSP_OSPI_DTR_ENABLE) ? HAL_OSPI_ALTERNATE_BYTES_DTR_ENABLE : HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
}

/**
 * @brief 映射数据 DTR 模式到 HAL_OSPI_DATA_DTR_*。
 *
 * @param value BSP DTR 模式。
 * @return HAL OSPI data DTR mode。
 */
static uint32_t map_data_dtr_mode(bsp_ospi_dtr_mode_t value)
{
    return (value == BSP_OSPI_DTR_ENABLE) ? HAL_OSPI_DATA_DTR_ENABLE : HAL_OSPI_DATA_DTR_DISABLE;
}

/**
 * @brief 映射 DQS 模式到 HAL_OSPI_DQS_*。
 *
 * @param value BSP DQS 模式。
 * @return HAL OSPI DQS mode。
 */
static uint32_t map_dqs_mode(bsp_ospi_dqs_mode_t value)
{
    return (value == BSP_OSPI_DQS_ENABLE) ? HAL_OSPI_DQS_ENABLE : HAL_OSPI_DQS_DISABLE;
}

/**
 * @brief 映射 SIOO 模式到 HAL_OSPI_SIOO_*。
 *
 * @param value BSP SIOO 模式。
 * @return HAL OSPI SIOO mode。
 */
static uint32_t map_sioo_mode(bsp_ospi_sioo_mode_t value)
{
    return (value == BSP_OSPI_SIOO_INST_ONLY_FIRST_CMD) ? HAL_OSPI_SIOO_INST_ONLY_FIRST_CMD : HAL_OSPI_SIOO_INST_EVERY_CMD;
}

/**
 * @brief 映射自动轮询匹配模式到 HAL_OSPI_MATCH_MODE_*。
 *
 * @param value BSP 匹配模式。
 * @return HAL OSPI match mode。
 */
static uint32_t map_match_mode(bsp_ospi_match_mode_t value)
{
    return (value == BSP_OSPI_MATCH_MODE_OR) ? HAL_OSPI_MATCH_MODE_OR : HAL_OSPI_MATCH_MODE_AND;
}

/**
 * @brief 映射自动停止模式到 HAL_OSPI_AUTOMATIC_STOP_*。
 *
 * @param value BSP 自动停止模式。
 * @return HAL OSPI automatic stop mode。
 */
static uint32_t map_auto_stop(bsp_ospi_automatic_stop_t value)
{
    return (value == BSP_OSPI_AUTOMATIC_STOP_DISABLE) ? HAL_OSPI_AUTOMATIC_STOP_DISABLE : HAL_OSPI_AUTOMATIC_STOP_ENABLE;
}

/**
 * @brief 映射 memory mapped timeout counter 模式到 HAL_OSPI_TIMEOUT_COUNTER_*。
 *
 * @param value BSP timeout counter 模式。
 * @return HAL OSPI timeout activation mode。
 */
static uint32_t map_timeout_act(bsp_ospi_timeout_activation_t value)
{
    return (value == BSP_OSPI_TIMEOUT_COUNTER_ENABLE) ? HAL_OSPI_TIMEOUT_COUNTER_ENABLE : HAL_OSPI_TIMEOUT_COUNTER_DISABLE;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只暴露板级能力和安全钳位，Devices/Apps 通过这些接口绑定硬件。
 * ========================================================================== */
bsp_ospi_status_t bsp_ospi_command(bsp_ospi_bus_t *bus,
                                   const bsp_ospi_cmd_t *cmd,
                                   uint32_t timeout_ms)
{
    OSPI_RegularCmdTypeDef hal_cmd = {0};
    HAL_StatusTypeDef hal_status;

    if ((bus == NULL) || (bus->hal == NULL) || (cmd == NULL)) {
        return BSP_OSPI_STATUS_INVALID_ARGUMENT;
    }

    hal_cmd.OperationType = map_op_type(cmd->operation_type);
    hal_cmd.FlashId = map_flash_id(cmd->flash_id);
    hal_cmd.Instruction = cmd->instruction;
    hal_cmd.InstructionMode = map_line_mode(cmd->instruction_mode);
    hal_cmd.InstructionSize = map_bus_width(cmd->instruction_size);
    hal_cmd.InstructionDtrMode = map_dtr_mode(cmd->instruction_dtr);
    hal_cmd.Address = cmd->address;
    hal_cmd.AddressMode = map_addr_mode(cmd->address_mode);
    hal_cmd.AddressSize = map_addr_width(cmd->address_size);
    hal_cmd.AddressDtrMode = map_addr_dtr_mode(cmd->address_dtr);
    hal_cmd.AlternateBytes = 0U;
    hal_cmd.AlternateBytesMode = map_alt_bytes_mode(cmd->alternate_bytes_mode);
    hal_cmd.AlternateBytesSize = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
    hal_cmd.AlternateBytesDtrMode = map_alt_bytes_dtr(cmd->alternate_bytes_dtr);
    hal_cmd.DataMode = map_data_mode(cmd->data_mode);
    hal_cmd.DataDtrMode = map_data_dtr_mode(cmd->data_dtr);
    hal_cmd.NbData = cmd->data_length;
    hal_cmd.DummyCycles = cmd->dummy_cycles;
    hal_cmd.DQSMode = map_dqs_mode(cmd->dqs_mode);
    hal_cmd.SIOOMode = map_sioo_mode(cmd->sioo_mode);

    hal_status = HAL_OSPI_Command(bus->hal, &hal_cmd, timeout_ms);
    bus->last_hal_status = hal_status;
    if (hal_status == HAL_OK) {
        bus->last_data_length = cmd->data_length;
    }

    return status_from_hal(hal_status);
}

bsp_ospi_status_t bsp_ospi_transmit(bsp_ospi_bus_t *bus,
                                    const uint8_t *data,
                                    size_t length,
                                    uint32_t timeout_ms)
{
    HAL_StatusTypeDef hal_status;

    if ((bus == NULL) || (bus->hal == NULL) || (data == NULL)) {
        return BSP_OSPI_STATUS_INVALID_ARGUMENT;
    }

    if ((length == 0U) || (length != (size_t)bus->last_data_length)) {
        /* @diff: 旧版 HAL 由上一条 command 的 NbData 决定长度；新版保留 length 参数并校验，避免调用者误传。 */
        return BSP_OSPI_STATUS_LENGTH_MISMATCH;
    }

    hal_status = HAL_OSPI_Transmit(bus->hal, (uint8_t *)data, timeout_ms);
    bus->last_hal_status = hal_status;
    return status_from_hal(hal_status);
}

bsp_ospi_status_t bsp_ospi_receive(bsp_ospi_bus_t *bus,
                                   uint8_t *data,
                                   size_t length,
                                   uint32_t timeout_ms)
{
    HAL_StatusTypeDef hal_status;

    if ((bus == NULL) || (bus->hal == NULL) || (data == NULL)) {
        return BSP_OSPI_STATUS_INVALID_ARGUMENT;
    }

    if ((length == 0U) || (length != (size_t)bus->last_data_length)) {
        /* @diff: 旧版 HAL 由上一条 command 的 NbData 决定长度；新版保留 length 参数并校验，避免调用者误传。 */
        return BSP_OSPI_STATUS_LENGTH_MISMATCH;
    }

    hal_status = HAL_OSPI_Receive(bus->hal, data, timeout_ms);
    bus->last_hal_status = hal_status;
    return status_from_hal(hal_status);
}

bsp_ospi_status_t bsp_ospi_auto_polling(bsp_ospi_bus_t *bus,
                                        const bsp_ospi_polling_t *polling,
                                        uint32_t timeout_ms)
{
    OSPI_AutoPollingTypeDef hal_polling = {0};
    HAL_StatusTypeDef hal_status;

    if ((bus == NULL) || (bus->hal == NULL) || (polling == NULL)) {
        return BSP_OSPI_STATUS_INVALID_ARGUMENT;
    }

    hal_polling.Match = polling->match;
    hal_polling.Mask = polling->mask;
    hal_polling.MatchMode = map_match_mode(polling->match_mode);
    hal_polling.Interval = polling->interval;
    hal_polling.AutomaticStop = map_auto_stop(polling->automatic_stop);

    hal_status = HAL_OSPI_AutoPolling(bus->hal, &hal_polling, timeout_ms);
    bus->last_hal_status = hal_status;
    return status_from_hal(hal_status);
}

bsp_ospi_status_t bsp_ospi_memory_mapped(bsp_ospi_bus_t *bus,
                                         const bsp_ospi_memory_mapped_cfg_t *cfg)
{
    OSPI_MemoryMappedTypeDef hal_cfg = {0};
    HAL_StatusTypeDef hal_status;

    if ((bus == NULL) || (bus->hal == NULL) || (cfg == NULL)) {
        return BSP_OSPI_STATUS_INVALID_ARGUMENT;
    }

    hal_cfg.TimeOutActivation = map_timeout_act(cfg->timeout_activation);
    hal_cfg.TimeOutPeriod = cfg->timeout_period;

    hal_status = HAL_OSPI_MemoryMapped(bus->hal, &hal_cfg);
    bus->last_hal_status = hal_status;
    return status_from_hal(hal_status);
}

bsp_ospi_status_t bsp_ospi_poll_command(bsp_ospi_bus_t *bus,
                                        const bsp_ospi_cmd_t *cmd,
                                        const bsp_ospi_polling_t *polling,
                                        uint32_t timeout_ms)
{
    bsp_ospi_status_t status;

    if ((cmd == NULL) || (polling == NULL)) {
        return BSP_OSPI_STATUS_INVALID_ARGUMENT;
    }

    status = bsp_ospi_command(bus, cmd, timeout_ms);
    if (status != BSP_OSPI_STATUS_OK) {
        return status;
    }

    return bsp_ospi_auto_polling(bus, polling, timeout_ms);
}

bsp_ospi_status_t bsp_ospi_enter_memory_mapped(bsp_ospi_bus_t *bus,
                                               const bsp_ospi_cmd_t *cmd,
                                               const bsp_ospi_memory_mapped_cfg_t *cfg,
                                               uint32_t timeout_ms)
{
    bsp_ospi_status_t status;

    if ((cmd == NULL) || (cfg == NULL)) {
        return BSP_OSPI_STATUS_INVALID_ARGUMENT;
    }

    status = bsp_ospi_command(bus, cmd, timeout_ms);
    if (status != BSP_OSPI_STATUS_OK) {
        return status;
    }

    return bsp_ospi_memory_mapped(bus, cfg);
}

/**
 ******************************************************************************
 * @file    bsp_ospi.h
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   OSPI 控制器 BSP 抽象接口。
 *
 * @note    本文件只负责 STM32 OSPI 控制器和 HAL 适配，不包含具体
 *          NOR Flash 器件协议。
 * @note    新版通过 bsp_ospi_bus_t 描述一条 OSPI bus；默认板级
 *          flash bus 当前绑定 CubeMX 生成的 hospi2。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef PRINTK_BSP_OSPI_H
#define PRINTK_BSP_OSPI_H

#include "octospi.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * BSP 头文件描述板级外设能力、ID 和轻量配置，不描述设备用途或机器人装配语义。
 * ========================================================================== */
#define BSP_OSPI_TIMEOUT_DEFAULT_MS 5000U

/**
 * @brief OSPI BSP 返回状态。
 */
typedef enum
{
    BSP_OSPI_STATUS_OK = 0,
    BSP_OSPI_STATUS_ERROR = -1,
    BSP_OSPI_STATUS_INVALID_ARGUMENT = -2,
    BSP_OSPI_STATUS_TIMEOUT = -3,
    BSP_OSPI_STATUS_BUSY = -4,
    BSP_OSPI_STATUS_LENGTH_MISMATCH = -5
} bsp_ospi_status_t;

/**
 * @brief 板上可由 BSP 管理的 OSPI bus 标识。
 */
typedef enum
{
    BSP_OSPI_ID_FLASH = 0,
    BSP_OSPI_ID_COUNT
} bsp_ospi_id_t;

/**
 * @brief OSPI 命令操作类型，对应 HAL_OSPI_OPTYPE_*。
 */
typedef enum
{
    BSP_OSPI_OPTYPE_COMMON_CFG = 0,
    BSP_OSPI_OPTYPE_READ_CFG,
    BSP_OSPI_OPTYPE_WRITE_CFG,
    BSP_OSPI_OPTYPE_WRAP_CFG
} bsp_ospi_operation_type_t;

/**
 * @brief OSPI flash 片选编号，对应 HAL_OSPI_FLASH_ID_*。
 */
typedef enum
{
    BSP_OSPI_FLASH_ID_1 = 0,
    BSP_OSPI_FLASH_ID_2
} bsp_ospi_flash_id_t;

/**
 * @brief OSPI 指令、地址、数据、交替字节使用的线宽。
 */
typedef enum
{
    BSP_OSPI_LINE_NONE = 0,
    BSP_OSPI_LINE_1,
    BSP_OSPI_LINE_2,
    BSP_OSPI_LINE_4,
    BSP_OSPI_LINE_8
} bsp_ospi_line_mode_t;

/**
 * @brief OSPI 指令或地址字段宽度。
 */
typedef enum
{
    BSP_OSPI_SIZE_8_BITS = 8,
    BSP_OSPI_SIZE_16_BITS = 16,
    BSP_OSPI_SIZE_24_BITS = 24,
    BSP_OSPI_SIZE_32_BITS = 32
} bsp_ospi_bus_width_t;

/**
 * @brief OSPI DTR 模式开关。
 */
typedef enum
{
    BSP_OSPI_DTR_DISABLE = 0,
    BSP_OSPI_DTR_ENABLE
} bsp_ospi_dtr_mode_t;

/**
 * @brief OSPI DQS 模式开关。
 */
typedef enum
{
    BSP_OSPI_DQS_DISABLE = 0,
    BSP_OSPI_DQS_ENABLE
} bsp_ospi_dqs_mode_t;

/**
 * @brief OSPI SIOO 模式。
 */
typedef enum
{
    BSP_OSPI_SIOO_INST_EVERY_CMD = 0,
    BSP_OSPI_SIOO_INST_ONLY_FIRST_CMD
} bsp_ospi_sioo_mode_t;

/**
 * @brief OSPI 自动轮询匹配方式。
 */
typedef enum
{
    BSP_OSPI_MATCH_MODE_AND = 0,
    BSP_OSPI_MATCH_MODE_OR
} bsp_ospi_match_mode_t;

/**
 * @brief OSPI 自动轮询命中后的自动停止模式。
 */
typedef enum
{
    BSP_OSPI_AUTOMATIC_STOP_DISABLE = 0,
    BSP_OSPI_AUTOMATIC_STOP_ENABLE
} bsp_ospi_automatic_stop_t;

/**
 * @brief OSPI memory mapped timeout counter 模式。
 */
typedef enum
{
    BSP_OSPI_TIMEOUT_COUNTER_DISABLE = 0,
    BSP_OSPI_TIMEOUT_COUNTER_ENABLE
} bsp_ospi_timeout_activation_t;

/**
 * @brief OSPI bus 上下文。
 *
 * 新增 OSPI 实例时，定义一个新的 bsp_ospi_bus_t，并把 hal 指向
 * CubeMX 生成的 OSPI_HandleTypeDef。last_data_length 由 command
 * 接口维护，用于 transmit/receive 的长度校验。
 */
typedef struct
{
    OSPI_HandleTypeDef *hal;              /**< 底层 HAL OSPI 句柄。 */
    uint32_t last_data_length;            /**< 最近一次 command 配置的 NbData。 */
    HAL_StatusTypeDef last_hal_status;    /**< 最近一次 HAL 调用返回值。 */
} bsp_ospi_bus_t;

/**
 * @brief OSPI regular command 描述。
 *
 * 该结构只描述 OSPI 事务字段，具体 HAL 宏映射由 bsp_ospi.c 完成。
 */
typedef struct
{
    bsp_ospi_operation_type_t operation_type;     /**< HAL OSPI operation type。 */
    bsp_ospi_flash_id_t flash_id;                 /**< 目标 flash ID。 */
    uint32_t instruction;                         /**< 指令值。 */
    bsp_ospi_line_mode_t instruction_mode;        /**< 指令线宽。 */
    bsp_ospi_bus_width_t instruction_size;        /**< 指令宽度。 */
    bsp_ospi_dtr_mode_t instruction_dtr;          /**< 指令 DTR 模式。 */
    uint32_t address;                             /**< 地址值。 */
    bsp_ospi_line_mode_t address_mode;            /**< 地址线宽。 */
    bsp_ospi_bus_width_t address_size;            /**< 地址宽度。 */
    bsp_ospi_dtr_mode_t address_dtr;              /**< 地址 DTR 模式。 */
    bsp_ospi_line_mode_t alternate_bytes_mode;    /**< 交替字节线宽。 */
    bsp_ospi_dtr_mode_t alternate_bytes_dtr;      /**< 交替字节 DTR 模式。 */
    bsp_ospi_line_mode_t data_mode;               /**< 数据线宽。 */
    bsp_ospi_dtr_mode_t data_dtr;                 /**< 数据 DTR 模式。 */
    uint32_t data_length;                         /**< 数据长度，对应 HAL NbData。 */
    uint32_t dummy_cycles;                        /**< dummy cycles 数量。 */
    bsp_ospi_dqs_mode_t dqs_mode;                 /**< DQS 模式。 */
    bsp_ospi_sioo_mode_t sioo_mode;               /**< SIOO 模式。 */
} bsp_ospi_cmd_t;

/**
 * @brief OSPI 自动轮询配置。
 */
typedef struct
{
    uint32_t match;                              /**< 期望匹配值。 */
    uint32_t mask;                               /**< 匹配掩码。 */
    bsp_ospi_match_mode_t match_mode;            /**< AND/OR 匹配模式。 */
    uint32_t interval;                           /**< 轮询间隔。 */
    bsp_ospi_automatic_stop_t automatic_stop;    /**< 匹配后是否自动停止。 */
} bsp_ospi_polling_t;

/**
 * @brief OSPI memory mapped 配置。
 */
typedef struct
{
    bsp_ospi_timeout_activation_t timeout_activation;    /**< timeout counter 开关。 */
    uint32_t timeout_period;                             /**< timeout 周期。 */
} bsp_ospi_memory_mapped_cfg_t;

/**
 * @brief 板级外部 flash 默认 OSPI bus，当前绑定 hospi2。
 *
 * 如果后续从 OSPI2 换到 OSPI1，优先改 bsp_ospi.c 中该实例的 hal 字段；
 * 如果一块板上存在多条 OSPI bus，则新增独立的 bsp_ospi_bus_t 实例。
 */
/**
 * @brief 根据 BSP OSPI 标识获取 OSPI bus。
 *
 * @param id BSP OSPI bus 标识。
 * @return 匹配成功返回 OSPI bus 指针，否则返回 `NULL`。
 */
bsp_ospi_bus_t *bsp_ospi_bus(bsp_ospi_id_t id);

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按初始化、查询、收发和回调辅助排列，上层不应直接访问 CubeMX 句柄。
 * ========================================================================== */

/**
 * @brief 在指定 OSPI bus 上发送 regular command。
 *
 * 发送成功后会记录 cmd->data_length，供 transmit/receive 校验长度。
 *
 * @param bus 目标 OSPI bus。
 * @param cmd OSPI regular command 描述。
 * @param timeout_ms 超时时间，单位为毫秒。
 * @return 返回 BSP OSPI 状态码。
 */
bsp_ospi_status_t bsp_ospi_command(bsp_ospi_bus_t *bus,
                                   const bsp_ospi_cmd_t *cmd,
                                   uint32_t timeout_ms);
/**
 * @brief 在指定 OSPI bus 上发送数据，length 必须等于上一条 command 的 data_length。
 *
 * @param bus 目标 OSPI bus。
 * @param data 待发送数据缓冲区。
 * @param length 待发送数据长度，单位为字节。
 * @param timeout_ms 超时时间，单位为毫秒。
 * @return 返回 BSP OSPI 状态码。
 */
bsp_ospi_status_t bsp_ospi_transmit(bsp_ospi_bus_t *bus,
                                    const uint8_t *data,
                                    size_t length,
                                    uint32_t timeout_ms);
/**
 * @brief 在指定 OSPI bus 上接收数据，length 必须等于上一条 command 的 data_length。
 *
 * @param bus 目标 OSPI bus。
 * @param data 接收缓冲区。
 * @param length 接收数据长度，单位为字节。
 * @param timeout_ms 超时时间，单位为毫秒。
 * @return 返回 BSP OSPI 状态码。
 */
bsp_ospi_status_t bsp_ospi_receive(bsp_ospi_bus_t *bus,
                                   uint8_t *data,
                                   size_t length,
                                   uint32_t timeout_ms);
/**
 * @brief 在指定 OSPI bus 上执行自动轮询。
 *
 * 调用前通常需要先发送读状态寄存器命令。
 *
 * @param bus 目标 OSPI bus。
 * @param polling 自动轮询配置。
 * @param timeout_ms 超时时间，单位为毫秒。
 * @return 返回 BSP OSPI 状态码。
 */
bsp_ospi_status_t bsp_ospi_auto_polling(bsp_ospi_bus_t *bus,
                                        const bsp_ospi_polling_t *polling,
                                        uint32_t timeout_ms);
/**
 * @brief 在指定 OSPI bus 上进入 memory mapped 模式。
 *
 * 调用前通常需要先发送 memory mapped 使用的读命令配置。
 *
 * @param bus 目标 OSPI bus。
 * @param cfg memory mapped 配置。
 * @return 返回 BSP OSPI 状态码。
 */
bsp_ospi_status_t bsp_ospi_memory_mapped(bsp_ospi_bus_t *bus,
                                         const bsp_ospi_memory_mapped_cfg_t *cfg);

/**
 * @brief 发送轮询命令并立即执行自动轮询。
 *
 * @param bus 目标 OSPI bus。
 * @param cmd 轮询前需要发送的命令。
 * @param polling 自动轮询配置。
 * @param timeout_ms 超时时间，单位为毫秒。
 * @return 返回 BSP OSPI 状态码。
 */
bsp_ospi_status_t bsp_ospi_poll_command(bsp_ospi_bus_t *bus,
                                        const bsp_ospi_cmd_t *cmd,
                                        const bsp_ospi_polling_t *polling,
                                        uint32_t timeout_ms);
/**
 * @brief 发送 memory mapped 配置命令并进入 memory mapped 模式。
 *
 * @param bus 目标 OSPI bus。
 * @param cmd memory mapped 前需要发送的读命令配置。
 * @param cfg memory mapped 配置。
 * @param timeout_ms 命令阶段超时时间，单位为毫秒。
 * @return 返回 BSP OSPI 状态码。
 */
bsp_ospi_status_t bsp_ospi_enter_memory_mapped(bsp_ospi_bus_t *bus,
                                               const bsp_ospi_cmd_t *cmd,
                                               const bsp_ospi_memory_mapped_cfg_t *cfg,
                                               uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_BSP_OSPI_H

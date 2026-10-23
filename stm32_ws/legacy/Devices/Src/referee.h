/**
 ******************************************************************************
 * @file    referee.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   RoboMaster 裁判系统设备接口。
 *
 * @note    本文件定义裁判系统 UART 接收、协议状态缓存和发送接口。
 * @note    本层只描述裁判系统收到了什么、能发送什么协议帧，不绑定 UI 画面或控制任务。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_REFEREE_H
#define PRINTK_DEVICE_REFEREE_H

#include "robot_board.h"
#include "referee_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 设备头文件描述协议事实、配置结构、状态和诊断，具体接口绑定由 Robot 配置展开。
 * ========================================================================== */
/** @brief 比赛状态有效位。 */
#define REFEREE_VALID_GAME_STATUS ((uint32_t)(1UL << 0))

/** @brief 比赛结果有效位。 */
#define REFEREE_VALID_GAME_RESULT ((uint32_t)(1UL << 1))

/** @brief 机器人血量有效位。 */
#define REFEREE_VALID_GAME_ROBOT_HP ((uint32_t)(1UL << 2))

/** @brief 场地事件有效位。 */
#define REFEREE_VALID_EVENT_DATA ((uint32_t)(1UL << 3))

/** @brief 裁判警告有效位。 */
#define REFEREE_VALID_WARNING ((uint32_t)(1UL << 4))

/** @brief 飞镖信息有效位。 */
#define REFEREE_VALID_DART_INFO ((uint32_t)(1UL << 5))

/** @brief 本机器人状态有效位。 */
#define REFEREE_VALID_ROBOT_STATUS ((uint32_t)(1UL << 6))

/** @brief 功率热量有效位。 */
#define REFEREE_VALID_POWER_HEAT ((uint32_t)(1UL << 7))

/** @brief 机器人位置有效位。 */
#define REFEREE_VALID_ROBOT_POS ((uint32_t)(1UL << 8))

/** @brief 增益状态有效位。 */
#define REFEREE_VALID_BUFF ((uint32_t)(1UL << 9))

/** @brief 受击数据有效位。 */
#define REFEREE_VALID_HURT ((uint32_t)(1UL << 10))

/** @brief 发射数据有效位。 */
#define REFEREE_VALID_SHOOT_DATA ((uint32_t)(1UL << 11))

/** @brief 弹丸允许量有效位。 */
#define REFEREE_VALID_PROJECTILE ((uint32_t)(1UL << 12))

/** @brief RFID 状态有效位。 */
#define REFEREE_VALID_RFID ((uint32_t)(1UL << 13))

/** @brief 飞镖客户端命令有效位。 */
#define REFEREE_VALID_DART_CLIENT ((uint32_t)(1UL << 14))

/** @brief 地面机器人位置有效位。 */
#define REFEREE_VALID_GROUND_POS ((uint32_t)(1UL << 15))

/** @brief 雷达标记有效位。 */
#define REFEREE_VALID_RADAR_MARK ((uint32_t)(1UL << 16))

/** @brief 哨兵信息有效位。 */
#define REFEREE_VALID_SENTRY_INFO ((uint32_t)(1UL << 17))

/** @brief 雷达信息有效位。 */
#define REFEREE_VALID_RADAR_INFO ((uint32_t)(1UL << 18))

/** @brief 机器人交互数据有效位。 */
#define REFEREE_VALID_INTERACT ((uint32_t)(1UL << 19))

/** @brief 自定义机器人数据有效位。 */
#define REFEREE_VALID_CUSTOM_ROBOT ((uint32_t)(1UL << 20))

/** @brief 小地图命令有效位。 */
#define REFEREE_VALID_MAP_COMMAND ((uint32_t)(1UL << 21))

/** @brief 遥控控制数据有效位。 */
#define REFEREE_VALID_RC_DATA ((uint32_t)(1UL << 22))

/** @brief 小地图机器人数据有效位。 */
#define REFEREE_VALID_MAP_ROBOT ((uint32_t)(1UL << 23))

/** @brief 自定义客户端数据有效位。 */
#define REFEREE_VALID_CUSTOM_CLIENT ((uint32_t)(1UL << 24))

/** @brief 小地图路径数据有效位。 */
#define REFEREE_VALID_MAP_DATA ((uint32_t)(1UL << 25))

/** @brief 自定义字符信息有效位。 */
#define REFEREE_VALID_CUSTOM_INFO ((uint32_t)(1UL << 26))

/** @brief 机器人自定义数据有效位。 */
#define REFEREE_VALID_ROBOT_CUSTOM ((uint32_t)(1UL << 27))

/** @brief 裁判系统配置。 */
typedef struct
{
    bsp_uart_id_t uart_id;  /**< 绑定的 UART BSP 标识。 */
    uint32_t baud;          /**< UART 波特率。 */
    uint32_t tx_timeout_ms; /**< 阻塞发送超时时间，单位 ms。 */
} RefereeCfg;

/** @brief 裁判系统状态缓存。 */
typedef struct
{
    RefereeGameStatus game_status;       /**< 比赛状态。 */
    RefereeGameResult game_result;       /**< 比赛结果。 */
    RefereeGameRobotHp game_robot_hp;    /**< 机器人血量。 */
    RefereeEventData event_data;         /**< 场地事件。 */
    RefereeWarning warning;              /**< 裁判警告。 */
    RefereeDartInfo dart_info;           /**< 飞镖信息。 */
    RefereeRobotStatus robot_status;     /**< 本机器人状态。 */
    RefereePowerHeat power_heat;         /**< 功率热量。 */
    RefereeRobotPos robot_pos;           /**< 机器人位置。 */
    RefereeBuff buff;                    /**< 增益状态。 */
    RefereeHurt hurt;                    /**< 受击数据。 */
    RefereeShootData shoot_data;         /**< 发射数据。 */
    RefereeProjectile projectile;        /**< 弹丸允许量。 */
    RefereeRfid rfid;                    /**< RFID 状态。 */
    RefereeDartClient dart_client;       /**< 飞镖客户端命令。 */
    RefereeGroundPos ground_pos;         /**< 地面机器人位置。 */
    RefereeRadarMark radar_mark;         /**< 雷达标记。 */
    RefereeSentryInfo sentry_info;       /**< 哨兵信息。 */
    RefereeRadarInfo radar_info;         /**< 雷达信息。 */
    RefereeRobotInteract interact;       /**< 机器人交互数据。 */
    RefereeCustomRobot custom_robot;     /**< 自定义机器人数据。 */
    RefereeMapCommand map_command;       /**< 小地图命令。 */
    RefereeRcData rc_data; /**< 自定义控制器数据。 */
    RefereeMapRobot map_robot;           /**< 小地图机器人数据。 */
    RefereeCustomClient custom_client;   /**< 自定义客户端数据。 */
    RefereeMapData map_data;             /**< 小地图路径数据。 */
    RefereeCustomInfo custom_info;       /**< 自定义字符信息。 */
    RefereeRobotCustom robot_custom;     /**< 机器人自定义数据。 */
    uint32_t valid_mask;                 /**< REFEREE_VALID_* 有效位。 */
    uint32_t last_rx_tick;               /**< 最近一次合法帧接收 tick，单位 ms。 */
    uint16_t last_cmd_id;                /**< 最近一次合法帧命令 ID。 */
    uint8_t last_seq;                    /**< 最近一次合法帧序号。 */
} RefereeState;

/** @brief 裁判系统诊断信息。 */
typedef struct
{
    uint32_t rx_event_count;      /**< UART idle 事件次数。 */
    uint32_t frame_count;         /**< 合法帧数量。 */
    uint32_t crc_error_count;     /**< CRC 错误数量。 */
    uint32_t length_error_count;  /**< 长度错误数量。 */
    uint32_t unknown_cmd_count;   /**< 未处理命令数量。 */
    uint32_t tx_count;            /**< 发送成功次数。 */
    uint32_t tx_error_count;      /**< 发送失败次数。 */
    uint32_t restart_fail_count;  /**< DMA 接收重启失败次数。 */
    uint32_t error_count;         /**< UART 错误事件次数。 */
    uint32_t last_event_tick;     /**< 最近一次 UART 事件 tick。 */
    uint32_t last_valid_tick;     /**< 最近一次合法帧 tick。 */
    uint16_t last_size;           /**< 最近一次 idle 接收长度。 */
    uint16_t last_frame_len;      /**< 最近一次合法帧长度。 */
    uint16_t last_cmd_id;         /**< 最近一次合法帧命令 ID。 */
    uint8_t last_restart_status;  /**< 最近一次重启 DMA 的 HAL 状态。 */
    uint8_t last_tx_status;       /**< 最近一次发送的 HAL 状态。 */
    uint8_t ready;                /**< 设备是否已初始化。 */
    uint8_t uart_rx_state;        /**< UART HAL RxState 快照。 */
    uint8_t uart_g_state;         /**< UART HAL gState 快照。 */
    uint32_t uart_error_code;     /**< UART HAL ErrorCode 快照。 */
    uint8_t dma_state;            /**< RX DMA 状态快照。 */
    uint32_t dma_error_code;      /**< RX DMA 错误码快照。 */
    uint16_t dma_left;            /**< RX DMA 剩余计数。 */
} RefereeDiag;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口只表达设备能做什么，不把遥控器、裁判系统或电机解释成具体机器人控制动作。
 * ========================================================================== */

/**
 * @brief 初始化裁判系统设备。
 *
 * @return 执行状态。
 */
RefereeStatus referee_init(void);

/**
 * @brief 处理裁判系统 UART idle 接收事件。
 *
 * @param size 本次接收长度，单位 byte。
 */
void referee_rx_event(uint16_t size);

/**
 * @brief 向裁判系统状态机喂入字节流。
 *
 * @param data 输入字节流。
 * @param len 输入长度。
 * @param tick_ms 当前时间戳，单位 ms。
 * @return 执行状态。
 */
RefereeStatus referee_feed(const uint8_t *data, uint16_t len, uint32_t tick_ms);

/**
 * @brief 获取裁判系统状态快照。
 *
 * @param out 状态快照输出。
 * @return 执行状态。
 */
RefereeStatus referee_get(RefereeState *out);

/**
 * @brief 获取裁判系统诊断快照。
 *
 * @param out 诊断快照输出。
 * @return 执行状态。
 */
RefereeStatus referee_diag(RefereeDiag *out);

/**
 * @brief 判断裁判系统数据是否在线。
 *
 * @param timeout_ms 超时时间，0 表示只检查机器人 ID 是否有效。
 * @param now_ms 当前时间戳，单位 ms。
 * @return 在线返回 1，否则返回 0。
 */
uint8_t referee_online(uint32_t timeout_ms, uint32_t now_ms);

/**
 * @brief 发送一帧裁判系统数据。
 *
 * @param cmd_id 命令 ID。
 * @param payload 数据段。
 * @param payload_len 数据段长度。
 * @return 执行状态。
 */
RefereeStatus referee_send(uint16_t cmd_id, const uint8_t *payload, uint16_t payload_len);

/**
 * @brief 发送 0x0301 机器人交互数据。
 *
 * @param sender_id 发送者机器人 ID。
 * @param receiver_id 接收者客户端 ID。
 * @param data_cmd_id 交互子内容 ID。
 * @param data 用户载荷。
 * @param data_len 用户载荷长度。
 * @return 执行状态。
 */
RefereeStatus referee_send_interact(uint16_t sender_id,
                                    uint16_t receiver_id,
                                    uint16_t data_cmd_id,
                                    const uint8_t *data,
                                    uint16_t data_len);

/**
 * @brief 获取本机器人 ID。
 *
 * @return 本机器人 ID，未知时返回 0。
 */
uint8_t referee_robot_id(void);

/**
 * @brief 获取最近一次合法帧接收 tick。
 *
 * @return 最近一次合法帧 tick，单位 ms。
 */
uint32_t referee_last_rx_tick(void);

/**
 * @brief 获取底盘功率预算。
 *
 * 本函数只返回裁判系统可靠提供的功率上限和缓冲能量，不伪造实时功率测量。
 *
 * @param limit 底盘功率上限输出。
 * @param buffer 缓冲能量输出。
 * @return 执行状态。
 */
RefereeStatus referee_chassis_budget(float *limit, float *buffer);

/**
 * @brief 获取发射热量上限和当前热量。
 *
 * @param shooter 17mm 发射机构编号，1 或 2；3 表示 42mm。
 * @param heat_limit 热量上限输出。
 * @param heat 当前热量输出。
 * @return 执行状态。
 */
RefereeStatus referee_shoot_heat(uint8_t shooter, uint16_t *heat_limit, uint16_t *heat);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_DEVICE_REFEREE_H

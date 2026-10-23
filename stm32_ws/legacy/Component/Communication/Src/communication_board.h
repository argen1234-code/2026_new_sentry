/**
 ******************************************************************************
 * @file    communication_board.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   板间 RT 通信缓存与发送接口。
 *
 * @note    本文件只描述云台板和底盘板之间的紧凑状态帧，不读取任何 App 全局。
 * @note    物理 UART、任务周期和是否启用均由上层配置注入。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_COMMUNICATION_BOARD_H
#define PRINTK_COMPONENT_COMMUNICATION_BOARD_H

#include "communication_common.h"
#include "Generate/proto2_rt.h"
#include "bsp_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */

/**
 * @brief 板间通信配置。
 */
typedef struct
{
    bsp_uart_id_t uart_id; /**< 板间通信使用的 UART BSP 标识。 */
    uint8_t enable;        /**< 非 0 表示启用板间通信通道。 */
} CommunicationBoardCfg;

/**
 * @brief 云台板发往底盘板的状态快照。
 */
typedef struct
{
    float yaw_motor_rad;  /**< yaw 电机相对角，单位 rad。 */
    float ins_yaw_rad;    /**< INS yaw 连续角，单位 rad。 */
    float ins_pitch_rad;  /**< INS pitch，单位 rad。 */
    float ins_roll_rad;   /**< INS roll，单位 rad。 */
    uint8_t gimbal_mode;  /**< 云台当前模式。 */
    uint8_t shoot_mode;   /**< 发射当前模式。 */
    int16_t rc_ch0;       /**< 哨兵/特殊分支保留的板间 RC 通道 0。 */
    int16_t rc_ch1;       /**< 哨兵/特殊分支保留的板间 RC 通道 1。 */
    int16_t rc_ch2;       /**< 哨兵/特殊分支保留的板间 RC 通道 2。 */
    int16_t rc_ch3;       /**< 哨兵/特殊分支保留的板间 RC 通道 3。 */
    uint8_t rc_sw0;       /**< 哨兵/特殊分支保留的板间拨杆 0。 */
    uint8_t rc_sw1;       /**< 哨兵/特殊分支保留的板间拨杆 1。 */
    uint8_t status;       /**< 上层定义的状态 bit。 */
    uint32_t time_us;     /**< 本地时间戳低 32 位，单位 us。 */
    uint32_t seq;         /**< 本地缓存递增序号。 */
} CommunicationBoardState;

/**
 * @brief 底盘板发往云台板的状态快照。
 */
typedef struct
{
    float yaw_rad;     /**< 底盘 yaw，单位 rad。 */
    float pitch_rad;   /**< 底盘 pitch，单位 rad。 */
    float roll_rad;    /**< 底盘 roll，单位 rad。 */
    uint16_t robot_id; /**< 底盘侧维护的机器人 ID。 */
    uint8_t status;    /**< 上层定义的状态 bit。 */
    uint32_t time_us;  /**< 本地时间戳低 32 位，单位 us。 */
    uint32_t seq;      /**< 本地缓存递增序号。 */
} CommunicationChassisState;

/**
 * @brief 板间通信诊断快照。
 */
typedef struct
{
    uint8_t enabled;             /**< 非 0 表示通道配置为启用。 */
    uint8_t ready;               /**< 非 0 表示 UART 已完成绑定并开始接收。 */
    uint8_t last_status;         /**< 最近一次组件状态码。 */
    uint8_t last_restart_status; /**< 最近一次 DMA idle 接收重启状态。 */
    uint8_t last_msg_id;         /**< 最近一次合法消息 ID。 */
    uint16_t last_rx_size;       /**< 最近一次 idle 收到的字节数。 */
    uint32_t rx_events;          /**< UART idle 接收事件次数。 */
    uint32_t rx_frames;          /**< Proto2 RT 合法帧数量。 */
    uint32_t crc_errors;         /**< CRC 错误数量。 */
    uint32_t id_errors;          /**< 未知消息 ID 数量。 */
    uint32_t seq_errors;         /**< 序号不连续次数。 */
    uint32_t direction_errors;   /**< 方向错误数量。 */
    uint32_t decode_errors;      /**< payload 解码错误次数。 */
    uint32_t uart_errors;        /**< UART 错误事件次数。 */
    uint32_t board_count;        /**< CommunicationBoardState 接收次数。 */
    uint32_t chassis_count;      /**< CommunicationChassisState 接收次数。 */
    uint32_t tx_frames;          /**< 发送成功帧数。 */
    uint32_t tx_errors;          /**< 发送失败次数。 */
    uint32_t restart_fail_count; /**< DMA idle 接收重启失败次数。 */
} CommunicationBoardDiag;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 填充板间通信默认配置。
 *
 * 默认关闭板间通信，避免在未明确配置独立 UART 时占用已有裁判或 host 通道。
 *
 * @param cfg 配置输出。
 * @return 返回通信组件状态码。
 */
CommunicationStatus communication_board_default_cfg(CommunicationBoardCfg *cfg);

/**
 * @brief 初始化板间通信通道。
 *
 * @param cfg 板间通信配置。
 * @return 返回通信组件状态码。
 */
CommunicationStatus communication_board_init(const CommunicationBoardCfg *cfg);

/**
 * @brief 处理一次板间 UART DMA idle 接收事件。
 *
 * @param size 本次 DMA idle 收到的字节数。
 * @return 返回通信组件状态码。
 */
CommunicationStatus communication_board_rx_event(uint16_t size);

/**
 * @brief 直接喂入板间协议字节流。
 *
 * @param data 字节流首地址。
 * @param length 字节流长度，单位 byte。
 * @return 返回通信组件状态码。
 */
CommunicationStatus communication_board_feed(const uint8_t *data, uint16_t length);

/**
 * @brief 读取最近一次云台板状态快照。
 *
 * @param out 状态输出。
 * @return 返回通信组件状态码。
 */
CommunicationStatus communication_board_get_state(CommunicationBoardState *out);

/**
 * @brief 读取最近一次底盘板状态快照。
 *
 * @param out 状态输出。
 * @return 返回通信组件状态码。
 */
CommunicationStatus communication_board_get_chassis(CommunicationChassisState *out);

/**
 * @brief 发送云台板状态快照。
 *
 * @param state 状态载荷。
 * @return 返回通信组件状态码。
 */
CommunicationStatus communication_board_send_state(const CommunicationBoardState *state);

/**
 * @brief 发送底盘板状态快照。
 *
 * @param state 状态载荷。
 * @return 返回通信组件状态码。
 */
CommunicationStatus communication_board_send_chassis(const CommunicationChassisState *state);

/**
 * @brief 读取板间通信诊断快照。
 *
 * @param out 诊断输出。
 * @return 返回通信组件状态码。
 */
CommunicationStatus communication_board_diag(CommunicationBoardDiag *out);

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_COMPONENT_COMMUNICATION_BOARD_H */

/**
 ******************************************************************************
 * @file    rc.h
 * @author  GPT
 * @version V1.1.0
 * @date    2026-05-19
 * @brief   遥控器设备公共接口。
 *
 * @note    本文件定义遥控器硬件接收、原始协议解析后的物理输入状态和基础工具。
 * @note    本层只描述遥控器发来了什么，不绑定云台、底盘、发射等机器人控制语义。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_RC_H
#define PRINTK_DEVICE_RC_H

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
/** @brief 遥控器有效原始帧长度，单位 byte。 */
#define RC_FRAME_LEN ((uint16_t)18U)

/** @brief DBUS 原始帧长度，单位 byte。 */
#define RC_DBUS_LEN ((uint16_t)18U)

/** @brief SBUS 原始帧长度，单位 byte。 */
#define RC_SBUS_LEN ((uint16_t)25U)

/** @brief DBUS 参考转发帧长度，单位 byte。 */
#define RC_TX_LEN ((uint16_t)20U)

/** @brief 遥控器通道中值偏移。 */
#define RC_CH_OFFSET ((int16_t)1024)

/** @brief 遥控器通道绝对值错误阈值。 */
#define RC_CH_ERR_VALUE ((int16_t)700)

/** @brief 三档拨杆上档值。 */
#define RC_SW_UP ((uint8_t)1U)

/** @brief 三档拨杆中档值。 */
#define RC_SW_MID ((uint8_t)3U)

/** @brief 三档拨杆下档值。 */
#define RC_SW_DOWN ((uint8_t)2U)

/** @brief 键盘 W 键位掩码。 */
#define RC_KEY_W ((uint16_t)(1U << 0))

/** @brief 键盘 S 键位掩码。 */
#define RC_KEY_S ((uint16_t)(1U << 1))

/** @brief 键盘 A 键位掩码。 */
#define RC_KEY_A ((uint16_t)(1U << 2))

/** @brief 键盘 D 键位掩码。 */
#define RC_KEY_D ((uint16_t)(1U << 3))

/** @brief 键盘 Shift 键位掩码。 */
#define RC_KEY_SHIFT ((uint16_t)(1U << 4))

/** @brief 键盘 Ctrl 键位掩码。 */
#define RC_KEY_CTRL ((uint16_t)(1U << 5))

/** @brief 键盘 Q 键位掩码。 */
#define RC_KEY_Q ((uint16_t)(1U << 6))

/** @brief 键盘 E 键位掩码。 */
#define RC_KEY_E ((uint16_t)(1U << 7))

/** @brief 键盘 R 键位掩码。 */
#define RC_KEY_R ((uint16_t)(1U << 8))

/** @brief 键盘 F 键位掩码。 */
#define RC_KEY_F ((uint16_t)(1U << 9))

/** @brief 键盘 G 键位掩码。 */
#define RC_KEY_G ((uint16_t)(1U << 10))

/** @brief 键盘 Z 键位掩码。 */
#define RC_KEY_Z ((uint16_t)(1U << 11))

/** @brief 键盘 X 键位掩码。 */
#define RC_KEY_X ((uint16_t)(1U << 12))

/** @brief 键盘 C 键位掩码。 */
#define RC_KEY_C ((uint16_t)(1U << 13))

/** @brief 键盘 V 键位掩码。 */
#define RC_KEY_V ((uint16_t)(1U << 14))

/** @brief 键盘 B 键位掩码。 */
#define RC_KEY_B ((uint16_t)(1U << 15))

/**
 * @brief 遥控器协议解析状态。
 */
typedef enum
{
    RcOk = 0,       /**< 解析成功。 */
    RcNull = 1,     /**< 输入指针为空。 */
    RcLen = 2,      /**< 帧长度非法。 */
    RcFrameErr = 3, /**< 帧头、帧尾或结构非法。 */
    RcValue = 4     /**< 解析后的物理输入值越界。 */
} RcStatus;

/**
 * @brief 遥控器原始帧快照。
 */
typedef struct
{
    uint8_t data[RC_FRAME_LEN]; /**< 最近一次有效原始帧。 */
    uint32_t tick_ms;           /**< 最近一次有效原始帧时间戳，单位 ms tick。 */
    uint32_t seq;               /**< 原始帧递增序号。 */
    uint16_t size;              /**< 原始帧长度，单位 byte。 */
} RcFrame;

/**
 * @brief 遥控器接收诊断快照。
 */
typedef struct
{
    uint32_t rx_event_count;     /**< UART idle 接收事件次数。 */
    uint32_t valid_count;        /**< 成功提交的原始帧数。 */
    uint32_t invalid_size_count; /**< 帧长不符合遥控原始帧长度的次数。 */
    uint32_t error_count;        /**< UART 错误事件次数。 */
    uint32_t restart_fail_count; /**< DMA idle 接收重启失败次数。 */
    uint32_t last_event_tick;    /**< 最近一次 idle 事件 tick。 */
    uint32_t last_valid_tick;    /**< 最近一次有效原始帧 tick。 */
    uint16_t last_size;          /**< 最近一次 idle 接收长度。 */
    uint16_t dma_left;           /**< 当前 DMA 剩余接收计数。 */
    uint32_t uart_error_code;    /**< 当前 UART HAL 错误码。 */
    uint32_t dma_error_code;     /**< 当前 DMA HAL 错误码。 */
    uint8_t uart_rx_state;       /**< 当前 UART RxState。 */
    uint8_t uart_g_state;        /**< 当前 UART gState。 */
    uint8_t dma_state;           /**< 当前 DMA State。 */
    uint8_t last_event_type;     /**< 最近一次 HAL UART RX 事件类型。 */
    uint8_t ready;               /**< 设备初始化完成标志。 */
    uint8_t last_frame_ok;       /**< 最近一次接收是否提交为有效原始帧。 */
    uint8_t last_restart_status; /**< 最近一次重启接收的 HAL 状态。 */
} RcDiag;

/**
 * @brief DBUS 风格物理输入状态。
 */
typedef struct
{
    struct
    {
        int16_t ch[5];  /**< 摇杆和附加通道，均以 0 为中值。 */
        uint8_t sw[2];  /**< 两个三档拨杆，取值为 RC_SW_*。 */
    } rc;               /**< 摇杆、拨杆物理输入。 */

    struct
    {
        int16_t x;       /**< 鼠标 X 轴增量。 */
        int16_t y;       /**< 鼠标 Y 轴增量。 */
        int16_t z;       /**< 鼠标滚轮增量。 */
        uint8_t press_l; /**< 鼠标左键状态。 */
        uint8_t press_r; /**< 鼠标右键状态。 */
    } mouse;             /**< 鼠标物理输入。 */

    struct
    {
        uint16_t v; /**< 键盘按键位图。 */
    } key;          /**< 键盘物理输入。 */

    uint32_t last_update_tick; /**< 最近一次有效状态更新时间，单位 ms tick。 */
    uint8_t online;            /**< 非 0 表示输入源在线。 */
} RcDbusState;

/**
 * @brief SBUS 原始物理输入状态。
 */
typedef struct
{
    uint16_t online;             /**< SBUS 在线标志。 */
    uint32_t sbus_receiver_tick; /**< 最近一次 SBUS 接收时间，单位 ms tick。 */

    struct
    {
        int16_t ch[10]; /**< SBUS 10 路原始通道。 */
    } rc;               /**< SBUS 原始通道。 */

    struct
    {
        int16_t left_vert;  /**< 左摇杆竖直控制量。 */
        int16_t left_hori;  /**< 左摇杆水平控制量。 */
        int16_t right_vert; /**< 右摇杆竖直控制量。 */
        int16_t right_hori; /**< 右摇杆水平控制量。 */
    } joy;                  /**< SBUS 摇杆物理输入。 */

    struct
    {
        uint8_t swa; /**< 预留拨杆 A。 */
        uint8_t swb; /**< 预留拨杆 B。 */
        uint8_t swc; /**< 预留拨杆 C。 */
        uint8_t swd; /**< 预留拨杆 D。 */
    } toggle;        /**< 预留拨杆状态。 */

    struct
    {
        float a; /**< 预留模拟量 A。 */
        float b; /**< 预留模拟量 B。 */
    } var;       /**< 预留模拟量。 */

    struct
    {
        uint8_t l; /**< 预留左键状态。 */
        uint8_t r; /**< 预留右键状态。 */
    } key;         /**< 预留按键状态。 */
} RcSbusState;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口只表达设备能做什么，不把遥控器、裁判系统或电机解释成具体机器人控制动作。
 * ========================================================================== */

/**
 * @brief 初始化遥控器 UART DMA idle 接收。
 *
 * 调用前必须完成 CubeMX UART/DMA 初始化，并先调用 bsp_uart_init()。
 *
 * @return 返回 HAL UART 状态码。
 */
HAL_StatusTypeDef rc_init(void);

/**
 * @brief 遥控器 UART DMA idle 接收事件处理。
 *
 * @param size 本次 idle 接收长度，单位 byte。
 */
void rc_rx_event(uint16_t size);

/**
 * @brief 遥控器 UART 错误事件处理。
 */
void rc_error_event(void);

/**
 * @brief 获取遥控器原始帧快照。
 *
 * @param out 原始帧输出指针。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t rc_frame(RcFrame *out);

/**
 * @brief 获取遥控器接收诊断快照。
 *
 * @param out 诊断输出指针。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t rc_diag(RcDiag *out);

/**
 * @brief 清空 DBUS 风格物理输入状态。
 *
 * @param state 状态指针。
 */
void rc_dbus_state_clear(RcDbusState *state);

/**
 * @brief 判断拨杆值是否合法。
 *
 * @param sw 拨杆值。
 * @return 合法返回 1，否则返回 0。
 */
uint8_t rc_sw_valid(uint8_t sw);

/**
 * @brief 判断 DBUS 风格物理输入是否处于有效范围。
 *
 * @param state 状态指针。
 * @return 有效返回 1，否则返回 0。
 */
uint8_t rc_dbus_state_valid(const RcDbusState *state);

/**
 * @brief 判断拨杆是否为上档。
 *
 * @param sw 拨杆值。
 * @return 是上档返回 1，否则返回 0。
 */
uint8_t rc_sw_is_up(uint8_t sw);

/**
 * @brief 判断拨杆是否为中档。
 *
 * @param sw 拨杆值。
 * @return 是中档返回 1，否则返回 0。
 */
uint8_t rc_sw_is_mid(uint8_t sw);

/**
 * @brief 判断拨杆是否为下档。
 *
 * @param sw 拨杆值。
 * @return 是下档返回 1，否则返回 0。
 */
uint8_t rc_sw_is_down(uint8_t sw);

/**
 * @brief 判断指定按键是否按下。
 *
 * @param key 当前按键位图。
 * @param mask 目标按键掩码。
 * @return 按下返回 1，否则返回 0。
 */
uint8_t rc_key_down(uint16_t key, uint16_t mask);

/**
 * @brief 对摇杆输入应用死区。
 *
 * @param input 原始输入。
 * @param deadband 死区绝对值。
 * @return 死区处理后的输入。
 */
int16_t rc_deadband(int16_t input, int16_t deadband);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_DEVICE_RC_H

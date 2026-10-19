/**
 ******************************************************************************
 * @file    input_shoot_rc.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   RC 到发射参考的映射接口。
 *
 * @note    本文件把遥控器物理输入解释为发射人工参考，单发上升沿状态保存在上下文中。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_INPUT_SHOOT_RC_H
#define PRINTK_COMPONENT_INPUT_SHOOT_RC_H

#include "input_common.h"
#include "rc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */

/**
 * @brief RC 发射映射配置。
 */
typedef struct
{
    int8_t mode_sw;             /**< 发射模式拨杆索引，范围 0~1，负数表示禁用。 */
    uint8_t toggle_sw;          /**< 摩擦轮拨杆切换档位，通常为 RC_SW_UP。 */
    uint8_t fire_sw;            /**< 拨杆发射档位，通常为 RC_SW_DOWN。 */
    uint16_t fric_on_key;       /**< 摩擦轮开启键位掩码，0 表示禁用。 */
    uint16_t fric_off_key;      /**< 摩擦轮关闭键位掩码，0 表示禁用。 */
    uint16_t fire_once_key;     /**< 单发键位掩码，0 表示禁用。 */
    uint16_t fire_hold_key;     /**< 连发键位掩码，0 表示禁用。 */
    uint16_t mag_open_key;      /**< 弹舱打开键位掩码，0 表示禁用。 */
    uint16_t mag_close_key;     /**< 弹舱关闭键位掩码，0 表示禁用。 */
    uint16_t press_long_time;   /**< 鼠标长按进入连发的周期数。 */
    uint16_t switch_long_time;  /**< 拨杆长按进入连发的周期数。 */
    int8_t mag_mode_sw;         /**< 弹舱动作拨杆索引，范围 0~1，负数表示禁用。 */
    int8_t mag_guard_sw;        /**< 弹舱拨杆动作保护拨杆索引，范围 0~1，负数表示禁用。 */
    uint8_t mag_guard_value;    /**< 弹舱拨杆动作保护拨杆目标档位。 */
    uint8_t mag_open_sw;        /**< 弹舱打开对应的拨杆档位。 */
    uint8_t mag_close_sw;       /**< 弹舱关闭对应的拨杆档位。 */
    uint8_t once_mouse_l;       /**< 非 0 表示鼠标左键上升沿作为单发输入。 */
    uint8_t hold_mouse_l;       /**< 非 0 表示鼠标左键长按作为连发输入。 */
    uint8_t hold_mouse_r;       /**< 非 0 表示鼠标右键长按作为连发输入。 */
    uint8_t priority;           /**< 输出参考优先级。 */
} InputShootRcMap;

/**
 * @brief RC 发射映射上下文。
 */
typedef struct
{
    uint16_t press_l_time; /**< 鼠标左键长按计数。 */
    uint16_t press_r_time; /**< 鼠标右键长按计数。 */
    uint16_t switch_time;  /**< 发射拨杆长按计数。 */
    uint16_t last_key;     /**< 上一次键盘按键位图。 */
    uint8_t last_switch;   /**< 上一次拨杆值。 */
    uint8_t fric_enable;   /**< 摩擦轮请求锁存状态。 */
    uint8_t last_once;     /**< 上一次单发物理输入状态。 */
    uint8_t initialized;   /**< 非 0 表示上下文已经记录过拨杆初值。 */
} InputShootRcCtx;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 初始化 RC 发射映射上下文。
 *
 * @param ctx 映射上下文。
 */
void input_shoot_rc_init(InputShootRcCtx *ctx);

/**
 * @brief 将 RC 物理输入映射为发射参考。
 *
 * @param ctx 映射上下文。
 * @param rc DBUS 风格物理输入状态。
 * @param map 映射配置。
 * @param out 发射参考输出。
 * @param mag 弹舱参考输出，可为空。
 * @return 执行状态。
 */
InputStatus input_shoot_rc_map(InputShootRcCtx *ctx,
                               const RcDbusState *rc,
                               const InputShootRcMap *map,
                               InputShootRef *out,
                               InputMagRef *mag);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_INPUT_SHOOT_RC_H

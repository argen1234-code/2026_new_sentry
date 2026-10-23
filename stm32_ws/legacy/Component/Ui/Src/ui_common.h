/**
 ******************************************************************************
 * @file    ui_common.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   UI 组件内部公共工具接口。
 *
 * @note    本文件只提供图元命名、字符串长度和脏标记比较工具。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_UI_COMMON_H
#define PRINTK_COMPONENT_UI_COMMON_H

#include "ui.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 按数值 ID 写入裁判客户端图元名称。
 *
 * @param name 3 byte 图元名称输出。
 * @param id 数值 ID，只使用低 24 bit。
 */
void ui_name_set(uint8_t name[UI_NAME_LEN], uint32_t id);

/**
 * @brief 清空图元载荷。
 *
 * @param graphic 图元载荷。
 */
void ui_graphic_clear(UiGraphic *graphic);

/**
 * @brief 清空字符串载荷。
 *
 * @param string 字符串载荷。
 */
void ui_string_clear(UiString *string);

/**
 * @brief 比较两个图元载荷是否一致。
 *
 * @param a 图元 A。
 * @param b 图元 B。
 * @return 一致返回 1，否则返回 0。
 */
uint8_t ui_graphic_equal(const UiGraphic *a, const UiGraphic *b);

/**
 * @brief 比较两个字符串载荷是否一致。
 *
 * @param a 字符串 A。
 * @param b 字符串 B。
 * @return 一致返回 1，否则返回 0。
 */
uint8_t ui_string_equal(const UiString *a, const UiString *b);

/**
 * @brief 设置字符串载荷内容。
 *
 * 输入字符串长度必须不超过 UI_TEXT_LEN，不做静默截断。
 *
 * @param string 字符串载荷。
 * @param text 输入字符串。
 * @return 执行状态。
 */
UiStatus ui_string_set_text(UiString *string, const char *text);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_UI_COMMON_H

/**
 ******************************************************************************
 * @file    ui.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   裁判客户端 UI 组件 C++ 包装。
 *
 * @note    本文件只做类型别名和 inline 转发，运行时实现全部位于 C 文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_UI_HPP
#define PRINTK_COMPONENT_UI_HPP

#include "ui.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只包装组件 C API 和类型别名，不写运行时控制逻辑。
 * ========================================================================== */
namespace Component::Ui {

/** @brief UI 执行状态别名。 */
using Status = ::UiStatus;

/** @brief UI 图元操作别名。 */
using Op = ::UiOp;

/** @brief UI 图元类型别名。 */
using FigureType = ::UiFigureType;

/** @brief UI 图元颜色别名。 */
using Color = ::UiColor;

/** @brief UI 图元载荷别名。 */
using Graphic = ::UiGraphic;

/** @brief UI 字符串载荷别名。 */
using String = ::UiString;

/** @brief UI 配置别名。 */
using Config = ::UiCfg;

/** @brief UI 诊断信息别名。 */
using Diag = ::UiDiag;

/** @brief UI 上下文别名。 */
using Context = ::Ui;

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 默认图元槽位数量。 */
inline constexpr uint8_t FigureMax = UI_FIGURE_MAX_DEFAULT;

/** @brief 默认字符串槽位数量。 */
inline constexpr uint8_t StringMax = UI_STRING_MAX_DEFAULT;

/** @brief 默认字符串最大长度。 */
inline constexpr uint8_t TextLen = UI_TEXT_LEN;

/** @brief 默认客户端 ID 偏移。 */
inline constexpr uint16_t ClientIdOffset = UI_CLIENT_ID_OFFSET;

/** @brief 初始化 UI 组件。 */
inline Status init(Context &ui, const Config &cfg)
{
    return ui_init(&ui, &cfg);
}

/** @brief 复位 UI 组件。 */
inline Status reset(Context &ui)
{
    return ui_reset(&ui);
}

/** @brief 设置一个图元槽位。 */
inline Status set_graphic(Context &ui, uint8_t index, const Graphic &graphic)
{
    return ui_set_graphic(&ui, index, &graphic);
}

/** @brief 设置一个字符串槽位。 */
inline Status set_string(Context &ui, uint8_t index, const String &string)
{
    return ui_set_string(&ui, index, &string);
}

/** @brief 设置删除图层命令。 */
inline Status delete_layer(Context &ui, uint8_t delete_type, uint8_t layer)
{
    return ui_delete_layer(&ui, delete_type, layer);
}

/** @brief 发送全部待发送 UI 数据。 */
inline Status flush(Context &ui)
{
    return ui_flush(&ui);
}

/** @brief 执行一次 UI 发送周期。 */
inline Status step(Context &ui)
{
    return ui_step(&ui);
}

/** @brief 获取 UI 诊断快照。 */
inline Status diag(const Context &ui, Diag &out)
{
    return ui_diag(&ui, &out);
}

} // namespace Component::Ui

#endif // PRINTK_COMPONENT_UI_HPP

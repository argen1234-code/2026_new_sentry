/**
 ******************************************************************************
 * @file    ui_referee.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   UI 组件裁判系统发送适配接口。
 *
 * @note    本文件只给 Component/Ui 内部使用，负责把 UI 载荷发送为 0x0301 交互帧。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_UI_REFEREE_H
#define PRINTK_COMPONENT_UI_REFEREE_H

#include "ui.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */
/** @brief 删除图层交互子内容 ID。 */
#define UI_DATA_DELETE 0x0100U

/** @brief 单图元交互子内容 ID。 */
#define UI_DATA_GRAPHIC_1 0x0101U

/** @brief 双图元交互子内容 ID。 */
#define UI_DATA_GRAPHIC_2 0x0102U

/** @brief 五图元交互子内容 ID。 */
#define UI_DATA_GRAPHIC_5 0x0103U

/** @brief 七图元交互子内容 ID。 */
#define UI_DATA_GRAPHIC_7 0x0104U

/** @brief 字符串交互子内容 ID。 */
#define UI_DATA_STRING 0x0110U

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 发送删除图层命令。
 *
 * @param ui UI 组件上下文。
 * @return 执行状态。
 */
UiStatus ui_referee_send_delete(Ui *ui);

/**
 * @brief 发送一组图元。
 *
 * @param ui UI 组件上下文。
 * @param indices 图元槽位索引数组。
 * @param count 图元数量，只允许 1、2、5、7。
 * @return 执行状态。
 */
UiStatus ui_referee_send_graphics(Ui *ui, const uint8_t *indices, uint8_t count);

/**
 * @brief 发送一个字符串图元。
 *
 * @param ui UI 组件上下文。
 * @param index 字符串槽位索引。
 * @return 执行状态。
 */
UiStatus ui_referee_send_string(Ui *ui, uint8_t index);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_UI_REFEREE_H

/**
 ******************************************************************************
 * @file    ui_hud_g.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   默认云台 HUD 页面实现。
 *
 * @note    本文件迁移简版 HUD 静态布局，不包含状态面板。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "ui_hud_g.h"

#include "ui_common.h"

#include <stddef.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具只处理参数、限幅、wrap、矩阵工作区等实现细节，不读取任务或设备全局。
 * ========================================================================== */

/**
 * @brief 填充默认图元公共字段。
 *
 * @param graphic 图元输出。
 * @param id 图元 ID。
 * @param type 图元类型。
 */
static void ui_hud_graphic_base(UiGraphic *graphic, uint32_t id, UiFigureType type)
{
    ui_graphic_clear(graphic);
    ui_name_set(graphic->name, id);
    graphic->op = (uint32_t)UiOpAdd;
    graphic->type = (uint32_t)type;
    graphic->layer = 0U;
    graphic->color = (uint32_t)UiColorTeam;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
UiStatus ui_hud_g_init(Ui *ui)
{
    UiGraphic auto_aim;
    UiGraphic cross;
    UiStatus status;

    if (ui == NULL) {
        return UiNull;
    }

    ui_hud_graphic_base(&auto_aim, UI_HUD_G_AUTO_AIM_INDEX, UiFigureRect);
    auto_aim.start_x = 649U;
    auto_aim.start_y = 203U;
    auto_aim.width = 1U;
    auto_aim.end_x = 1249U;
    auto_aim.end_y = 803U;

    status = ui_set_graphic(ui, UI_HUD_G_AUTO_AIM_INDEX, &auto_aim);
    if (status != UiOk) {
        return status;
    }

    ui_hud_graphic_base(&cross, UI_HUD_G_CROSS_INDEX, UiFigureCircle);
    cross.start_x = 958U;
    cross.start_y = 441U;
    cross.width = 5U;
    cross.c = 15U;

    return ui_set_graphic(ui, UI_HUD_G_CROSS_INDEX, &cross);
}

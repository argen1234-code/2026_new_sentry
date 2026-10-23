/**
 ******************************************************************************
 * @file    ui.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   裁判客户端 UI 组件实现。
 *
 * @note    本文件只管理 UI 本地状态、脏标记和发送调度，不访问 USART、RTOS 或任务层。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "ui.h"

#include "Src/ui_common.h"
#include "Src/ui_hud_g.h"
#include "Src/ui_referee.h"

#include <string.h>

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
 * @brief 统计待发送图元数量。
 *
 * @param ui UI 组件上下文。
 * @return 待发送图元数量。
 */
static uint16_t ui_dirty_graphic_count(const Ui *ui)
{
    uint16_t count = 0U;
    uint8_t i;

    if (ui == NULL) {
        return 0U;
    }

    for (i = 0U; i < UI_FIGURE_MAX_DEFAULT; i++) {
        if ((ui->graphic_used[i] != 0U) && (ui->graphic_dirty[i] != 0U)) {
            count++;
        }
    }

    return count;
}

/**
 * @brief 统计待发送字符串数量。
 *
 * @param ui UI 组件上下文。
 * @return 待发送字符串数量。
 */
static uint16_t ui_dirty_string_count(const Ui *ui)
{
    uint16_t count = 0U;
    uint8_t i;

    if (ui == NULL) {
        return 0U;
    }

    for (i = 0U; i < UI_STRING_MAX_DEFAULT; i++) {
        if ((ui->string_used[i] != 0U) && (ui->string_dirty[i] != 0U)) {
            count++;
        }
    }

    return count;
}

/**
 * @brief 统计待发送包数量。
 *
 * @param ui UI 组件上下文。
 * @return 待发送对象数量。
 */
static uint16_t ui_pending_count(const Ui *ui)
{
    uint16_t count;

    if (ui == NULL) {
        return 0U;
    }

    count = ui_dirty_graphic_count(ui);
    count = (uint16_t)(count + ui_dirty_string_count(ui));
    if (ui->delete_pending != 0U) {
        count++;
    }

    return count;
}

/* ==========================================================================
 * 输入、状态更新与控制步骤
 * --------------------------------------------------------------------------
 * 这一组把注入的输入转换成组件输出；具体调度和硬件发送仍由 App 层负责。
 * ========================================================================== */

/**
 * @brief 更新诊断中的 pending 快照。
 *
 * @param ui UI 组件上下文。
 */
static void ui_update_pending_diag(Ui *ui)
{
    if (ui == NULL) {
        return;
    }

    ui->diag.dirty_graphic_count = ui_dirty_graphic_count(ui);
    ui->diag.dirty_string_count = ui_dirty_string_count(ui);
    ui->diag.delete_pending = ui->delete_pending;
}

/**
 * @brief 将新增操作切换为后续修改操作。
 *
 * @param graphic 图元载荷。
 */
static void ui_graphic_after_send(UiGraphic *graphic)
{
    if (graphic == NULL) {
        return;
    }

    if (graphic->op == (uint32_t)UiOpAdd) {
        graphic->op = (uint32_t)UiOpModify;
    }
}

/**
 * @brief 将新增字符串操作切换为后续修改操作。
 *
 * @param string 字符串载荷。
 */
static void ui_string_after_send(UiString *string)
{
    if (string == NULL) {
        return;
    }

    if (string->op == (uint32_t)UiOpAdd) {
        string->op = (uint32_t)UiOpModify;
    }
}

/**
 * @brief 选择本次图元分包大小。
 *
 * @param remain 剩余待发送图元数量。
 * @return 本次分包图元数量。
 */
static uint8_t ui_graphic_pack_size(uint16_t remain)
{
    if (remain > 5U) {
        return 7U;
    }
    if (remain > 2U) {
        return 5U;
    }
    if (remain > 1U) {
        return 2U;
    }
    return 1U;
}

/**
 * @brief 发送一个图元分包。
 *
 * @param ui UI 组件上下文。
 * @return 执行状态。
 */
static UiStatus ui_send_graphic_pack(Ui *ui)
{
    uint8_t indices[UI_PACKET_FIG_MAX];
    uint16_t remain;
    uint8_t pack_size;
    uint8_t count = 0U;
    uint8_t i;
    UiStatus status;

    remain = ui_dirty_graphic_count(ui);
    if (remain == 0U) {
        return UiOk;
    }

    pack_size = ui_graphic_pack_size(remain);
    for (i = 0U; (i < UI_FIGURE_MAX_DEFAULT) && (count < pack_size); i++) {
        if ((ui->graphic_used[i] != 0U) && (ui->graphic_dirty[i] != 0U)) {
            indices[count++] = i;
        }
    }

    status = ui_referee_send_graphics(ui, indices, count);
    if (status != UiOk) {
        ui_update_pending_diag(ui);
        return status;
    }

    for (i = 0U; i < count; i++) {
        ui->graphic_dirty[indices[i]] = 0U;
        ui_graphic_after_send(&ui->graphics[indices[i]]);
    }

    ui_update_pending_diag(ui);
    return UiOk;
}

/**
 * @brief 发送一个字符串分包。
 *
 * @param ui UI 组件上下文。
 * @return 执行状态。
 */
static UiStatus ui_send_string_pack(Ui *ui)
{
    uint8_t i;
    UiStatus status;

    for (i = 0U; i < UI_STRING_MAX_DEFAULT; i++) {
        if ((ui->string_used[i] == 0U) || (ui->string_dirty[i] == 0U)) {
            continue;
        }

        status = ui_referee_send_string(ui, i);
        if (status != UiOk) {
            ui_update_pending_diag(ui);
            return status;
        }

        ui->string_dirty[i] = 0U;
        ui_string_after_send(&ui->strings[i]);
        ui_update_pending_diag(ui);
        return UiOk;
    }

    return UiOk;
}

/**
 * @brief 发送一个删除命令。
 *
 * @param ui UI 组件上下文。
 * @return 执行状态。
 */
static UiStatus ui_send_delete(Ui *ui)
{
    UiStatus status;

    if (ui->delete_pending == 0U) {
        return UiOk;
    }

    status = ui_referee_send_delete(ui);
    if (status != UiOk) {
        ui_update_pending_diag(ui);
        return status;
    }

    ui->delete_pending = 0U;
    ui_update_pending_diag(ui);
    return UiOk;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
UiStatus ui_init(Ui *ui, const UiCfg *cfg)
{
    UiStatus status;

    if ((ui == NULL) || (cfg == NULL)) {
        return UiNull;
    }

    if ((cfg->sender_id == 0U) || (cfg->receiver_id == 0U)) {
        return UiArg;
    }

    memset(ui, 0, sizeof(*ui));
    ui->cfg = *cfg;
    if (ui->cfg.send_per_step == 0U) {
        ui->cfg.send_per_step = UI_SEND_PER_STEP_DEFAULT;
    }
    ui->initialized = 1U;

    status = ui_hud_g_init(ui);
    ui_update_pending_diag(ui);
    if (status != UiOk) {
        ui->initialized = 0U;
    }

    return status;
}

UiStatus ui_reset(Ui *ui)
{
    UiCfg cfg;

    if (ui == NULL) {
        return UiNull;
    }

    if (ui->initialized == 0U) {
        return UiBadState;
    }

    cfg = ui->cfg;
    return ui_init(ui, &cfg);
}

UiStatus ui_set_graphic(Ui *ui, uint8_t index, const UiGraphic *graphic)
{
    UiGraphic next;

    if ((ui == NULL) || (graphic == NULL)) {
        return UiNull;
    }

    if (ui->initialized == 0U) {
        return UiBadState;
    }

    if (index >= UI_FIGURE_MAX_DEFAULT) {
        return UiArg;
    }

    next = *graphic;
    if (next.op == (uint32_t)UiOpNone) {
        next.op = (ui->graphic_used[index] == 0U) ? (uint32_t)UiOpAdd : (uint32_t)UiOpModify;
    }

    if ((ui->graphic_used[index] == 0U) || (ui_graphic_equal(&ui->graphics[index], &next) == 0U)) {
        ui->graphics[index] = next;
        ui->graphic_used[index] = 1U;
        ui->graphic_dirty[index] = 1U;
        ui->diag.set_graphic_count++;
        ui_update_pending_diag(ui);
    }

    return UiOk;
}

UiStatus ui_set_string(Ui *ui, uint8_t index, const UiString *string)
{
    UiString next;

    if ((ui == NULL) || (string == NULL)) {
        return UiNull;
    }

    if (ui->initialized == 0U) {
        return UiBadState;
    }

    if (index >= UI_STRING_MAX_DEFAULT) {
        return UiArg;
    }

    if (string->str_len > UI_TEXT_LEN) {
        return UiLen;
    }

    next = *string;
    next.type = (uint32_t)UiFigureString;
    if (next.op == (uint32_t)UiOpNone) {
        next.op = (ui->string_used[index] == 0U) ? (uint32_t)UiOpAdd : (uint32_t)UiOpModify;
    }

    if ((ui->string_used[index] == 0U) || (ui_string_equal(&ui->strings[index], &next) == 0U)) {
        ui->strings[index] = next;
        ui->string_used[index] = 1U;
        ui->string_dirty[index] = 1U;
        ui->diag.set_string_count++;
        ui_update_pending_diag(ui);
    }

    return UiOk;
}

UiStatus ui_delete_layer(Ui *ui, uint8_t delete_type, uint8_t layer)
{
    if (ui == NULL) {
        return UiNull;
    }

    if (ui->initialized == 0U) {
        return UiBadState;
    }

    ui->delete_type = delete_type;
    ui->delete_layer = layer;
    ui->delete_pending = 1U;
    ui->diag.delete_count++;
    ui_update_pending_diag(ui);

    return UiOk;
}

UiStatus ui_flush(Ui *ui)
{
    UiStatus status;

    if (ui == NULL) {
        return UiNull;
    }

    if (ui->initialized == 0U) {
        return UiBadState;
    }

    while (ui_pending_count(ui) != 0U) {
        status = ui_step(ui);
        if (status != UiOk) {
            return status;
        }
    }

    return UiOk;
}

UiStatus ui_step(Ui *ui)
{
    UiStatus status;
    uint8_t sent = 0U;
    uint8_t limit;

    if (ui == NULL) {
        return UiNull;
    }

    if (ui->initialized == 0U) {
        return UiBadState;
    }

    limit = ui->cfg.send_per_step;
    if (limit == 0U) {
        limit = UI_SEND_PER_STEP_DEFAULT;
    }

    while ((sent < limit) && (ui_pending_count(ui) != 0U)) {
        if (ui->delete_pending != 0U) {
            status = ui_send_delete(ui);
        } else if (ui_dirty_graphic_count(ui) != 0U) {
            status = ui_send_graphic_pack(ui);
        } else {
            status = ui_send_string_pack(ui);
        }

        if (status != UiOk) {
            return status;
        }
        sent++;
    }

    ui_update_pending_diag(ui);
    return UiOk;
}

UiStatus ui_diag(const Ui *ui, UiDiag *out)
{
    UiDiag diag;

    if ((ui == NULL) || (out == NULL)) {
        return UiNull;
    }

    diag = ui->diag;
    diag.dirty_graphic_count = ui_dirty_graphic_count(ui);
    diag.dirty_string_count = ui_dirty_string_count(ui);
    diag.delete_pending = ui->delete_pending;
    *out = diag;

    return UiOk;
}

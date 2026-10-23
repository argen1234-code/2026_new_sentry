/**
 ******************************************************************************
 * @file    ui_referee.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   UI 组件裁判系统发送适配实现。
 *
 * @note    本文件不打包完整裁判帧，不复制 CRC 表，完整帧由 Devices/Referee 完成。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "ui_referee.h"

#include "referee.h"

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
 * @brief 将图元数量映射为交互子内容 ID。
 *
 * @param count 图元数量。
 * @param data_cmd_id 交互子内容 ID 输出。
 * @return 执行状态。
 */
static UiStatus ui_graphic_cmd(uint8_t count, uint16_t *data_cmd_id)
{
    if (data_cmd_id == NULL) {
        return UiNull;
    }

    switch (count) {
    case 1U:
        *data_cmd_id = UI_DATA_GRAPHIC_1;
        return UiOk;
    case 2U:
        *data_cmd_id = UI_DATA_GRAPHIC_2;
        return UiOk;
    case 5U:
        *data_cmd_id = UI_DATA_GRAPHIC_5;
        return UiOk;
    case 7U:
        *data_cmd_id = UI_DATA_GRAPHIC_7;
        return UiOk;
    default:
        return UiArg;
    }
}

/**
 * @brief 将裁判系统状态映射为 UI 状态。
 *
 * @param status 裁判系统状态。
 * @return UI 状态。
 */
static UiStatus ui_from_referee(RefereeStatus status)
{
    switch (status) {
    case RefereeOk:
        return UiOk;
    case RefereeNull:
        return UiNull;
    case RefereeLen:
    case RefereeOverflow:
        return UiLen;
    case RefereeArg:
        return UiArg;
    case RefereeBadState:
        return UiBadState;
    case RefereeIo:
    default:
        return UiIo;
    }
}

/* ==========================================================================
 * 输入、状态更新与控制步骤
 * --------------------------------------------------------------------------
 * 这一组把注入的输入转换成组件输出；具体调度和硬件发送仍由 App 层负责。
 * ========================================================================== */

/**
 * @brief 记录一次发送结果。
 *
 * @param ui UI 组件上下文。
 * @param status UI 状态。
 * @param data_cmd_id 交互子内容 ID。
 * @param payload_len 0x0301 数据段总长度。
 */
static void ui_record_tx(Ui *ui, UiStatus status, uint16_t data_cmd_id, uint16_t payload_len)
{
    if (ui == NULL) {
        return;
    }

    ui->diag.last_status = (uint8_t)status;
    ui->diag.last_data_cmd_id = data_cmd_id;
    ui->diag.last_payload_len = payload_len;
    if (status == UiOk) {
        ui->diag.tx_count++;
    } else {
        ui->diag.tx_error_count++;
    }
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
UiStatus ui_referee_send_delete(Ui *ui)
{
    uint8_t payload[2];
    RefereeStatus ref_status;
    UiStatus status;

    if (ui == NULL) {
        return UiNull;
    }

    payload[0] = ui->delete_type;
    payload[1] = ui->delete_layer;
    ref_status = referee_send_interact(ui->cfg.sender_id,
                                       ui->cfg.receiver_id,
                                       UI_DATA_DELETE,
                                       payload,
                                       (uint16_t)sizeof(payload));
    status = ui_from_referee(ref_status);
    ui_record_tx(ui, status, UI_DATA_DELETE, (uint16_t)(REFEREE_INTERACT_HEAD_SIZE + sizeof(payload)));

    return status;
}

UiStatus ui_referee_send_graphics(Ui *ui, const uint8_t *indices, uint8_t count)
{
    uint8_t payload[UI_PACKET_FIG_MAX * UI_GRAPHIC_SIZE];
    uint16_t data_cmd_id = 0U;
    RefereeStatus ref_status;
    UiStatus status;
    uint8_t i;

    if ((ui == NULL) || (indices == NULL)) {
        return UiNull;
    }

    status = ui_graphic_cmd(count, &data_cmd_id);
    if (status != UiOk) {
        return status;
    }

    memset(payload, 0, sizeof(payload));
    for (i = 0U; i < count; i++) {
        if (indices[i] >= UI_FIGURE_MAX_DEFAULT) {
            return UiArg;
        }
        memcpy(&payload[(uint16_t)i * UI_GRAPHIC_SIZE], &ui->graphics[indices[i]], sizeof(UiGraphic));
    }

    ref_status = referee_send_interact(ui->cfg.sender_id,
                                       ui->cfg.receiver_id,
                                       data_cmd_id,
                                       payload,
                                       (uint16_t)(count * UI_GRAPHIC_SIZE));
    status = ui_from_referee(ref_status);
    ui_record_tx(ui,
                 status,
                 data_cmd_id,
                 (uint16_t)(REFEREE_INTERACT_HEAD_SIZE + (uint16_t)(count * UI_GRAPHIC_SIZE)));

    return status;
}

UiStatus ui_referee_send_string(Ui *ui, uint8_t index)
{
    RefereeStatus ref_status;
    UiStatus status;

    if (ui == NULL) {
        return UiNull;
    }

    if (index >= UI_STRING_MAX_DEFAULT) {
        return UiArg;
    }

    ref_status = referee_send_interact(ui->cfg.sender_id,
                                       ui->cfg.receiver_id,
                                       UI_DATA_STRING,
                                       (const uint8_t *)&ui->strings[index],
                                       UI_STRING_SIZE);
    status = ui_from_referee(ref_status);
    ui_record_tx(ui, status, UI_DATA_STRING, (uint16_t)(REFEREE_INTERACT_HEAD_SIZE + UI_STRING_SIZE));

    return status;
}

/**
 ******************************************************************************
 * @file    ui_common.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   UI 组件内部公共工具实现。
 *
 * @note    本文件不访问裁判系统设备，只处理本地 UI 协议载荷。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "ui_common.h"

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
 * @brief 计算不超过 UI_TEXT_LEN 的字符串长度。
 *
 * @param text 输入字符串。
 * @param len 字符串长度输出。
 * @return 执行状态。
 */
static UiStatus ui_text_len(const char *text, uint8_t *len)
{
    uint8_t i;

    if ((text == NULL) || (len == NULL)) {
        return UiNull;
    }

    for (i = 0U; i < UI_TEXT_LEN; i++) {
        if (text[i] == '\0') {
            *len = i;
            return UiOk;
        }
    }

    if (text[UI_TEXT_LEN] == '\0') {
        *len = UI_TEXT_LEN;
        return UiOk;
    }

    return UiLen;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
void ui_name_set(uint8_t name[UI_NAME_LEN], uint32_t id)
{
    if (name == NULL) {
        return;
    }

    name[0] = (uint8_t)((id >> 16) & 0xFFU);
    name[1] = (uint8_t)((id >> 8) & 0xFFU);
    name[2] = (uint8_t)(id & 0xFFU);
}

void ui_graphic_clear(UiGraphic *graphic)
{
    if (graphic == NULL) {
        return;
    }

    memset(graphic, 0, sizeof(*graphic));
}

void ui_string_clear(UiString *string)
{
    if (string == NULL) {
        return;
    }

    memset(string, 0, sizeof(*string));
}

uint8_t ui_graphic_equal(const UiGraphic *a, const UiGraphic *b)
{
    if ((a == NULL) || (b == NULL)) {
        return 0U;
    }

    return (memcmp(a, b, sizeof(*a)) == 0) ? 1U : 0U;
}

uint8_t ui_string_equal(const UiString *a, const UiString *b)
{
    if ((a == NULL) || (b == NULL)) {
        return 0U;
    }

    return (memcmp(a, b, sizeof(*a)) == 0) ? 1U : 0U;
}

UiStatus ui_string_set_text(UiString *string, const char *text)
{
    UiStatus status;
    uint8_t len = 0U;

    if ((string == NULL) || (text == NULL)) {
        return UiNull;
    }

    status = ui_text_len(text, &len);
    if (status != UiOk) {
        return status;
    }

    memset(string->text, 0, sizeof(string->text));
    if (len != 0U) {
        memcpy(string->text, text, len);
    }
    string->str_len = len;
    string->type = (uint32_t)UiFigureString;

    return UiOk;
}

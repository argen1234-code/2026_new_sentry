/**
 ******************************************************************************
 * @file    input_gimbal_vision.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   视觉到云台参考的映射实现。
 *
 * @note    本文件只把外部注入的视觉状态转换为统一云台参考。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "input_gimbal_vision.h"

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
InputStatus input_gimbal_vision_map(const InputVisionState *state, InputGimbalRef *out)
{
    if ((state == NULL) || (out == NULL)) {
        return InputNull;
    }

    input_gimbal_clear(out);
    out->priority = state->priority;
    out->source = InputSourceVision;
    out->online = state->online;
    out->force_stop = state->force_stop;
    out->tick_ms = state->tick_ms;

    if ((state->online == 0U) || (state->force_stop != 0U)) {
        return InputOk;
    }

    if ((state->valid_mask & INPUT_GIMBAL_YAW_VALID) != 0U) {
        input_axis_set(&out->yaw, state->yaw, state->priority, InputSourceVision, state->tick_ms);
        out->valid_mask |= INPUT_GIMBAL_YAW_VALID;
    }
    if ((state->valid_mask & INPUT_GIMBAL_PITCH_VALID) != 0U) {
        input_axis_set(&out->pitch, state->pitch, state->priority, InputSourceVision, state->tick_ms);
        out->valid_mask |= INPUT_GIMBAL_PITCH_VALID;
    }
    if (((state->valid_mask & INPUT_GIMBAL_MODE_VALID) != 0U) || (state->mode_valid != 0U)) {
        out->mode = state->mode;
        out->mode_valid = 1U;
        out->valid_mask |= INPUT_GIMBAL_MODE_VALID;
    }

    return InputOk;
}

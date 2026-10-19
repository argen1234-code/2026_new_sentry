/**
 ******************************************************************************
 * @file    input_chassis_slam.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   SLAM 到底盘参考的映射实现。
 *
 * @note    本文件只把外部注入的 SLAM 状态转换为统一底盘参考。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "input_chassis_slam.h"

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
InputStatus input_chassis_slam_map(const InputSlamState *state, InputChassisRef *out)
{
    if ((state == NULL) || (out == NULL)) {
        return InputNull;
    }

    input_chassis_clear(out);
    out->priority = state->priority;
    out->source = InputSourceSlam;
    out->online = state->online;
    out->force_stop = state->force_stop;
    out->jump = state->jump;
    out->tick_ms = state->tick_ms;

    if ((state->online == 0U) || (state->force_stop != 0U)) {
        return InputOk;
    }

    if ((state->valid_mask & INPUT_CHASSIS_VX_VALID) != 0U) {
        input_axis_set(&out->vx, state->vx, state->priority, InputSourceSlam, state->tick_ms);
        out->valid_mask |= INPUT_CHASSIS_VX_VALID;
    }
    if ((state->valid_mask & INPUT_CHASSIS_VY_VALID) != 0U) {
        input_axis_set(&out->vy, state->vy, state->priority, InputSourceSlam, state->tick_ms);
        out->valid_mask |= INPUT_CHASSIS_VY_VALID;
    }
    if ((state->valid_mask & INPUT_CHASSIS_WZ_VALID) != 0U) {
        input_axis_set(&out->wz, state->wz, state->priority, InputSourceSlam, state->tick_ms);
        out->valid_mask |= INPUT_CHASSIS_WZ_VALID;
    }
    if ((state->valid_mask & INPUT_CHASSIS_YAW_VALID) != 0U) {
        input_axis_set(&out->yaw_ref, state->yaw_ref, state->priority, InputSourceSlam, state->tick_ms);
        out->valid_mask |= INPUT_CHASSIS_YAW_VALID;
    }
    if ((state->valid_mask & INPUT_CHASSIS_FRAME_VALID) != 0U) {
        input_axis_set(&out->frame_angle, state->frame_angle, state->priority, InputSourceSlam, state->tick_ms);
        out->valid_mask |= INPUT_CHASSIS_FRAME_VALID;
    }
    if ((state->valid_mask & INPUT_CHASSIS_LEG_VALID) != 0U) {
        input_axis_set(&out->leg_length, state->leg_length, state->priority, InputSourceSlam, state->tick_ms);
        out->valid_mask |= INPUT_CHASSIS_LEG_VALID;
    }
    if ((state->valid_mask & INPUT_CHASSIS_LEFT_VALID) != 0U) {
        input_axis_set(&out->leg_left, state->leg_left, state->priority, InputSourceSlam, state->tick_ms);
        out->valid_mask |= INPUT_CHASSIS_LEFT_VALID;
    }
    if ((state->valid_mask & INPUT_CHASSIS_RIGHT_VALID) != 0U) {
        input_axis_set(&out->leg_right, state->leg_right, state->priority, InputSourceSlam, state->tick_ms);
        out->valid_mask |= INPUT_CHASSIS_RIGHT_VALID;
    }
    if (((state->valid_mask & INPUT_CHASSIS_MODE_VALID) != 0U) || (state->mode_valid != 0U)) {
        out->mode = state->mode;
        out->mode_valid = 1U;
        out->valid_mask |= INPUT_CHASSIS_MODE_VALID;
    }

    return InputOk;
}

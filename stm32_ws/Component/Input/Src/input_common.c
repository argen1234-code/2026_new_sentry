/**
 ******************************************************************************
 * @file    input_common.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   统一输入参考公共语言实现。
 *
 * @note    本文件只实现参考清理、优先级仲裁和机构命令生成。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "input_common.h"

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
 * @brief 判断候选优先级是否应覆盖当前结果。
 *
 * @param old_valid 当前结果是否有效。
 * @param old_prio 当前结果优先级。
 * @param new_prio 候选结果优先级。
 * @return 应覆盖返回 1，否则返回 0。
 */
static uint8_t prio_take(uint8_t old_valid, uint8_t old_prio, uint8_t new_prio)
{
    if (old_valid == 0U) {
        return 1U;
    }

    return (new_prio >= old_prio) ? 1U : 0U;
}

/**
 * @brief 判断云台参考是否包含有效信息。
 *
 * @param ref 云台参考。
 * @return 包含有效信息返回 1，否则返回 0。
 */
static uint8_t gimbal_has_ref(const InputGimbalRef *ref)
{
    if (ref == NULL) {
        return 0U;
    }

    return ((ref->valid_mask != 0U) || (ref->mode_valid != 0U) || (ref->force_stop != 0U)) ? 1U : 0U;
}

/**
 * @brief 判断底盘参考是否包含有效信息。
 *
 * @param ref 底盘参考。
 * @return 包含有效信息返回 1，否则返回 0。
 */
static uint8_t chassis_has_ref(const InputChassisRef *ref)
{
    if (ref == NULL) {
        return 0U;
    }

    return ((ref->valid_mask != 0U) || (ref->mode_valid != 0U) || (ref->force_stop != 0U) || (ref->jump != 0U)) ? 1U : 0U;
}

/**
 * @brief 判断弹舱参考是否包含有效信息。
 *
 * @param ref 弹舱参考。
 * @return 包含有效信息返回 1，否则返回 0。
 */
static uint8_t mag_has_ref(const InputMagRef *ref)
{
    if (ref == NULL) {
        return 0U;
    }

    return ((ref->valid_mask != 0U) && (ref->action_valid != 0U)) ? 1U : 0U;
}

/**
 * @brief 判断校准参考是否包含有效信息。
 *
 * @param ref 校准参考。
 * @return 包含有效信息返回 1，否则返回 0。
 */
static uint8_t cali_has_ref(const InputCaliRef *ref)
{
    if (ref == NULL) {
        return 0U;
    }

    return ((ref->valid_mask != 0U) && (ref->action_valid != 0U)) ? 1U : 0U;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
void input_axis_clear(InputAxisRef *axis)
{
    if (axis == NULL) {
        return;
    }

    memset(axis, 0, sizeof(*axis));
    axis->source = InputSourceNone;
}

void input_axis_set(InputAxisRef *axis,
                    float32_t value,
                    uint8_t priority,
                    InputSource source,
                    uint32_t tick_ms)
{
    if (axis == NULL) {
        return;
    }

    axis->value = value;
    axis->valid = 1U;
    axis->priority = priority;
    axis->source = source;
    axis->tick_ms = tick_ms;
}

void input_axis_choose(InputAxisRef *dst, const InputAxisRef *src)
{
    if ((dst == NULL) || (src == NULL) || (src->valid == 0U)) {
        return;
    }

    if (prio_take(dst->valid, dst->priority, src->priority) != 0U) {
        *dst = *src;
    }
}

void input_gimbal_clear(InputGimbalRef *ref)
{
    if (ref == NULL) {
        return;
    }

    memset(ref, 0, sizeof(*ref));
    ref->mode = GimbalModeZero;
    ref->source = InputSourceNone;
}

void input_chassis_clear(InputChassisRef *ref)
{
    if (ref == NULL) {
        return;
    }

    memset(ref, 0, sizeof(*ref));
    ref->mode = ChassisModeStop;
    ref->source = InputSourceNone;
}

void input_shoot_clear(InputShootRef *ref)
{
    if (ref == NULL) {
        return;
    }

    memset(ref, 0, sizeof(*ref));
    ref->source = InputSourceNone;
}

void input_mag_clear(InputMagRef *ref)
{
    if (ref == NULL) {
        return;
    }

    memset(ref, 0, sizeof(*ref));
    ref->action = InputMagHold;
    ref->source = InputSourceNone;
}

void input_cali_clear(InputCaliRef *ref)
{
    if (ref == NULL) {
        return;
    }

    memset(ref, 0, sizeof(*ref));
    ref->action = InputCaliNone;
    ref->source = InputSourceNone;
}

void input_ref_clear(InputRef *ref)
{
    if (ref == NULL) {
        return;
    }

    memset(ref, 0, sizeof(*ref));
    input_gimbal_clear(&ref->gimbal);
    input_chassis_clear(&ref->chassis);
    input_shoot_clear(&ref->shoot);
    input_mag_clear(&ref->mag);
    input_cali_clear(&ref->cali);
}

void input_merge_gimbal(InputGimbalRef *dst, const InputGimbalRef *src)
{
    uint8_t mode_old_valid;

    if ((dst == NULL) || (src == NULL) || (gimbal_has_ref(src) == 0U)) {
        return;
    }

    input_axis_choose(&dst->yaw, &src->yaw);
    input_axis_choose(&dst->pitch, &src->pitch);
    dst->valid_mask = 0U;
    if (dst->yaw.valid != 0U) {
        dst->valid_mask |= INPUT_GIMBAL_YAW_VALID;
    }
    if (dst->pitch.valid != 0U) {
        dst->valid_mask |= INPUT_GIMBAL_PITCH_VALID;
    }

    mode_old_valid = dst->mode_valid;
    if ((src->mode_valid != 0U) && (prio_take(dst->mode_valid, dst->priority, src->priority) != 0U)) {
        dst->mode = src->mode;
        dst->mode_valid = 1U;
        if (src->priority > dst->priority) {
            dst->priority = src->priority;
        }
        dst->source = src->source;
        dst->valid_mask |= INPUT_GIMBAL_MODE_VALID;
    } else if (mode_old_valid != 0U) {
        dst->valid_mask |= INPUT_GIMBAL_MODE_VALID;
    }

    if (prio_take((uint8_t)(dst->online | dst->force_stop), dst->priority, src->priority) != 0U) {
        dst->online = src->online;
        dst->force_stop = src->force_stop;
        dst->priority = src->priority;
        dst->source = src->source;
        dst->tick_ms = src->tick_ms;
    }
}

void input_merge_chassis(InputChassisRef *dst, const InputChassisRef *src)
{
    uint8_t mode_old_valid;

    if ((dst == NULL) || (src == NULL) || (chassis_has_ref(src) == 0U)) {
        return;
    }

    input_axis_choose(&dst->vx, &src->vx);
    input_axis_choose(&dst->vy, &src->vy);
    input_axis_choose(&dst->wz, &src->wz);
    input_axis_choose(&dst->yaw_ref, &src->yaw_ref);
    input_axis_choose(&dst->frame_angle, &src->frame_angle);
    input_axis_choose(&dst->leg_length, &src->leg_length);
    input_axis_choose(&dst->leg_left, &src->leg_left);
    input_axis_choose(&dst->leg_right, &src->leg_right);

    dst->valid_mask = 0U;
    if (dst->vx.valid != 0U) {
        dst->valid_mask |= INPUT_CHASSIS_VX_VALID;
    }
    if (dst->vy.valid != 0U) {
        dst->valid_mask |= INPUT_CHASSIS_VY_VALID;
    }
    if (dst->wz.valid != 0U) {
        dst->valid_mask |= INPUT_CHASSIS_WZ_VALID;
    }
    if (dst->yaw_ref.valid != 0U) {
        dst->valid_mask |= INPUT_CHASSIS_YAW_VALID;
    }
    if (dst->frame_angle.valid != 0U) {
        dst->valid_mask |= INPUT_CHASSIS_FRAME_VALID;
    }
    if (dst->leg_length.valid != 0U) {
        dst->valid_mask |= INPUT_CHASSIS_LEG_VALID;
    }
    if (dst->leg_left.valid != 0U) {
        dst->valid_mask |= INPUT_CHASSIS_LEFT_VALID;
    }
    if (dst->leg_right.valid != 0U) {
        dst->valid_mask |= INPUT_CHASSIS_RIGHT_VALID;
    }

    mode_old_valid = dst->mode_valid;
    if ((src->mode_valid != 0U) && (prio_take(dst->mode_valid, dst->priority, src->priority) != 0U)) {
        dst->mode = src->mode;
        dst->mode_valid = 1U;
        if (src->priority > dst->priority) {
            dst->priority = src->priority;
        }
        dst->source = src->source;
        dst->valid_mask |= INPUT_CHASSIS_MODE_VALID;
    } else if (mode_old_valid != 0U) {
        dst->valid_mask |= INPUT_CHASSIS_MODE_VALID;
    }

    if (prio_take((uint8_t)(dst->online | dst->force_stop | dst->jump), dst->priority, src->priority) != 0U) {
        dst->online = src->online;
        dst->force_stop = src->force_stop;
        dst->jump = src->jump;
        dst->priority = src->priority;
        dst->source = src->source;
        dst->tick_ms = src->tick_ms;
    }
}

void input_merge_shoot(InputShootRef *dst, const InputShootRef *src)
{
    if ((dst == NULL) || (src == NULL) || (src->valid_mask == 0U)) {
        return;
    }

    if ((src->valid_mask & INPUT_SHOOT_FRIC_VALID) != 0U &&
        prio_take((uint8_t)((dst->valid_mask & INPUT_SHOOT_FRIC_VALID) != 0U), dst->priority, src->priority) != 0U) {
        dst->fric_enable = src->fric_enable;
        dst->valid_mask |= INPUT_SHOOT_FRIC_VALID;
    }

    if ((src->valid_mask & INPUT_SHOOT_ONCE_VALID) != 0U &&
        prio_take((uint8_t)((dst->valid_mask & INPUT_SHOOT_ONCE_VALID) != 0U), dst->priority, src->priority) != 0U) {
        dst->fire_once = src->fire_once;
        dst->valid_mask |= INPUT_SHOOT_ONCE_VALID;
    }

    if ((src->valid_mask & INPUT_SHOOT_HOLD_VALID) != 0U &&
        prio_take((uint8_t)((dst->valid_mask & INPUT_SHOOT_HOLD_VALID) != 0U), dst->priority, src->priority) != 0U) {
        dst->fire_hold = src->fire_hold;
        dst->valid_mask |= INPUT_SHOOT_HOLD_VALID;
    }

    if ((src->valid_mask & INPUT_SHOOT_STOP_VALID) != 0U &&
        prio_take((uint8_t)((dst->valid_mask & INPUT_SHOOT_STOP_VALID) != 0U), dst->priority, src->priority) != 0U) {
        dst->force_stop = src->force_stop;
        dst->valid_mask |= INPUT_SHOOT_STOP_VALID;
    }

    if (prio_take((uint8_t)(dst->online | dst->force_stop), dst->priority, src->priority) != 0U) {
        dst->online = src->online;
        dst->priority = src->priority;
        dst->source = src->source;
        dst->tick_ms = src->tick_ms;
    }
}

void input_merge_mag(InputMagRef *dst, const InputMagRef *src)
{
    if ((dst == NULL) || (src == NULL) || (mag_has_ref(src) == 0U)) {
        return;
    }

    if (prio_take(dst->action_valid, dst->priority, src->priority) != 0U) {
        dst->action = src->action;
        dst->action_valid = 1U;
        dst->online = src->online;
        dst->priority = src->priority;
        dst->source = src->source;
        dst->valid_mask |= INPUT_MAG_ACTION_VALID;
        dst->tick_ms = src->tick_ms;
    }
}

void input_merge_cali(InputCaliRef *dst, const InputCaliRef *src)
{
    if ((dst == NULL) || (src == NULL) || (cali_has_ref(src) == 0U)) {
        return;
    }

    if (prio_take(dst->action_valid, dst->priority, src->priority) != 0U) {
        dst->action = src->action;
        dst->action_valid = 1U;
        dst->online = src->online;
        dst->priority = src->priority;
        dst->source = src->source;
        dst->valid_mask |= INPUT_CALI_ACTION_VALID;
        dst->tick_ms = src->tick_ms;
    }
}

InputStatus input_make_gimbal(const InputGimbalRef *ref, GimbalCmd *cmd)
{
    if ((ref == NULL) || (cmd == NULL)) {
        return InputNull;
    }

    memset(cmd, 0, sizeof(*cmd));
    cmd->yaw = (ref->yaw.valid != 0U) ? ref->yaw.value : 0.0F;
    cmd->pitch = (ref->pitch.valid != 0U) ? ref->pitch.value : 0.0F;
    cmd->mode = (ref->mode_valid != 0U) ? ref->mode : GimbalModeZero;
    cmd->online = ((ref->online != 0U) && (ref->force_stop == 0U)) ? 1U : 0U;

    return InputOk;
}

InputStatus input_make_chassis(const InputChassisRef *ref, ChassisCmd *cmd)
{
    if ((ref == NULL) || (cmd == NULL)) {
        return InputNull;
    }

    memset(cmd, 0, sizeof(*cmd));
    cmd->online = ((ref->online != 0U) && (ref->force_stop == 0U)) ? 1U : 0U;
    cmd->force_stop = ref->force_stop;
    cmd->jump = ref->jump;
    cmd->mode = (ref->mode_valid != 0U) ? ref->mode : ChassisModeStop;
    cmd->vx = (ref->vx.valid != 0U) ? ref->vx.value : 0.0F;
    cmd->vy = (ref->vy.valid != 0U) ? ref->vy.value : 0.0F;
    cmd->wz = (ref->wz.valid != 0U) ? ref->wz.value : 0.0F;
    cmd->yaw_ref = (ref->yaw_ref.valid != 0U) ? ref->yaw_ref.value : 0.0F;
    cmd->frame_angle = (ref->frame_angle.valid != 0U) ? ref->frame_angle.value : 0.0F;
    cmd->leg_length = (ref->leg_length.valid != 0U) ? ref->leg_length.value : 0.0F;
    cmd->leg_left = (ref->leg_left.valid != 0U) ? ref->leg_left.value : 0.0F;
    cmd->leg_right = (ref->leg_right.valid != 0U) ? ref->leg_right.value : 0.0F;

    return InputOk;
}

InputStatus input_make_shoot(const InputShootRef *ref, ShootCmd *cmd)
{
    if ((ref == NULL) || (cmd == NULL)) {
        return InputNull;
    }

    memset(cmd, 0, sizeof(*cmd));
    cmd->online = ((ref->online != 0U) && (ref->force_stop == 0U)) ? 1U : 0U;
    cmd->force_stop = ref->force_stop;
    cmd->fric_enable = ((ref->valid_mask & INPUT_SHOOT_FRIC_VALID) != 0U) ? ref->fric_enable : 0U;
    cmd->fire_once = ((ref->valid_mask & INPUT_SHOOT_ONCE_VALID) != 0U) ? ref->fire_once : 0U;
    cmd->fire_hold = ((ref->valid_mask & INPUT_SHOOT_HOLD_VALID) != 0U) ? ref->fire_hold : 0U;

    return InputOk;
}

InputStatus input_make_mag(const InputMagRef *ref, InputMagCmd *cmd)
{
    if ((ref == NULL) || (cmd == NULL)) {
        return InputNull;
    }

    memset(cmd, 0, sizeof(*cmd));
    cmd->online = ref->online;
    if (((ref->valid_mask & INPUT_MAG_ACTION_VALID) != 0U) &&
        (ref->action_valid != 0U) &&
        (ref->online != 0U)) {
        cmd->action = ref->action;
        cmd->valid = 1U;
    } else {
        cmd->action = InputMagHold;
    }

    return InputOk;
}

InputStatus input_make_cali(const InputCaliRef *ref, InputCaliCmd *cmd)
{
    if ((ref == NULL) || (cmd == NULL)) {
        return InputNull;
    }

    memset(cmd, 0, sizeof(*cmd));
    cmd->online = ref->online;
    if (((ref->valid_mask & INPUT_CALI_ACTION_VALID) != 0U) &&
        (ref->action_valid != 0U) &&
        (ref->online != 0U)) {
        cmd->action = ref->action;
        cmd->valid = 1U;
    } else {
        cmd->action = InputCaliNone;
    }

    return InputOk;
}

/**
 ******************************************************************************
 * @file    input_gimbal_rc.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   RC 到云台参考的映射实现。
 *
 * @note    本文件只解释 RC 物理输入，不读取任务或机构全局状态。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "input_gimbal_rc.h"

#include "common.h"

#include <math.h>
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
 * @brief 判断通道索引是否可用。
 *
 * @param ch 通道索引。
 * @return 可用返回 1，否则返回 0。
 */
static uint8_t ch_valid(int8_t ch)
{
    return ((ch >= 0) && (ch < 5)) ? 1U : 0U;
}

/**
 * @brief 判断拨杆索引是否可用。
 *
 * @param sw 拨杆索引。
 * @return 可用返回 1，否则返回 0。
 */
static uint8_t sw_valid(int8_t sw)
{
    return ((sw >= 0) && (sw < 2)) ? 1U : 0U;
}

/**
 * @brief 判断键盘覆盖是否允许生效。
 *
 * @param rc DBUS 风格物理输入状态。
 * @param map 映射配置。
 * @return 允许返回 1，否则返回 0。
 */
static uint8_t override_allowed(const RcDbusState *rc, const InputGimbalRcMap *map)
{
    if ((rc == NULL) || (map == NULL)) {
        return 0U;
    }

    if (map->override_sw < 0) {
        return 1U;
    }

    if (sw_valid(map->override_sw) == 0U) {
        return 0U;
    }

    return rc_sw_is_down(rc->rc.sw[(uint8_t)map->override_sw]);
}

/**
 * @brief 按拨杆值选择云台模式。
 *
 * @param sw 拨杆值。
 * @param map 映射配置。
 * @param mode 模式输出。
 * @return 选择成功返回 1，否则返回 0。
 */
static uint8_t mode_from_sw(uint8_t sw, const InputGimbalRcMap *map, GimbalMode *mode)
{
    if ((map == NULL) || (mode == NULL)) {
        return 0U;
    }

    if (rc_sw_is_up(sw) != 0U) {
        *mode = map->sw_up_mode;
        return 1U;
    }
    if (rc_sw_is_mid(sw) != 0U) {
        *mode = map->sw_mid_mode;
        return 1U;
    }
    if (rc_sw_is_down(sw) != 0U) {
        *mode = map->sw_down_mode;
        return 1U;
    }

    return 0U;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
void input_gimbal_rc_init(InputGimbalRcCtx *ctx)
{
    if (ctx == NULL) {
        return;
    }

    (void)memset(ctx, 0, sizeof(*ctx));
}

/**
 * @brief 处理 180 度转向参考。
 *
 * @param ctx 映射上下文。
 * @param rc DBUS 风格物理输入状态。
 * @param map 映射配置。
 * @param yaw 当前 yaw 角。
 * @param yaw_valid yaw 是否有效。
 * @param yaw_value yaw 输出增量。
 */
static void turn_ref(InputGimbalRcCtx *ctx,
                     const RcDbusState *rc,
                     const InputGimbalRcMap *map,
                     float32_t yaw,
                     uint8_t yaw_valid,
                     float32_t *yaw_value)
{
    uint8_t turn_now;
    float32_t err;

    if ((ctx == NULL) || (rc == NULL) || (map == NULL) || (yaw_value == NULL)) {
        return;
    }

    if ((map->turn_key == 0U) || (yaw_valid == 0U)) {
        ctx->turn_active = 0U;
        ctx->last_key = rc->key.v;
        return;
    }

    turn_now = rc_key_down(rc->key.v, map->turn_key);
    if ((turn_now != 0U) && (rc_key_down(ctx->last_key, map->turn_key) == 0U)) {
        ctx->turn_target = angle_wrap_pi(yaw + COMMON_PI);
        ctx->turn_active = 1U;
    }
    ctx->last_key = rc->key.v;

    if (ctx->turn_active == 0U) {
        return;
    }

    err = angle_diff(ctx->turn_target, yaw);
    if (fabsf(err) < map->turn_done_err) {
        ctx->turn_active = 0U;
        return;
    }

    if (err > 0.0F) {
        *yaw_value += map->turn_speed;
    } else {
        *yaw_value -= map->turn_speed;
    }
}

InputStatus input_gimbal_rc_map(InputGimbalRcCtx *ctx,
                                const RcDbusState *rc,
                                const InputGimbalRcMap *map,
                                float32_t yaw,
                                uint8_t yaw_valid,
                                InputGimbalRef *out)
{
    float32_t yaw_value = 0.0F;
    float32_t pitch_value = 0.0F;
    GimbalMode mode;
    uint8_t yaw_cfg;
    uint8_t pitch_cfg;

    if ((ctx == NULL) || (rc == NULL) || (map == NULL) || (out == NULL)) {
        return InputNull;
    }

    input_gimbal_clear(out);
    out->priority = map->priority;
    out->source = InputSourceRc;
    out->tick_ms = rc->last_update_tick;

    if (rc->online == 0U) {
        out->force_stop = 1U;
        ctx->turn_active = 0U;
        ctx->last_key = 0U;
        return InputOk;
    }

    if (rc_dbus_state_valid(rc) == 0U) {
        return InputValue;
    }

    out->online = 1U;
    yaw_cfg = (uint8_t)((ch_valid(map->yaw_ch) != 0U) || (map->mouse_yaw_scale != 0.0F));
    if (ch_valid(map->yaw_ch) != 0U) {
        yaw_value += (float32_t)rc_deadband(rc->rc.ch[(uint8_t)map->yaw_ch], map->deadband) * map->yaw_scale;
    }
    yaw_value += (float32_t)rc->mouse.x * map->mouse_yaw_scale;
    if (yaw_cfg != 0U) {
        turn_ref(ctx, rc, map, yaw, yaw_valid, &yaw_value);
        input_axis_set(&out->yaw, yaw_value, map->priority, InputSourceRc, rc->last_update_tick);
        out->valid_mask |= INPUT_GIMBAL_YAW_VALID;
    }

    pitch_cfg = (uint8_t)((ch_valid(map->pitch_ch) != 0U) || (map->mouse_pitch_scale != 0.0F));
    if (ch_valid(map->pitch_ch) != 0U) {
        pitch_value += (float32_t)rc_deadband(rc->rc.ch[(uint8_t)map->pitch_ch], map->deadband) * map->pitch_scale;
    }
    pitch_value += (float32_t)rc->mouse.y * map->mouse_pitch_scale;
    if (pitch_cfg != 0U) {
        input_axis_set(&out->pitch, pitch_value, map->priority, InputSourceRc, rc->last_update_tick);
        out->valid_mask |= INPUT_GIMBAL_PITCH_VALID;
    }

    if ((sw_valid(map->mode_sw) != 0U) &&
        (mode_from_sw(rc->rc.sw[(uint8_t)map->mode_sw], map, &mode) != 0U)) {
        out->mode = mode;
        out->mode_valid = 1U;
        out->valid_mask |= INPUT_GIMBAL_MODE_VALID;
    }

    if (override_allowed(rc, map) != 0U) {
        if ((map->zero_key != 0U) && (rc_key_down(rc->key.v, map->zero_key) != 0U)) {
            out->mode = GimbalModeZero;
            out->mode_valid = 1U;
            out->valid_mask |= INPUT_GIMBAL_MODE_VALID;
        } else if ((map->gyro_key != 0U) && (rc_key_down(rc->key.v, map->gyro_key) != 0U)) {
            out->mode = GimbalModeGyro;
            out->mode_valid = 1U;
            out->valid_mask |= INPUT_GIMBAL_MODE_VALID;
        } else if ((map->encode_key != 0U) && (rc_key_down(rc->key.v, map->encode_key) != 0U)) {
            out->mode = GimbalModeEncode;
            out->mode_valid = 1U;
            out->valid_mask |= INPUT_GIMBAL_MODE_VALID;
        } else if ((map->raw_key != 0U) && (rc_key_down(rc->key.v, map->raw_key) != 0U)) {
            out->mode = GimbalModeRaw;
            out->mode_valid = 1U;
            out->valid_mask |= INPUT_GIMBAL_MODE_VALID;
        }
    }

    return InputOk;
}

/**
 ******************************************************************************
 * @file    input_chassis_rc.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   RC 到底盘参考的映射实现。
 *
 * @note    本文件只解释 RC 物理输入，不读取任务、裁判系统或机构全局状态。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "input_chassis_rc.h"

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
static uint8_t override_allowed(const RcDbusState *rc, const InputChassisRcMap *map)
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

/* ==========================================================================
 * 输入、状态更新与控制步骤
 * --------------------------------------------------------------------------
 * 这一组把注入的输入转换成组件输出；具体调度和硬件发送仍由 App 层负责。
 * ========================================================================== */

/**
 * @brief 一阶低通滤波。
 *
 * 与旧 first_order_filter_cali() 等价：out = a*out + (1-a)*input。
 *
 * @param last 上一次输出。
 * @param input 当前输入。
 * @param period_s 周期，单位 s。
 * @param num 旧低通参数。
 * @return 新输出。
 */
static float32_t lpf_step(float32_t last, float32_t input, float32_t period_s, float32_t num)
{
    float32_t den;

    if ((num <= 0.0F) || (period_s <= 0.0F)) {
        return input;
    }

    den = num + period_s;
    return ((num / den) * last) + ((period_s / den) * input);
}

/**
 * @brief 判断键盘是否正在给出平移速度。
 *
 * @param rc DBUS 风格物理输入状态。
 * @param map 映射配置。
 * @return 正在平移返回 1，否则返回 0。
 */
static uint8_t move_key_down(const RcDbusState *rc, const InputChassisRcMap *map)
{
    if ((rc == NULL) || (map == NULL)) {
        return 0U;
    }

    return (uint8_t)(((map->front_key != 0U) && (rc_key_down(rc->key.v, map->front_key) != 0U)) ||
                     ((map->back_key != 0U) && (rc_key_down(rc->key.v, map->back_key) != 0U)) ||
                     ((map->left_key != 0U) && (rc_key_down(rc->key.v, map->left_key) != 0U)) ||
                     ((map->right_key != 0U) && (rc_key_down(rc->key.v, map->right_key) != 0U)));
}

/**
 * @brief 按拨杆值选择底盘模式。
 *
 * @param sw 拨杆值。
 * @param map 映射配置。
 * @param mode 模式输出。
 * @return 选择成功返回 1，否则返回 0。
 */
static uint8_t mode_from_sw(uint8_t sw, const InputChassisRcMap *map, ChassisMode *mode)
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

/**
 * @brief 按旧步兵底盘双拨杆逻辑选择模式。
 *
 * @param rc DBUS 风格物理输入状态。
 * @param map 映射配置。
 * @param mode 模式输出。
 * @return 选择成功返回 1，否则返回 0。
 */
static uint8_t mode_from_rc(const RcDbusState *rc, const InputChassisRcMap *map, ChassisMode *mode)
{
    uint8_t mode_sw_value;

    if ((rc == NULL) || (map == NULL) || (mode == NULL)) {
        return 0U;
    }

    if (sw_valid(map->mode_sw) == 0U) {
        return 0U;
    }

    mode_sw_value = rc->rc.sw[(uint8_t)map->mode_sw];
    if ((rc_sw_is_down(mode_sw_value) != 0U) && (sw_valid(map->follow_sw) != 0U)) {
        if (rc_sw_is_down(rc->rc.sw[(uint8_t)map->follow_sw]) != 0U) {
            *mode = map->follow_down_mode;
        } else {
            *mode = map->follow_else_mode;
        }
        return 1U;
    }

    return mode_from_sw(mode_sw_value, map, mode);
}

/**
 * @brief 根据键盘覆盖旧模式。
 *
 * @param rc DBUS 风格物理输入状态。
 * @param map 映射配置。
 * @param mode 模式输入输出。
 */
static void mode_from_key(const RcDbusState *rc, const InputChassisRcMap *map, ChassisMode *mode)
{
    if ((rc == NULL) || (map == NULL) || (mode == NULL) || (override_allowed(rc, map) == 0U)) {
        return;
    }

    if ((map->stop_key != 0U) && (rc_key_down(rc->key.v, map->stop_key) != 0U)) {
        *mode = ChassisModeStop;
    } else if ((map->spin_key != 0U) && (rc_key_down(rc->key.v, map->spin_key) != 0U)) {
        *mode = ChassisModeSpin;
    } else if ((map->follow_key != 0U) && (rc_key_down(rc->key.v, map->follow_key) != 0U)) {
        *mode = ChassisModeFollowGimbalYaw;
    }
}

/**
 * @brief 生成跟随模式下的摆动 yaw 参考。
 *
 * @param ctx 映射上下文。
 * @param rc DBUS 风格物理输入状态。
 * @param map 映射配置。
 * @param out 底盘参考输出。
 */
static void swing_ref(InputChassisRcCtx *ctx,
                      const RcDbusState *rc,
                      const InputChassisRcMap *map,
                      InputChassisRef *out)
{
    float32_t angle;
    float32_t max_angle;

    if ((ctx == NULL) || (rc == NULL) || (map == NULL) || (out == NULL)) {
        return;
    }

    if ((map->swing_key == 0U) || (rc_key_down(rc->key.v, map->swing_key) == 0U)) {
        ctx->swing_active = 0U;
        ctx->swing_phase = 0.0F;
        return;
    }

    if (ctx->swing_active == 0U) {
        ctx->swing_phase = 0.0F;
        ctx->swing_active = 1U;
    }

    max_angle = (move_key_down(rc, map) != 0U) ? map->swing_move_angle : map->swing_stop_angle;
    angle = max_angle * arm_sin_f32(ctx->swing_phase);
    ctx->swing_phase = angle_wrap_2pi(ctx->swing_phase + map->swing_step);
    input_axis_set(&out->yaw_ref, angle, map->priority, InputSourceRc, rc->last_update_tick);
    out->valid_mask |= INPUT_CHASSIS_YAW_VALID;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
void input_chassis_rc_init(InputChassisRcCtx *ctx)
{
    if (ctx == NULL) {
        return;
    }

    (void)memset(ctx, 0, sizeof(*ctx));
}

InputStatus input_chassis_rc_map(InputChassisRcCtx *ctx,
                                 const RcDbusState *rc,
                                 const InputChassisRcMap *map,
                                 InputChassisRef *out)
{
    float32_t vx = 0.0F;
    float32_t vy = 0.0F;
    float32_t wz = 0.0F;
    ChassisMode mode;
    uint8_t vx_cfg;
    uint8_t vy_cfg;
    uint8_t wz_cfg;

    if ((ctx == NULL) || (rc == NULL) || (map == NULL) || (out == NULL)) {
        return InputNull;
    }

    input_chassis_clear(out);
    out->priority = map->priority;
    out->source = InputSourceRc;
    out->tick_ms = rc->last_update_tick;

    if (rc->online == 0U) {
        out->force_stop = 1U;
        (void)memset(ctx, 0, sizeof(*ctx));
        return InputOk;
    }

    if (rc_dbus_state_valid(rc) == 0U) {
        return InputValue;
    }

    out->online = 1U;
    vx_cfg = (uint8_t)((ch_valid(map->vx_ch) != 0U) ||
                       ((map->front_key != 0U) && (map->key_vx_max != 0.0F)) ||
                       ((map->back_key != 0U) && (map->key_vx_min != 0.0F)));
    vy_cfg = (uint8_t)((ch_valid(map->vy_ch) != 0U) ||
                       ((map->left_key != 0U) && (map->key_vy_max != 0.0F)) ||
                       ((map->right_key != 0U) && (map->key_vy_min != 0.0F)));
    wz_cfg = (uint8_t)((ch_valid(map->wz_ch) != 0U) ||
                       ((map->turn_l_key != 0U) && (map->key_wz != 0.0F)) ||
                       ((map->turn_r_key != 0U) && (map->key_wz != 0.0F)));

    if (ch_valid(map->vx_ch) != 0U) {
        vx += (float32_t)rc_deadband(rc->rc.ch[(uint8_t)map->vx_ch], map->deadband) * map->vx_scale;
    }
    if (ch_valid(map->vy_ch) != 0U) {
        vy += (float32_t)rc_deadband(rc->rc.ch[(uint8_t)map->vy_ch], map->deadband) * map->vy_scale;
    }
    if (ch_valid(map->wz_ch) != 0U) {
        wz += (float32_t)rc_deadband(rc->rc.ch[(uint8_t)map->wz_ch], map->deadband) * map->wz_scale;
    }

    if ((map->front_key != 0U) && (rc_key_down(rc->key.v, map->front_key) != 0U)) {
        vx = map->key_vx_max;
    } else if ((map->back_key != 0U) && (rc_key_down(rc->key.v, map->back_key) != 0U)) {
        vx = map->key_vx_min;
    }
    if ((map->left_key != 0U) && (rc_key_down(rc->key.v, map->left_key) != 0U)) {
        vy = map->key_vy_max;
    } else if ((map->right_key != 0U) && (rc_key_down(rc->key.v, map->right_key) != 0U)) {
        vy = map->key_vy_min;
    }
    if ((map->turn_l_key != 0U) && (rc_key_down(rc->key.v, map->turn_l_key) != 0U)) {
        wz += map->key_wz;
    }
    if ((map->turn_r_key != 0U) && (rc_key_down(rc->key.v, map->turn_r_key) != 0U)) {
        wz -= map->key_wz;
    }

    ctx->vx_out = lpf_step(ctx->vx_out, vx, map->period_s, map->vx_lpf_num);
    ctx->vy_out = lpf_step(ctx->vy_out, vy, map->period_s, map->vy_lpf_num);
    if (fabsf(vx) < ((float32_t)map->deadband * fabsf(map->vx_scale))) {
        ctx->vx_out = 0.0F;
    }
    if (fabsf(vy) < ((float32_t)map->deadband * fabsf(map->vy_scale))) {
        ctx->vy_out = 0.0F;
    }
    vx = ctx->vx_out;
    vy = ctx->vy_out;

    if (vx_cfg != 0U) {
        input_axis_set(&out->vx, vx, map->priority, InputSourceRc, rc->last_update_tick);
        out->valid_mask |= INPUT_CHASSIS_VX_VALID;
    }
    if (vy_cfg != 0U) {
        input_axis_set(&out->vy, vy, map->priority, InputSourceRc, rc->last_update_tick);
        out->valid_mask |= INPUT_CHASSIS_VY_VALID;
    }
    if (wz_cfg != 0U) {
        input_axis_set(&out->wz, wz, map->priority, InputSourceRc, rc->last_update_tick);
        out->valid_mask |= INPUT_CHASSIS_WZ_VALID;
    }

    if (mode_from_rc(rc, map, &mode) != 0U) {
        mode_from_key(rc, map, &mode);
        out->mode = mode;
        out->mode_valid = 1U;
        out->valid_mask |= INPUT_CHASSIS_MODE_VALID;
        if (mode == ChassisModeStop) {
            out->force_stop = 1U;
        } else if (mode == ChassisModeSpin) {
            input_axis_set(&out->wz, map->spin_wz, map->priority, InputSourceRc, rc->last_update_tick);
            out->valid_mask |= INPUT_CHASSIS_WZ_VALID;
        } else if (mode == ChassisModeFollowGimbalYaw) {
            input_axis_clear(&out->wz);
            out->valid_mask &= (uint32_t)(~INPUT_CHASSIS_WZ_VALID);
            swing_ref(ctx, rc, map, out);
        } else {
            ctx->swing_active = 0U;
        }
    }

    if ((map->jump_key != 0U) && (rc_key_down(rc->key.v, map->jump_key) != 0U)) {
        out->jump = 1U;
    }

    return InputOk;
}

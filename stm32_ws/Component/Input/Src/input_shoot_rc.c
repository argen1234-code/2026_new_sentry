/**
 ******************************************************************************
 * @file    input_shoot_rc.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   RC 到发射参考的映射实现。
 *
 * @note    本文件只解释 RC 物理输入，不读取裁判系统、云台或任务状态。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "input_shoot_rc.h"

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
 * @brief 饱和递增 16 位计数器。
 *
 * @param value 当前计数。
 * @param max 最大计数。
 * @return 递增后的计数。
 */
static uint16_t count_up(uint16_t value, uint16_t max)
{
    if (value < max) {
        value++;
    }

    return value;
}

/**
 * @brief 判断弹舱拨杆保护条件是否满足。
 *
 * @param rc DBUS 风格物理输入状态。
 * @param map 映射配置。
 * @return 满足保护条件返回 1，否则返回 0。
 */
static uint8_t mag_guard_ok(const RcDbusState *rc, const InputShootRcMap *map)
{
    if ((rc == NULL) || (map == NULL)) {
        return 0U;
    }

    if (sw_valid(map->mag_guard_sw) == 0U) {
        return 1U;
    }

    return (rc->rc.sw[(uint8_t)map->mag_guard_sw] == map->mag_guard_value) ? 1U : 0U;
}

/**
 * @brief 根据 RC 状态生成弹舱参考。
 *
 * @param rc DBUS 风格物理输入状态。
 * @param map 映射配置。
 * @param out 弹舱参考输出，可为空。
 */
static void map_magazine(const RcDbusState *rc, const InputShootRcMap *map, InputMagRef *out)
{
    uint8_t mode_value;
    uint8_t guard_ok;

    if (out == NULL) {
        return;
    }

    input_mag_clear(out);
    if ((rc == NULL) || (map == NULL) || (rc->online == 0U)) {
        return;
    }

    out->online = 1U;
    out->priority = map->priority;
    out->source = InputSourceRc;
    out->tick_ms = rc->last_update_tick;

    guard_ok = mag_guard_ok(rc, map);
    if ((map->mag_open_key != 0U) && (rc_key_down(rc->key.v, map->mag_open_key) != 0U)) {
        out->action = InputMagOpen;
        out->action_valid = 1U;
    } else if ((map->mag_close_key != 0U) && (rc_key_down(rc->key.v, map->mag_close_key) != 0U)) {
        out->action = InputMagClose;
        out->action_valid = 1U;
    } else if ((guard_ok != 0U) && (sw_valid(map->mag_mode_sw) != 0U)) {
        mode_value = rc->rc.sw[(uint8_t)map->mag_mode_sw];
        if (mode_value == map->mag_open_sw) {
            out->action = InputMagOpen;
            out->action_valid = 1U;
        } else if (mode_value == map->mag_close_sw) {
            out->action = InputMagClose;
            out->action_valid = 1U;
        }
    }

    if (out->action_valid != 0U) {
        out->valid_mask = INPUT_MAG_ACTION_VALID;
    }
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
void input_shoot_rc_init(InputShootRcCtx *ctx)
{
    if (ctx == NULL) {
        return;
    }

    memset(ctx, 0, sizeof(*ctx));
}

InputStatus input_shoot_rc_map(InputShootRcCtx *ctx,
                               const RcDbusState *rc,
                               const InputShootRcMap *map,
                               InputShootRef *out,
                               InputMagRef *mag)
{
    uint8_t once_now = 0U;
    uint8_t hold_now = 0U;
    uint8_t sw_value = RC_SW_MID;
    uint8_t fire_sw_now = 0U;
    uint8_t sw_cfg;

    if ((ctx == NULL) || (rc == NULL) || (map == NULL) || (out == NULL)) {
        return InputNull;
    }

    input_shoot_clear(out);
    if (mag != NULL) {
        input_mag_clear(mag);
    }
    out->priority = map->priority;
    out->source = InputSourceRc;
    out->tick_ms = rc->last_update_tick;

    if (rc->online == 0U) {
        out->force_stop = 1U;
        out->valid_mask = INPUT_SHOOT_STOP_VALID;
        ctx->fric_enable = 0U;
        ctx->last_once = 0U;
        ctx->press_l_time = 0U;
        ctx->press_r_time = 0U;
        ctx->switch_time = 0U;
        return InputOk;
    }

    if (rc_dbus_state_valid(rc) == 0U) {
        return InputValue;
    }

    map_magazine(rc, map, mag);
    out->online = 1U;
    sw_cfg = sw_valid(map->mode_sw);
    if (sw_cfg != 0U) {
        sw_value = rc->rc.sw[(uint8_t)map->mode_sw];
        if (ctx->initialized == 0U) {
            ctx->last_switch = sw_value;
            ctx->initialized = 1U;
        }
    }

    if ((sw_cfg != 0U) &&
        (sw_value == map->toggle_sw) &&
        (ctx->last_switch != map->toggle_sw)) {
        ctx->fric_enable = (ctx->fric_enable == 0U) ? 1U : 0U;
    }
    if ((map->fric_on_key != 0U) && (rc_key_down(rc->key.v, map->fric_on_key) != 0U)) {
        ctx->fric_enable = 1U;
    }
    if ((map->fric_off_key != 0U) && (rc_key_down(rc->key.v, map->fric_off_key) != 0U)) {
        ctx->fric_enable = 0U;
    }
    out->fric_enable = ctx->fric_enable;
    out->valid_mask |= INPUT_SHOOT_FRIC_VALID;

    if ((map->fire_once_key != 0U) && (rc_key_down(rc->key.v, map->fire_once_key) != 0U)) {
        once_now = 1U;
    }
    if ((map->once_mouse_l != 0U) && (rc->mouse.press_l != 0U)) {
        once_now = 1U;
    }

    if (rc->mouse.press_l != 0U) {
        ctx->press_l_time = count_up(ctx->press_l_time, map->press_long_time);
    } else {
        ctx->press_l_time = 0U;
    }
    if (rc->mouse.press_r != 0U) {
        ctx->press_r_time = count_up(ctx->press_r_time, map->press_long_time);
    } else {
        ctx->press_r_time = 0U;
    }

    if ((sw_cfg != 0U) && (sw_value == map->fire_sw) && (ctx->fric_enable != 0U)) {
        fire_sw_now = 1U;
        ctx->switch_time = count_up(ctx->switch_time, map->switch_long_time);
    } else {
        ctx->switch_time = 0U;
    }

    if ((fire_sw_now != 0U) && (ctx->last_switch != map->fire_sw)) {
        once_now = 1U;
    }

    if ((map->fire_hold_key != 0U) && (rc_key_down(rc->key.v, map->fire_hold_key) != 0U)) {
        hold_now = 1U;
    }
    if ((map->hold_mouse_l != 0U) &&
        (map->press_long_time != 0U) &&
        (ctx->press_l_time >= map->press_long_time)) {
        hold_now = 1U;
    }
    if ((map->hold_mouse_r != 0U) &&
        (map->press_long_time != 0U) &&
        (ctx->press_r_time >= map->press_long_time)) {
        hold_now = 1U;
    }
    if ((map->switch_long_time != 0U) && (ctx->switch_time >= map->switch_long_time)) {
        hold_now = 1U;
    }

    out->fire_hold = (ctx->fric_enable != 0U) ? hold_now : 0U;
    out->fire_once = ((ctx->fric_enable != 0U) && (once_now != 0U) && (ctx->last_once == 0U)) ? 1U : 0U;
    out->valid_mask |= (INPUT_SHOOT_ONCE_VALID | INPUT_SHOOT_HOLD_VALID);

    ctx->last_switch = sw_value;
    ctx->last_key = rc->key.v;
    ctx->last_once = once_now;

    return InputOk;
}

/**
 ******************************************************************************
 * @file    input.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   统一输入组件公共接口实现。
 *
 * @note    本文件只做输入源聚合、优先级仲裁和机构命令输出。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "input.h"

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
 * @brief 记录首个非 OK 状态。
 *
 * @param dst 当前状态。
 * @param src 新状态。
 */
static void keep_status(InputStatus *dst, InputStatus src)
{
    if ((dst != NULL) && (*dst == InputOk) && (src != InputOk)) {
        *dst = src;
    }
}

/**
 * @brief 按输入源生成位图。
 *
 * @param source 输入源。
 * @return 输入源位图。
 */
static uint32_t source_bit(InputSource source)
{
    return (source == InputSourceNone) ? 0UL : (1UL << (uint32_t)source);
}

/**
 * @brief 合并云台参考并更新来源位图。
 *
 * @param ref 聚合参考。
 * @param src 候选云台参考。
 */
static void merge_gimbal(InputRef *ref, const InputGimbalRef *src)
{
    if ((ref == NULL) || (src == NULL)) {
        return;
    }

    input_merge_gimbal(&ref->gimbal, src);
    if ((src->valid_mask != 0U) || (src->force_stop != 0U) || (src->mode_valid != 0U)) {
        ref->source_mask |= source_bit(src->source);
    }
}

/**
 * @brief 合并底盘参考并更新来源位图。
 *
 * @param ref 聚合参考。
 * @param src 候选底盘参考。
 */
static void merge_chassis(InputRef *ref, const InputChassisRef *src)
{
    if ((ref == NULL) || (src == NULL)) {
        return;
    }

    input_merge_chassis(&ref->chassis, src);
    if ((src->valid_mask != 0U) || (src->force_stop != 0U) || (src->mode_valid != 0U) || (src->jump != 0U)) {
        ref->source_mask |= source_bit(src->source);
    }
}

/**
 * @brief 合并发射参考并更新来源位图。
 *
 * @param ref 聚合参考。
 * @param src 候选发射参考。
 */
static void merge_shoot(InputRef *ref, const InputShootRef *src)
{
    if ((ref == NULL) || (src == NULL)) {
        return;
    }

    input_merge_shoot(&ref->shoot, src);
    if (src->valid_mask != 0U) {
        ref->source_mask |= source_bit(src->source);
    }
}

/**
 * @brief 合并弹舱参考并更新来源位图。
 *
 * @param ref 聚合参考。
 * @param src 候选弹舱参考。
 */
static void merge_mag(InputRef *ref, const InputMagRef *src)
{
    if ((ref == NULL) || (src == NULL)) {
        return;
    }

    input_merge_mag(&ref->mag, src);
    if (src->valid_mask != 0U) {
        ref->source_mask |= source_bit(src->source);
    }
}

/**
 * @brief 合并校准参考并更新来源位图。
 *
 * @param ref 聚合参考。
 * @param src 候选校准参考。
 */
static void merge_cali(InputRef *ref, const InputCaliRef *src)
{
    if ((ref == NULL) || (src == NULL)) {
        return;
    }

    input_merge_cali(&ref->cali, src);
    if (src->valid_mask != 0U) {
        ref->source_mask |= source_bit(src->source);
    }
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
InputStatus input_init(Input *input, const InputCfg *cfg)
{
    if ((input == NULL) || (cfg == NULL)) {
        return InputNull;
    }

    memset(input, 0, sizeof(*input));
    input->cfg = *cfg;
    input_gimbal_rc_init(&input->gimbal_rc);
    input_chassis_rc_init(&input->chassis_rc);
    input_shoot_rc_init(&input->shoot_rc);
    input_cali_rc_init(&input->cali_rc);
    input_ref_clear(&input->output.ref);
    input->initialized = 1U;

    return InputOk;
}

InputStatus input_update(Input *input, const InputFrame *frame)
{
    InputGimbalRef gimbal_ref;
    InputChassisRef chassis_ref;
    InputShootRef shoot_ref;
    InputMagRef mag_ref;
    InputCaliRef cali_ref;
    InputStatus status = InputOk;

    if ((input == NULL) || (frame == NULL)) {
        return InputNull;
    }

    if (input->initialized == 0U) {
        return InputArg;
    }

    input_ref_clear(&input->output.ref);
    input->output.ref.tick_ms = frame->tick_ms;

    if (frame->rc != NULL) {
        keep_status(&status, input_gimbal_rc_map(&input->gimbal_rc,
                                                 frame->rc,
                                                 &input->cfg.gimbal_rc,
                                                 frame->gimbal_yaw,
                                                 frame->gimbal_yaw_valid,
                                                 &gimbal_ref));
        merge_gimbal(&input->output.ref, &gimbal_ref);

        keep_status(&status, input_chassis_rc_map(&input->chassis_rc,
                                                  frame->rc,
                                                  &input->cfg.chassis_rc,
                                                  &chassis_ref));
        merge_chassis(&input->output.ref, &chassis_ref);

        keep_status(&status, input_shoot_rc_map(&input->shoot_rc, frame->rc, &input->cfg.shoot_rc, &shoot_ref, &mag_ref));
        merge_shoot(&input->output.ref, &shoot_ref);
        merge_mag(&input->output.ref, &mag_ref);

        keep_status(&status, input_cali_rc_map(&input->cali_rc,
                                               frame->rc,
                                               &input->cfg.cali_rc,
                                               &cali_ref));
        merge_cali(&input->output.ref, &cali_ref);
    }

    if (frame->vision != NULL) {
        keep_status(&status, input_gimbal_vision_map(frame->vision, &gimbal_ref));
        merge_gimbal(&input->output.ref, &gimbal_ref);
    }

    if (frame->slam != NULL) {
        keep_status(&status, input_chassis_slam_map(frame->slam, &chassis_ref));
        merge_chassis(&input->output.ref, &chassis_ref);
    }

    if (frame->gimbal_plan != NULL) {
        keep_status(&status, input_gimbal_planner_map(frame->gimbal_plan, &gimbal_ref));
        merge_gimbal(&input->output.ref, &gimbal_ref);
    }

    if (frame->chassis_plan != NULL) {
        keep_status(&status, input_chassis_planner_map(frame->chassis_plan, &chassis_ref));
        merge_chassis(&input->output.ref, &chassis_ref);
    }

    if (frame->shoot_plan != NULL) {
        keep_status(&status, input_shoot_planner_map(frame->shoot_plan, &shoot_ref));
        merge_shoot(&input->output.ref, &shoot_ref);
    }

    keep_status(&status, input_make_gimbal(&input->output.ref.gimbal, &input->output.gimbal));
    keep_status(&status, input_make_chassis(&input->output.ref.chassis, &input->output.chassis));
    keep_status(&status, input_make_shoot(&input->output.ref.shoot, &input->output.shoot));
    keep_status(&status, input_make_mag(&input->output.ref.mag, &input->output.mag));
    keep_status(&status, input_make_cali(&input->output.ref.cali, &input->output.cali));

    return status;
}

InputStatus input_get(const Input *input, InputOutput *out)
{
    if ((input == NULL) || (out == NULL)) {
        return InputNull;
    }

    if (input->initialized == 0U) {
        return InputArg;
    }

    *out = input->output;
    return InputOk;
}

InputStatus input_step(Input *input, const InputFrame *frame, InputOutput *out)
{
    InputStatus status;

    if ((input == NULL) || (frame == NULL) || (out == NULL)) {
        return InputNull;
    }

    status = input_update(input, frame);
    if (status != InputOk) {
        return status;
    }

    return input_get(input, out);
}

/**
 ******************************************************************************
 * @file    gimbal_linkage_yaw_pitch.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   linkage-yaw-pitch 云台构型实现。
 *
 * @note    本文件只处理 linkage 构型的输入联动变换。
 * @note    yaw/pitch 反馈映射、目标更新和电流计算复用直驱核心。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "gimbal_linkage_yaw_pitch.h"

#include <string.h>

/** @brief yaw 输入到 pitch 输入的默认联动系数。 */
#ifndef GIMBAL_LINKAGE_PITCH_GAIN
/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */
#define GIMBAL_LINKAGE_PITCH_GAIN 0.0F
#endif

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具只处理参数、限幅、wrap、矩阵工作区等实现细节，不读取任务或设备全局。
 * ========================================================================== */

/**
 * @brief 生成联动后的直驱核心输入。
 *
 * @param ctx linkage 私有上下文。
 * @param src 原始输入。
 * @param dst 输出给直驱核心的输入。
 */
static void linkage_input(const GimbalLinkageCtx *ctx, const GimbalInput *src, GimbalInput *dst)
{
    if ((ctx == NULL) || (src == NULL) || (dst == NULL)) {
        return;
    }

    *dst = *src;
    dst->cmd.pitch += src->cmd.yaw * ctx->pitch_gain;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
GimbalStatus gimbal_linkage_init(Gimbal *gimbal, void *impl)
{
    GimbalLinkageCtx *ctx = (GimbalLinkageCtx *)impl;

    if ((gimbal == NULL) || (ctx == NULL)) {
        return GimbalNull;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->pitch_gain = GIMBAL_LINKAGE_PITCH_GAIN;
    return gimbal_direct_init(gimbal, &ctx->core);
}

GimbalStatus gimbal_linkage_step(Gimbal *gimbal,
                                 void *impl,
                                 const GimbalInput *input,
                                 GimbalOutput *output)
{
    GimbalStatus status;

    if ((gimbal == NULL) || (impl == NULL) || (input == NULL) || (output == NULL)) {
        return GimbalNull;
    }

    status = gimbal_linkage_update(gimbal, impl, input);
    if (status != GimbalOk) {
        return status;
    }

    status = gimbal_linkage_control(gimbal, impl, input);
    if (status != GimbalOk) {
        return status;
    }

    return gimbal_linkage_output(gimbal, impl, output);
}

GimbalStatus gimbal_linkage_update(Gimbal *gimbal, void *impl, const GimbalInput *input)
{
    GimbalLinkageCtx *ctx = (GimbalLinkageCtx *)impl;
    GimbalInput work;

    if ((gimbal == NULL) || (ctx == NULL) || (input == NULL)) {
        return GimbalNull;
    }

    linkage_input(ctx, input, &work);
    return gimbal_direct_update(gimbal, &ctx->core, &work);
}

GimbalStatus gimbal_linkage_control(Gimbal *gimbal, void *impl, const GimbalInput *input)
{
    GimbalLinkageCtx *ctx = (GimbalLinkageCtx *)impl;
    GimbalInput work;

    if ((gimbal == NULL) || (ctx == NULL) || (input == NULL)) {
        return GimbalNull;
    }

    linkage_input(ctx, input, &work);
    return gimbal_direct_control(gimbal, &ctx->core, &work);
}

GimbalStatus gimbal_linkage_output(const Gimbal *gimbal, void *impl, GimbalOutput *output)
{
    GimbalLinkageCtx *ctx = (GimbalLinkageCtx *)impl;

    if ((gimbal == NULL) || (ctx == NULL) || (output == NULL)) {
        return GimbalNull;
    }

    return gimbal_direct_output(gimbal, &ctx->core, output);
}

GimbalStatus gimbal_linkage_sync(Gimbal *gimbal, void *impl)
{
    GimbalLinkageCtx *ctx = (GimbalLinkageCtx *)impl;

    if ((gimbal == NULL) || (ctx == NULL)) {
        return GimbalNull;
    }

    return gimbal_direct_sync(gimbal, &ctx->core);
}

GimbalStatus gimbal_linkage_set_cali(Gimbal *gimbal, void *impl, const GimbalCali *cali)
{
    GimbalLinkageCtx *ctx = (GimbalLinkageCtx *)impl;

    if ((gimbal == NULL) || (ctx == NULL) || (cali == NULL)) {
        return GimbalNull;
    }

    return gimbal_direct_set_cali(gimbal, &ctx->core, cali);
}

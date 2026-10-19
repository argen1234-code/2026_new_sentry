/**
 ******************************************************************************
 * @file    shoot_none.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   无发射机构构型实现。
 *
 * @note    本文件只生成零输出，不读取任何外部运行时状态。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "shoot_none.h"

#include <string.h>

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
ShootStatus shoot_none_init(Shoot *shoot, void *impl)
{
    ShootNoneCtx *ctx = (ShootNoneCtx *)impl;

    if ((shoot == NULL) || (ctx == NULL)) {
        return ShootNull;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->ready = 1U;
    return ShootOk;
}

ShootStatus shoot_none_step(Shoot *shoot, void *impl, const ShootInput *input, ShootOutput *output)
{
    if ((shoot == NULL) || (impl == NULL) || (input == NULL) || (output == NULL)) {
        return ShootNull;
    }

    (void)input;
    return shoot_none_output(shoot, impl, output);
}

ShootStatus shoot_none_update(Shoot *shoot, void *impl, const ShootInput *input)
{
    if ((shoot == NULL) || (impl == NULL) || (input == NULL)) {
        return ShootNull;
    }

    return ShootOk;
}

ShootStatus shoot_none_control(Shoot *shoot, void *impl, const ShootInput *input)
{
    if ((shoot == NULL) || (impl == NULL) || (input == NULL)) {
        return ShootNull;
    }

    return ShootOk;
}

ShootStatus shoot_none_output(const Shoot *shoot, void *impl, ShootOutput *output)
{
    if ((shoot == NULL) || (impl == NULL) || (output == NULL)) {
        return ShootNull;
    }

    memset(output, 0, sizeof(*output));
    output->mode = ShootModeStop;
    output->send_enable = 1U;
    return ShootOk;
}

ShootStatus shoot_none_sync(Shoot *shoot, void *impl)
{
    if ((shoot == NULL) || (impl == NULL)) {
        return ShootNull;
    }

    return ShootOk;
}

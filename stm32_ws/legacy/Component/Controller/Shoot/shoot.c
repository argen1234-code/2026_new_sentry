/**
 ******************************************************************************
 * @file    shoot.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   发射机构公共接口实现。
 *
 * @note    本文件只负责公共上下文绑定、参数检查和编译期构型宏转发。
 * @note    具体控制律、反馈映射和输出生成必须放在 Src 下的构型实现中。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "shoot.h"

#include <string.h>

#if SHOOT_TYPE == SHOOT_TYPE_NONE
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
ShootStatus shoot_none_init(Shoot *shoot, void *impl);
ShootStatus shoot_none_step(Shoot *shoot, void *impl, const ShootInput *input, ShootOutput *output);
ShootStatus shoot_none_update(Shoot *shoot, void *impl, const ShootInput *input);
ShootStatus shoot_none_control(Shoot *shoot, void *impl, const ShootInput *input);
ShootStatus shoot_none_output(const Shoot *shoot, void *impl, ShootOutput *output);
ShootStatus shoot_none_sync(Shoot *shoot, void *impl);
#define SHOOT_IMPL_INIT shoot_none_init
#define SHOOT_IMPL_STEP shoot_none_step
#define SHOOT_IMPL_UPDATE shoot_none_update
#define SHOOT_IMPL_CONTROL shoot_none_control
#define SHOOT_IMPL_OUTPUT shoot_none_output
#define SHOOT_IMPL_SYNC shoot_none_sync
#elif SHOOT_TYPE == SHOOT_TYPE_MONO
ShootStatus shoot_mono_init(Shoot *shoot, void *impl);
ShootStatus shoot_mono_step(Shoot *shoot, void *impl, const ShootInput *input, ShootOutput *output);
ShootStatus shoot_mono_update(Shoot *shoot, void *impl, const ShootInput *input);
ShootStatus shoot_mono_control(Shoot *shoot, void *impl, const ShootInput *input);
ShootStatus shoot_mono_output(const Shoot *shoot, void *impl, ShootOutput *output);
ShootStatus shoot_mono_sync(Shoot *shoot, void *impl);
#define SHOOT_IMPL_INIT shoot_mono_init
#define SHOOT_IMPL_STEP shoot_mono_step
#define SHOOT_IMPL_UPDATE shoot_mono_update
#define SHOOT_IMPL_CONTROL shoot_mono_control
#define SHOOT_IMPL_OUTPUT shoot_mono_output
#define SHOOT_IMPL_SYNC shoot_mono_sync
#elif SHOOT_TYPE == SHOOT_TYPE_DOUBLE
ShootStatus shoot_double_init(Shoot *shoot, void *impl);
ShootStatus shoot_double_step(Shoot *shoot, void *impl, const ShootInput *input, ShootOutput *output);
ShootStatus shoot_double_update(Shoot *shoot, void *impl, const ShootInput *input);
ShootStatus shoot_double_control(Shoot *shoot, void *impl, const ShootInput *input);
ShootStatus shoot_double_output(const Shoot *shoot, void *impl, ShootOutput *output);
ShootStatus shoot_double_sync(Shoot *shoot, void *impl);
#define SHOOT_IMPL_INIT shoot_double_init
#define SHOOT_IMPL_STEP shoot_double_step
#define SHOOT_IMPL_UPDATE shoot_double_update
#define SHOOT_IMPL_CONTROL shoot_double_control
#define SHOOT_IMPL_OUTPUT shoot_double_output
#define SHOOT_IMPL_SYNC shoot_double_sync
#else
#error "Unsupported SHOOT_TYPE"
#endif

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具只处理参数、限幅、wrap、矩阵工作区等实现细节，不读取任务或设备全局。
 * ========================================================================== */

/* ==========================================================================
 * 输入、状态更新与控制步骤
 * --------------------------------------------------------------------------
 * 这一组把注入的输入转换成组件输出；具体调度和硬件发送仍由 App 层负责。
 * ========================================================================== */

/**
 * @brief 判断发射公共上下文是否已经初始化。
 *
 * @param shoot 发射公共上下文。
 * @return 已初始化返回 1，否则返回 0。
 */
static uint8_t shoot_ready(const Shoot *shoot)
{
    if ((shoot == NULL) || (shoot->initialized == 0U)) {
        return 0U;
    }

    return 1U;
}

ShootStatus shoot_init(Shoot *shoot, const ShootCfg *cfg, void *impl)
{
    ShootStatus status;

    if ((shoot == NULL) || (cfg == NULL)) {
        return ShootNull;
    }

    if ((uint32_t)cfg->type != (uint32_t)SHOOT_TYPE) {
        return ShootArg;
    }

    memset(shoot, 0, sizeof(*shoot));
    shoot->cfg = *cfg;
    shoot->impl = impl;

    status = SHOOT_IMPL_INIT(shoot, impl);
    if (status != ShootOk) {
        memset(shoot, 0, sizeof(*shoot));
        return status;
    }

    shoot->initialized = 1U;
    return ShootOk;
}

ShootStatus shoot_step(Shoot *shoot, const ShootInput *input, ShootOutput *output)
{
    if ((shoot == NULL) || (input == NULL) || (output == NULL)) {
        return ShootNull;
    }

    memset(output, 0, sizeof(*output));
    if (shoot_ready(shoot) == 0U) {
        return ShootState;
    }

    return SHOOT_IMPL_STEP(shoot, shoot->impl, input, output);
}

ShootStatus shoot_update(Shoot *shoot, const ShootInput *input)
{
    if ((shoot == NULL) || (input == NULL)) {
        return ShootNull;
    }

    if (shoot_ready(shoot) == 0U) {
        return ShootState;
    }

    return SHOOT_IMPL_UPDATE(shoot, shoot->impl, input);
}

ShootStatus shoot_control(Shoot *shoot, const ShootInput *input)
{
    if ((shoot == NULL) || (input == NULL)) {
        return ShootNull;
    }

    if (shoot_ready(shoot) == 0U) {
        return ShootState;
    }

    return SHOOT_IMPL_CONTROL(shoot, shoot->impl, input);
}

ShootStatus shoot_output(const Shoot *shoot, ShootOutput *output)
{
    if ((shoot == NULL) || (output == NULL)) {
        return ShootNull;
    }

    memset(output, 0, sizeof(*output));
    if (shoot_ready(shoot) == 0U) {
        return ShootState;
    }

    return SHOOT_IMPL_OUTPUT(shoot, shoot->impl, output);
}

ShootStatus shoot_sync(Shoot *shoot)
{
    if (shoot == NULL) {
        return ShootNull;
    }

    if (shoot_ready(shoot) == 0U) {
        return ShootState;
    }

    return SHOOT_IMPL_SYNC(shoot, shoot->impl);
}

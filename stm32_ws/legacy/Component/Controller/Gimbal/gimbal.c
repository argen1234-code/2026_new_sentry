/**
 ******************************************************************************
 * @file    gimbal.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   云台机构公共接口实现。
 *
 * @note    本文件只负责公共上下文绑定、参数检查和编译期构型宏转发。
 * @note    具体控制律、反馈映射和执行器封包必须放在 Src 下的构型实现中。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "gimbal.h"

#include <string.h>

#if GIMBAL_TYPE == GIMBAL_TYPE_DIRECT_YAW_PITCH
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
GimbalStatus gimbal_direct_init(Gimbal *gimbal, void *impl);
GimbalStatus gimbal_direct_step(Gimbal *gimbal, void *impl, const GimbalInput *input, GimbalOutput *output);
GimbalStatus gimbal_direct_update(Gimbal *gimbal, void *impl, const GimbalInput *input);
GimbalStatus gimbal_direct_control(Gimbal *gimbal, void *impl, const GimbalInput *input);
GimbalStatus gimbal_direct_output(const Gimbal *gimbal, void *impl, GimbalOutput *output);
GimbalStatus gimbal_direct_sync(Gimbal *gimbal, void *impl);
GimbalStatus gimbal_direct_set_cali(Gimbal *gimbal, void *impl, const GimbalCali *cali);
#define GIMBAL_IMPL_INIT gimbal_direct_init
#define GIMBAL_IMPL_STEP gimbal_direct_step
#define GIMBAL_IMPL_UPDATE gimbal_direct_update
#define GIMBAL_IMPL_CONTROL gimbal_direct_control
#define GIMBAL_IMPL_OUTPUT gimbal_direct_output
#define GIMBAL_IMPL_SYNC gimbal_direct_sync
#define GIMBAL_IMPL_SET_CALI gimbal_direct_set_cali
#elif GIMBAL_TYPE == GIMBAL_TYPE_DOUBLE_YAW_PITCH
GimbalStatus gimbal_double_init(Gimbal *gimbal, void *impl);
GimbalStatus gimbal_double_step(Gimbal *gimbal, void *impl, const GimbalInput *input, GimbalOutput *output);
GimbalStatus gimbal_double_update(Gimbal *gimbal, void *impl, const GimbalInput *input);
GimbalStatus gimbal_double_control(Gimbal *gimbal, void *impl, const GimbalInput *input);
GimbalStatus gimbal_double_output(const Gimbal *gimbal, void *impl, GimbalOutput *output);
GimbalStatus gimbal_double_sync(Gimbal *gimbal, void *impl);
GimbalStatus gimbal_double_set_cali(Gimbal *gimbal, void *impl, const GimbalCali *cali);
#define GIMBAL_IMPL_INIT gimbal_double_init
#define GIMBAL_IMPL_STEP gimbal_double_step
#define GIMBAL_IMPL_UPDATE gimbal_double_update
#define GIMBAL_IMPL_CONTROL gimbal_double_control
#define GIMBAL_IMPL_OUTPUT gimbal_double_output
#define GIMBAL_IMPL_SYNC gimbal_double_sync
#define GIMBAL_IMPL_SET_CALI gimbal_double_set_cali
#elif GIMBAL_TYPE == GIMBAL_TYPE_LINKAGE_YAW_PITCH
GimbalStatus gimbal_linkage_init(Gimbal *gimbal, void *impl);
GimbalStatus gimbal_linkage_step(Gimbal *gimbal, void *impl, const GimbalInput *input, GimbalOutput *output);
GimbalStatus gimbal_linkage_update(Gimbal *gimbal, void *impl, const GimbalInput *input);
GimbalStatus gimbal_linkage_control(Gimbal *gimbal, void *impl, const GimbalInput *input);
GimbalStatus gimbal_linkage_output(const Gimbal *gimbal, void *impl, GimbalOutput *output);
GimbalStatus gimbal_linkage_sync(Gimbal *gimbal, void *impl);
GimbalStatus gimbal_linkage_set_cali(Gimbal *gimbal, void *impl, const GimbalCali *cali);
#define GIMBAL_IMPL_INIT gimbal_linkage_init
#define GIMBAL_IMPL_STEP gimbal_linkage_step
#define GIMBAL_IMPL_UPDATE gimbal_linkage_update
#define GIMBAL_IMPL_CONTROL gimbal_linkage_control
#define GIMBAL_IMPL_OUTPUT gimbal_linkage_output
#define GIMBAL_IMPL_SYNC gimbal_linkage_sync
#define GIMBAL_IMPL_SET_CALI gimbal_linkage_set_cali
#else
#error "Unsupported GIMBAL_TYPE"
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
 * @brief 判断云台公共上下文是否已经初始化。
 *
 * @param gimbal 云台公共上下文。
 * @return 已初始化返回 1，否则返回 0。
 */
static uint8_t gimbal_ready(const Gimbal *gimbal)
{
    if ((gimbal == NULL) || (gimbal->initialized == 0U)) {
        return 0U;
    }

    return 1U;
}

GimbalStatus gimbal_init(Gimbal *gimbal, const GimbalCfg *cfg, void *impl)
{
    GimbalStatus status;

    if ((gimbal == NULL) || (cfg == NULL)) {
        return GimbalNull;
    }

    if ((uint32_t)cfg->type != (uint32_t)GIMBAL_TYPE) {
        return GimbalArg;
    }

    memset(gimbal, 0, sizeof(*gimbal));
    gimbal->cfg = *cfg;
    gimbal->impl = impl;

    status = GIMBAL_IMPL_INIT(gimbal, impl);
    if (status != GimbalOk) {
        memset(gimbal, 0, sizeof(*gimbal));
        return status;
    }

    gimbal->initialized = 1U;
    return GimbalOk;
}

GimbalStatus gimbal_step(Gimbal *gimbal, const GimbalInput *input, GimbalOutput *output)
{
    if ((gimbal == NULL) || (input == NULL) || (output == NULL)) {
        return GimbalNull;
    }

    memset(output, 0, sizeof(*output));
    if (gimbal_ready(gimbal) == 0U) {
        return GimbalState;
    }

    return GIMBAL_IMPL_STEP(gimbal, gimbal->impl, input, output);
}

GimbalStatus gimbal_update(Gimbal *gimbal, const GimbalInput *input)
{
    if ((gimbal == NULL) || (input == NULL)) {
        return GimbalNull;
    }

    if (gimbal_ready(gimbal) == 0U) {
        return GimbalState;
    }

    return GIMBAL_IMPL_UPDATE(gimbal, gimbal->impl, input);
}

GimbalStatus gimbal_control(Gimbal *gimbal, const GimbalInput *input)
{
    if ((gimbal == NULL) || (input == NULL)) {
        return GimbalNull;
    }

    if (gimbal_ready(gimbal) == 0U) {
        return GimbalState;
    }

    return GIMBAL_IMPL_CONTROL(gimbal, gimbal->impl, input);
}

GimbalStatus gimbal_output(const Gimbal *gimbal, GimbalOutput *output)
{
    if ((gimbal == NULL) || (output == NULL)) {
        return GimbalNull;
    }

    memset(output, 0, sizeof(*output));
    if (gimbal_ready(gimbal) == 0U) {
        return GimbalState;
    }

    return GIMBAL_IMPL_OUTPUT(gimbal, gimbal->impl, output);
}

GimbalStatus gimbal_sync(Gimbal *gimbal)
{
    if (gimbal == NULL) {
        return GimbalNull;
    }

    if (gimbal_ready(gimbal) == 0U) {
        return GimbalState;
    }

    return GIMBAL_IMPL_SYNC(gimbal, gimbal->impl);
}

GimbalStatus gimbal_set_cali(Gimbal *gimbal, const GimbalCali *cali)
{
    if ((gimbal == NULL) || (cali == NULL)) {
        return GimbalNull;
    }

    if (gimbal_ready(gimbal) == 0U) {
        return GimbalState;
    }

    return GIMBAL_IMPL_SET_CALI(gimbal, gimbal->impl, cali);
}

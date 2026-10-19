/**
 ******************************************************************************
 * @file    chassis.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   底盘机构公共接口实现。
 *
 * @note    本文件只负责公共上下文绑定、参数检查和编译期构型宏转发。
 * @note    具体控制律、反馈映射和输出生成必须放在 Src 下的构型实现中。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "chassis.h"

#include <string.h>

#if CHASSIS_TYPE == CHASSIS_TYPE_MECANUM
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
ChassisStatus chassis_mecanum_init(Chassis *chassis, void *impl);
ChassisStatus chassis_mecanum_step(Chassis *chassis, void *impl, const ChassisInput *input, ChassisOutput *output);
ChassisStatus chassis_mecanum_update(Chassis *chassis, void *impl, const ChassisInput *input);
ChassisStatus chassis_mecanum_control(Chassis *chassis, void *impl, const ChassisInput *input);
ChassisStatus chassis_mecanum_output(const Chassis *chassis, void *impl, ChassisOutput *output);
ChassisStatus chassis_mecanum_sync(Chassis *chassis, void *impl);
#define CHASSIS_IMPL_INIT chassis_mecanum_init
#define CHASSIS_IMPL_STEP chassis_mecanum_step
#define CHASSIS_IMPL_UPDATE chassis_mecanum_update
#define CHASSIS_IMPL_CONTROL chassis_mecanum_control
#define CHASSIS_IMPL_OUTPUT chassis_mecanum_output
#define CHASSIS_IMPL_SYNC chassis_mecanum_sync
#elif CHASSIS_TYPE == CHASSIS_TYPE_OMNI
ChassisStatus chassis_omni_init(Chassis *chassis, void *impl);
ChassisStatus chassis_omni_step(Chassis *chassis, void *impl, const ChassisInput *input, ChassisOutput *output);
ChassisStatus chassis_omni_update(Chassis *chassis, void *impl, const ChassisInput *input);
ChassisStatus chassis_omni_control(Chassis *chassis, void *impl, const ChassisInput *input);
ChassisStatus chassis_omni_output(const Chassis *chassis, void *impl, ChassisOutput *output);
ChassisStatus chassis_omni_sync(Chassis *chassis, void *impl);
#define CHASSIS_IMPL_INIT chassis_omni_init
#define CHASSIS_IMPL_STEP chassis_omni_step
#define CHASSIS_IMPL_UPDATE chassis_omni_update
#define CHASSIS_IMPL_CONTROL chassis_omni_control
#define CHASSIS_IMPL_OUTPUT chassis_omni_output
#define CHASSIS_IMPL_SYNC chassis_omni_sync
#elif CHASSIS_TYPE == CHASSIS_TYPE_STEERING
ChassisStatus chassis_steering_init(Chassis *chassis, void *impl);
ChassisStatus chassis_steering_step(Chassis *chassis, void *impl, const ChassisInput *input, ChassisOutput *output);
ChassisStatus chassis_steering_update(Chassis *chassis, void *impl, const ChassisInput *input);
ChassisStatus chassis_steering_control(Chassis *chassis, void *impl, const ChassisInput *input);
ChassisStatus chassis_steering_output(const Chassis *chassis, void *impl, ChassisOutput *output);
ChassisStatus chassis_steering_sync(Chassis *chassis, void *impl);
#define CHASSIS_IMPL_INIT chassis_steering_init
#define CHASSIS_IMPL_STEP chassis_steering_step
#define CHASSIS_IMPL_UPDATE chassis_steering_update
#define CHASSIS_IMPL_CONTROL chassis_steering_control
#define CHASSIS_IMPL_OUTPUT chassis_steering_output
#define CHASSIS_IMPL_SYNC chassis_steering_sync
#elif CHASSIS_TYPE == CHASSIS_TYPE_BALANCE
ChassisStatus chassis_balance_init(Chassis *chassis, void *impl);
ChassisStatus chassis_balance_step(Chassis *chassis, void *impl, const ChassisInput *input, ChassisOutput *output);
ChassisStatus chassis_balance_update(Chassis *chassis, void *impl, const ChassisInput *input);
ChassisStatus chassis_balance_control(Chassis *chassis, void *impl, const ChassisInput *input);
ChassisStatus chassis_balance_output(const Chassis *chassis, void *impl, ChassisOutput *output);
ChassisStatus chassis_balance_sync(Chassis *chassis, void *impl);
#define CHASSIS_IMPL_INIT chassis_balance_init
#define CHASSIS_IMPL_STEP chassis_balance_step
#define CHASSIS_IMPL_UPDATE chassis_balance_update
#define CHASSIS_IMPL_CONTROL chassis_balance_control
#define CHASSIS_IMPL_OUTPUT chassis_balance_output
#define CHASSIS_IMPL_SYNC chassis_balance_sync
#else
#error "Unsupported CHASSIS_TYPE"
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
 * @brief 判断底盘公共上下文是否已经初始化。
 *
 * @param chassis 底盘公共上下文。
 * @return 已初始化返回 1，否则返回 0。
 */
static uint8_t chassis_ready(const Chassis *chassis)
{
    if ((chassis == NULL) || (chassis->initialized == 0U)) {
        return 0U;
    }

    return 1U;
}

ChassisStatus chassis_init(Chassis *chassis, const ChassisCfg *cfg, void *impl)
{
    ChassisStatus status;

    if ((chassis == NULL) || (cfg == NULL)) {
        return ChassisNull;
    }

    if ((uint32_t)cfg->type != (uint32_t)CHASSIS_TYPE) {
        return ChassisArg;
    }

    memset(chassis, 0, sizeof(*chassis));
    chassis->cfg = *cfg;
    chassis->impl = impl;

    status = CHASSIS_IMPL_INIT(chassis, impl);
    if (status != ChassisOk) {
        memset(chassis, 0, sizeof(*chassis));
        return status;
    }

    chassis->initialized = 1U;
    return ChassisOk;
}

ChassisStatus chassis_step(Chassis *chassis, const ChassisInput *input, ChassisOutput *output)
{
    if ((chassis == NULL) || (input == NULL) || (output == NULL)) {
        return ChassisNull;
    }

    memset(output, 0, sizeof(*output));
    if (chassis_ready(chassis) == 0U) {
        return ChassisState;
    }

    return CHASSIS_IMPL_STEP(chassis, chassis->impl, input, output);
}

ChassisStatus chassis_update(Chassis *chassis, const ChassisInput *input)
{
    if ((chassis == NULL) || (input == NULL)) {
        return ChassisNull;
    }

    if (chassis_ready(chassis) == 0U) {
        return ChassisState;
    }

    return CHASSIS_IMPL_UPDATE(chassis, chassis->impl, input);
}

ChassisStatus chassis_control(Chassis *chassis, const ChassisInput *input)
{
    if ((chassis == NULL) || (input == NULL)) {
        return ChassisNull;
    }

    if (chassis_ready(chassis) == 0U) {
        return ChassisState;
    }

    return CHASSIS_IMPL_CONTROL(chassis, chassis->impl, input);
}

ChassisStatus chassis_output(const Chassis *chassis, ChassisOutput *output)
{
    if ((chassis == NULL) || (output == NULL)) {
        return ChassisNull;
    }

    memset(output, 0, sizeof(*output));
    if (chassis_ready(chassis) == 0U) {
        return ChassisState;
    }

    return CHASSIS_IMPL_OUTPUT(chassis, chassis->impl, output);
}

ChassisStatus chassis_sync(Chassis *chassis)
{
    if (chassis == NULL) {
        return ChassisNull;
    }

    if (chassis_ready(chassis) == 0U) {
        return ChassisState;
    }

    return CHASSIS_IMPL_SYNC(chassis, chassis->impl);
}

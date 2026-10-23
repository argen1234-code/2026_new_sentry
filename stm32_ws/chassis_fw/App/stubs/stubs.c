/**
  * @file       stubs.c
  * @brief      上游底盘代码所需的最小桩实现
  * @details    本工程只保留全向轮速度直驱：
  *             - 行为层：主机命令有效时进入 SPIN 分支直通 vx/vy/wz，
  *               无效时进入 NO_MOVE；
  *             - 云台 / IMU / 遥控器 / 功率计：全部返回空或缺省值；
  *             - 在线检测：保留上游 TOE 编号，按最后反馈时刻判超时。
  */

#include <string.h>

#include "chassis_behaviour.h"
#include "chassis_power_control.h"
#include "detect_task.h"
#include "gimbal_task.h"
#include "host_cmd.h"
#include "hwt_imu.h"
#include "legacy_bridge.h"
#include "pm01_api.h"
#include "remote_control.h"
#include "stm32h7xx_hal.h"
#include "wattmeter_api.h"

#define STUB_TOE_TIMEOUT_MS 100u

static uint32_t s_toe_last_ms[ERROR_LIST_LENGHT];
static uint8_t s_toe_hooked[ERROR_LIST_LENGHT];
static RC_ctrl_t s_rc_zero;

/* ========================= 行为层 ========================= */

void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    host_chassis_cmd_t cmd;

    if (chassis_move_mode == NULL)
    {
        return;
    }

    /* 速度直驱借用上游 SPIN 分支：该分支不做角度闭环，vx/vy/wz 原样下发。
     * 注意不能使用上游的 chassis_spin_control()，它会把 wz 固定成常值。 */
    chassis_move_mode->chassis_mode = (legacy_bridge_get_chassis_cmd(&cmd) != 0u)
                                          ? CHASSIS_VECTOR_SPIN
                                          : CHASSIS_VECTOR_NO_MOVE;
}

void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set,
                                   fp32 *wz_set,
                                   chassis_move_t *chassis_move_rc_to_vector)
{
    host_chassis_cmd_t cmd;

    (void)chassis_move_rc_to_vector;

    if ((vx_set == NULL) || (vy_set == NULL) || (wz_set == NULL))
    {
        return;
    }

    if (legacy_bridge_get_chassis_cmd(&cmd) != 0u)
    {
        *vx_set = cmd.vx_m_s;
        *vy_set = cmd.vy_m_s;
        *wz_set = HOST_CHASSIS_WZ_SIGN * cmd.wz_rad_s;
    }
    else
    {
        *vx_set = 0.0f;
        *vy_set = 0.0f;
        *wz_set = 0.0f;
    }
}

void chassis_power_control(chassis_move_t *chassis_move)
{
    (void)chassis_move; /* 本工程不做功率限制 */
}

/* ========================= 云台 / IMU / 遥控器 ========================= */

const gimbal_motor_t *get_yaw_motor_point(void)
{
    return NULL;
}

const gimbal_motor_t *get_pitch_motor_point(void)
{
    return NULL;
}

const hwt_imu_info_t *hwt906_get_info(void)
{
    return NULL;
}

const float *hwt906_get_gimbal_angle_point(void)
{
    return NULL;
}

RC_ctrl_t *get_remote_control_point(void)
{
    return &s_rc_zero;
}

/* ========================= 在线检测 ========================= */

void detect_hook(uint8_t toe)
{
    if (toe < ERROR_LIST_LENGHT)
    {
        s_toe_last_ms[toe] = HAL_GetTick();
        s_toe_hooked[toe] = 1u;
    }
}

bool_t toe_is_error(uint8_t err)
{
    if (err >= ERROR_LIST_LENGHT)
    {
        return 1u;
    }

    /* 本工程没有 DBUS 遥控器，底盘在线判定不应因此失败 */
    if (err == DBUS_TOE)
    {
        return 0u;
    }

    if (s_toe_hooked[err] == 0u)
    {
        return 1u;
    }

    return ((uint32_t)(HAL_GetTick() - s_toe_last_ms[err]) > STUB_TOE_TIMEOUT_MS) ? 1u : 0u;
}

/* ========================= 未接外设 ========================= */

void pm01_response_handle(uint16_t can_id, const uint8_t *can_rx_data)
{
    (void)can_id;
    (void)can_rx_data;
}

void wattmeter_feedback_handle(uint16_t can_id, const uint8_t *can_rx_data, uint8_t len)
{
    (void)can_id;
    (void)can_rx_data;
    (void)len;
}

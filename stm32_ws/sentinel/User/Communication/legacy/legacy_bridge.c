/**
  * @file       legacy_bridge.c
  * @brief      旧版 usb_bridge 协议兼容通道实现
  * @details    解析旧协议下行帧、翻译底盘命令、按旧格式回传状态。
  *             状态数据全部取自上游现有数据源：
  *               - 底盘速度/模式：chassis_move
  *               - 四轮反馈：CHASSIS_MOTOR_MEASURE
  *               - IMU：hwt906
  *               - 云台：gimbal_task 反馈
  *             不新增任何控制量，也不绕过上游的安全逻辑。
  */

#include "legacy_bridge.h"

#include <math.h>
#include <string.h>

#include "chassis_task.h"
#include "gimbal_task.h"
#include "hwt_imu.h"
#include "robot_param.h"
#include "stm32h7xx_hal.h"
#include "usbd_cdc_if.h"

/* chassis_task.c 中的全局底盘控制结构 */
extern chassis_move_t chassis_move;

#define LEGACY_BRIDGE_PI 3.14159265358979323846f
#define LEGACY_M3508_ECD_PER_REV 8192.0f

/* 主机 wz（ROS REP-103，逆时针为正）与上游底盘内部 wz（chas_inv_cal
 * 以顺时针为正，见 comm_app.c 中 ch_wz = -wz_rad_s / 0.01）符号相反。
 * 下行命令经 comm_app 注入时已取反，上行 0x81 在这里换算回主机方向。 */
#define LEGACY_HOST_CHASSIS_WZ_SIGN (-1.0f)

/* 0x83 帧长固定为 4 轮，轮数变化必须同步修改协议层与 ROS protocol.py */
#if CHASSIS_MODULE_NUM != 4
#error "legacy 0x83 WheelState assumes exactly 4 chassis wheels"
#endif

static legacy_parser_t s_parser;
static gimbal_chassis_cmd_t s_cmd;
static uint8_t s_cmd_valid;
static uint32_t s_cmd_rx_ms;
static uint32_t s_link_last_ms;
/* 每类帧独立递增：ROS usb_bridge 按消息类型检查 seq 连续性 */
static uint8_t s_seq_state;
static uint8_t s_seq_heartbeat;
static uint8_t s_seq_wheel;
static uint8_t s_seq_imu;

static uint32_t s_last_state_ms;
static uint32_t s_last_wheel_ms;
static uint32_t s_last_imu_ms;
static uint32_t s_last_hb_ms;

static uint32_t s_imu_ok_count;
static uint32_t s_imu_last_ok_ms;

/* ========================= 时间与发送 ========================= */

static uint8_t legacy_due(uint32_t now_ms, uint32_t *last_ms, uint32_t period_ms)
{
    if ((*last_ms == 0u) || ((uint32_t)(now_ms - *last_ms) >= period_ms))
    {
        *last_ms = now_ms;
        return 1u;
    }
    return 0u;
}

static void legacy_send(uint8_t msg_id, uint8_t *seq,
                        const uint8_t *payload, uint16_t payload_len)
{
    uint8_t frame[LEGACY_MAX_FRAME];
    uint16_t len = legacy_encode_frame(frame, msg_id, (*seq)++, payload, (uint8_t)payload_len);

    if (len != 0u)
    {
        (void)usb_cdc_write_queued(frame, (uint32_t)len);
    }
}

/* ========================= 下行帧处理 ========================= */

static void legacy_bridge_on_frame(const legacy_frame_t *frame, void *user)
{
    uint32_t now_ms = HAL_GetTick();

    (void)user;
    if (now_ms == 0u)
    {
        now_ms = 1u; /* 0 作为“从未收到”的哨兵值 */
    }
    s_link_last_ms = now_ms;

    if (frame->msg_id == LEGACY_ID_UNIFIED_CMD)
    {
        legacy_unified_cmd_t decoded;
        int32_t vx_mm_s;
        int32_t vy_mm_s;
        int32_t wz_mdeg_s;
        int32_t mode;

        if (legacy_decode_unified_cmd(frame, &decoded) == 0u)
        {
            return;
        }

        legacy_map_chassis_cmd(&decoded, &vx_mm_s, &vy_mm_s, &wz_mdeg_s, &mode);

        s_cmd.vx_mm_s = vx_mm_s;
        s_cmd.vy_mm_s = vy_mm_s;
        s_cmd.wz_mdeg_s = wz_mdeg_s;
        s_cmd.mode = mode;
        s_cmd.status = decoded.chassis_status;
        s_cmd.ts_us = decoded.host_ts_ns / 1000u;

        s_cmd_rx_ms = now_ms;
        s_cmd_valid = ((decoded.chassis_status & LEGACY_CMD_VALID) != 0u) ? 1u : 0u;
    }
}

void legacy_bridge_init(void)
{
    legacy_parser_init(&s_parser);
    memset(&s_cmd, 0, sizeof(s_cmd));
    s_cmd_valid = 0u;
    s_cmd_rx_ms = 0u;
    s_link_last_ms = 0u;
    s_seq_state = 0u;
    s_seq_heartbeat = 0u;
    s_seq_wheel = 0u;
    s_seq_imu = 0u;
    s_last_state_ms = 0u;
    s_last_wheel_ms = 0u;
    s_last_imu_ms = 0u;
    s_last_hb_ms = 0u;
    s_imu_ok_count = 0u;
    s_imu_last_ok_ms = 0u;
}

void legacy_bridge_feed_rx(const uint8_t *data,
                           uint32_t len,
                           legacy_pass_fn pass,
                           void *user)
{
    legacy_parser_feed(&s_parser, data, len, pass, legacy_bridge_on_frame, NULL);
}

/* ========================= 上行状态 ========================= */

static uint8_t legacy_mode_from_chassis(chassis_mode_e mode)
{
    switch (mode)
    {
    case CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW:
        return (uint8_t)LEGACY_CHASSIS_MODE_FOLLOW_YAW;
    case CHASSIS_VECTOR_YAW_HOLD:
        return 4u; /* ChassisModeFollowChassisYaw */
    case CHASSIS_VECTOR_SPIN:
        return (uint8_t)LEGACY_CHASSIS_MODE_SPIN;
    case CHASSIS_VECTOR_NO_MOVE:
    default:
        return (uint8_t)LEGACY_CHASSIS_MODE_STOP;
    }
}

static void legacy_send_mcu_state(uint32_t now_ms)
{
    uint8_t payload[LEGACY_MCU_STATE_LEN];
    uint16_t pos = 0u;
    const float *angle = get_INS_angle_point();
    const gimbal_motor_t *yaw_motor = get_yaw_motor_point();
    const gimbal_motor_t *pitch_motor = get_pitch_motor_point();
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float gimbal_yaw = 0.0f;
    float gimbal_pitch = 0.0f;
    uint8_t status = 0x01u; /* bit0：主机在线 */

    if (angle != NULL)
    {
        yaw = angle[HWT_GIMBAL_YAW_INDEX];
        pitch = angle[HWT_GIMBAL_PITCH_INDEX];
        roll = angle[HWT_GIMBAL_ROLL_INDEX];
        gimbal_yaw = yaw;
        gimbal_pitch = pitch;
        status |= 0x02u; /* bit1：INS 有效 */
    }
    if ((yaw_motor != NULL) && (pitch_motor != NULL))
    {
        gimbal_yaw = yaw_motor->relative_angle;
        gimbal_pitch = pitch_motor->relative_angle;
        status |= 0x04u; /* bit2：云台反馈在线 */
    }

    legacy_write_u32(&payload[pos], now_ms);
    pos += 4u;
    legacy_write_f32(&payload[pos], roll);
    pos += 4u;
    legacy_write_f32(&payload[pos], pitch);
    pos += 4u;
    legacy_write_f32(&payload[pos], yaw);
    pos += 4u;
    legacy_write_f32(&payload[pos], gimbal_yaw);
    pos += 4u;
    legacy_write_f32(&payload[pos], gimbal_pitch);
    pos += 4u;
    legacy_write_f32(&payload[pos], chassis_move.vx);
    pos += 4u;
    legacy_write_f32(&payload[pos], chassis_move.vy);
    pos += 4u;
    legacy_write_f32(&payload[pos], LEGACY_HOST_CHASSIS_WZ_SIGN * chassis_move.wz);
    pos += 4u;
    payload[pos++] = legacy_mode_from_chassis(chassis_move.chassis_mode);
    payload[pos++] = status;
    legacy_write_u16(&payload[pos], 0u); /* error_flags：上游无对应位图 */
    pos += 2u;

    legacy_send(LEGACY_ID_MCU_STATE, &s_seq_state, payload, pos);
}

static void legacy_send_wheel_state(uint32_t now_ms)
{
    uint8_t payload[LEGACY_WHEEL_STATE_LEN];
    uint16_t pos = 0u;
    uint8_t online_mask = 0u;

    legacy_write_u32(&payload[pos], now_ms);
    pos += 4u;
    pos += 1u; /* online_mask 占位，下面回填 */
    payload[pos++] = legacy_mode_from_chassis(chassis_move.chassis_mode);
    legacy_write_u16(&payload[pos], 0u); /* chassis_status */
    pos += 2u;
    legacy_write_u16(&payload[pos], 0u); /* fault_flags */
    pos += 2u;

    for (uint8_t i = 0u; i < CHASSIS_MODULE_NUM; i++)
    {
        const motor_measure_t *measure = get_chassis_motor_measure_point(i);
        float speed_mps = 0.0f;

        if (measure != NULL)
        {
            speed_mps = (float)measure->speed_rpm / MPS_to_RPM;
            if ((measure->last_fdb_time != 0u) &&
                ((uint32_t)(now_ms - measure->last_fdb_time) < 100u))
            {
                online_mask |= (uint8_t)(1u << i);
            }
        }
        legacy_write_f32(&payload[pos], speed_mps);
        pos += 4u;
    }

    for (uint8_t i = 0u; i < CHASSIS_MODULE_NUM; i++)
    {
        const motor_measure_t *measure = get_chassis_motor_measure_point(i);
        float position_rad = 0.0f;

        if (measure != NULL)
        {
            position_rad = ((float)measure->ecd / LEGACY_M3508_ECD_PER_REV) *
                           (2.0f * LEGACY_BRIDGE_PI);
        }
        legacy_write_f32(&payload[pos], position_rad);
        pos += 4u;
    }

    for (uint8_t i = 0u; i < CHASSIS_MODULE_NUM; i++)
    {
        const motor_measure_t *measure = get_chassis_motor_measure_point(i);

        payload[pos++] = (measure != NULL) ? measure->temperate : 0u;
    }

    for (uint8_t i = 0u; i < CHASSIS_MODULE_NUM; i++)
    {
        payload[pos++] = 0u; /* 3508 无电机温度反馈 */
    }

    for (uint8_t i = 0u; i < CHASSIS_MODULE_NUM; i++)
    {
        float torque_mnm = chassis_move.chassis_3508[i].given_current_a *
                           M3508_TORQUE_CONSTANT * M3508_REDUCTION_RATIO * 1000.0f;
        int16_t torque;

        if (torque_mnm > 32767.0f)
        {
            torque_mnm = 32767.0f;
        }
        else if (torque_mnm < -32768.0f)
        {
            torque_mnm = -32768.0f;
        }
        torque = (int16_t)torque_mnm;
        legacy_write_u16(&payload[pos], (uint16_t)torque);
        pos += 2u;
    }

    payload[4] = online_mask;
    legacy_send(LEGACY_ID_WHEEL_STATE, &s_seq_wheel, payload, pos);
}

static void legacy_euler_to_quat(float roll, float pitch, float yaw, float *wxyz)
{
    const float cr = cosf(roll * 0.5f);
    const float sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f);
    const float sp = sinf(pitch * 0.5f);
    const float cy = cosf(yaw * 0.5f);
    const float sy = sinf(yaw * 0.5f);

    wxyz[0] = (cr * cp * cy) + (sr * sp * sy);
    wxyz[1] = (sr * cp * cy) - (cr * sp * sy);
    wxyz[2] = (cr * sp * cy) + (sr * cp * sy);
    wxyz[3] = (cr * cp * sy) - (sr * sp * cy);
}

static void legacy_send_imu_state(uint32_t now_ms)
{
    uint8_t payload[LEGACY_IMU_STATE_LEN];
    uint16_t pos = 0u;
    const hwt_imu_info_t *info = hwt906_get_info();
    const float *gyro = hwt906_get_gimbal_gyro_point();
    const float *accel = hwt906_get_gimbal_accel_point();
    float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float temperature_c = 0.0f;
    uint8_t valid_mask = 0u;

    if (info != NULL)
    {
        if (info->ok_frame_count != s_imu_ok_count)
        {
            s_imu_ok_count = info->ok_frame_count;
            s_imu_last_ok_ms = now_ms;
        }
        /* 数据超过 100 ms 未更新时撤销有效位 */
        if ((s_imu_ok_count != 0u) &&
            ((uint32_t)(now_ms - s_imu_last_ok_ms) <= 100u))
        {
            valid_mask = 0x07u; /* bit0 陀螺+加速度，bit1 姿态，bit2 温度 */
        }
        temperature_c = info->accel.temp_c;
        legacy_euler_to_quat(info->angle.rad[HWT_AXIS_ROLL],
                             info->angle.rad[HWT_AXIS_PITCH],
                             info->angle.yaw_total_rad,
                             quat);
    }

    legacy_write_u32(&payload[pos], now_ms);
    pos += 4u;

    for (uint8_t i = 0u; i < 3u; i++)
    {
        legacy_write_f32(&payload[pos], (gyro != NULL) ? gyro[i] : 0.0f);
        pos += 4u;
    }
    for (uint8_t i = 0u; i < 3u; i++)
    {
        legacy_write_f32(&payload[pos], (accel != NULL) ? accel[i] : 0.0f);
        pos += 4u;
    }
    legacy_write_f32(&payload[pos], temperature_c);
    pos += 4u;
    for (uint8_t i = 0u; i < 4u; i++)
    {
        legacy_write_f32(&payload[pos], quat[i]);
        pos += 4u;
    }
    legacy_write_i32(&payload[pos], 0); /* imu_status */
    pos += 4u;
    payload[pos++] = valid_mask;
    payload[pos++] = 0u; /* reserved */
    legacy_write_u32(&payload[pos], s_imu_last_ok_ms);
    pos += 4u;

    legacy_send(LEGACY_ID_IMU_STATE, &s_seq_imu, payload, pos);
}

static void legacy_send_heartbeat(uint32_t now_ms)
{
    uint8_t payload[LEGACY_MCU_HEARTBEAT_LEN];

    legacy_write_u32(&payload[0], now_ms);
    payload[4] = legacy_mode_from_chassis(chassis_move.chassis_mode);
    payload[5] = 0x01u; /* 主机在线 */
    legacy_write_u16(&payload[6], 0u);

    legacy_send(LEGACY_ID_MCU_HEARTBEAT, &s_seq_heartbeat, payload, sizeof(payload));
}

/* ========================= 对外接口 ========================= */

uint8_t legacy_bridge_active(void)
{
    if (s_link_last_ms == 0u)
    {
        return 0u;
    }
    return ((uint32_t)(HAL_GetTick() - s_link_last_ms) <= LEGACY_BRIDGE_LINK_TIMEOUT_MS) ? 1u : 0u;
}

uint8_t legacy_bridge_get_chassis_cmd(gimbal_chassis_cmd_t *out)
{
    if ((s_cmd_valid == 0u) || (out == NULL))
    {
        return 0u;
    }
    if ((uint32_t)(HAL_GetTick() - s_cmd_rx_ms) > LEGACY_BRIDGE_CMD_TIMEOUT_MS)
    {
        s_cmd_valid = 0u;
        return 0u;
    }

    *out = s_cmd;
    return 1u;
}

void legacy_bridge_tick(void)
{
    uint32_t now_ms;

    if (legacy_bridge_active() == 0u)
    {
        return;
    }

    now_ms = HAL_GetTick();

    if (legacy_due(now_ms, &s_last_state_ms, LEGACY_BRIDGE_STATE_PERIOD_MS) != 0u)
    {
        legacy_send_mcu_state(now_ms);
    }
    if (legacy_due(now_ms, &s_last_wheel_ms, LEGACY_BRIDGE_STATE_PERIOD_MS) != 0u)
    {
        legacy_send_wheel_state(now_ms);
    }
    if (legacy_due(now_ms, &s_last_imu_ms, LEGACY_BRIDGE_STATE_PERIOD_MS) != 0u)
    {
        legacy_send_imu_state(now_ms);
    }
    if (legacy_due(now_ms, &s_last_hb_ms, LEGACY_BRIDGE_HEARTBEAT_PERIOD_MS) != 0u)
    {
        legacy_send_heartbeat(now_ms);
    }
}

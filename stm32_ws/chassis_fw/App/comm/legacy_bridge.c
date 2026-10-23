/**
  * @file       legacy_bridge.c
  * @brief      旧版 usb_bridge 协议兼容通道实现（本工程精简版）
  * @details    解析旧协议下行帧、翻译为底盘速度指令、按旧格式回传状态。
  *             状态数据取自：
  *               - 底盘速度/模式：chassis_move
  *               - 四轮反馈：CHASSIS_MOTOR_MEASURE
  *             本工程不含云台与 IMU，对应字段固定输出零。
  */

#include "legacy_bridge.h"

#include <string.h>

#include "bsp_fdcan.h"
#include "chassis_task.h"
#include "robot_param.h"
#include "stm32h7xx_hal.h"
#include "usbd_cdc_if.h"

/* chassis_task.c 中的全局底盘控制结构 */
extern chassis_move_t chassis_move;

#define LEGACY_BRIDGE_PI 3.14159265358979323846f
#define LEGACY_M3508_ECD_PER_REV 8192.0f
#define LEGACY_MDEG_TO_RAD 1.7453292519943296e-5f

/* 0x83 帧长固定为 4 轮，轮数变化必须同步修改协议层与 ROS protocol.py */
#if CHASSIS_MODULE_NUM != 4
#error "legacy 0x83 WheelState assumes exactly 4 chassis wheels"
#endif

static legacy_parser_t s_parser;
static host_chassis_cmd_t s_cmd;
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

        /* 只有 CMD_VALID 与 CMD_MOTION_ENABLE 同时置位，且模式为
         * NoFollow（本工程唯一支持的模式）时才驱动底盘。 */
        s_cmd.valid = (((decoded.chassis_status & LEGACY_CMD_VALID) != 0u) &&
                       ((decoded.chassis_status & LEGACY_CMD_MOTION_ENABLE) != 0u) &&
                       (decoded.chassis_mode == LEGACY_CHASSIS_MODE_NO_FOLLOW))
                          ? 1u
                          : 0u;
        s_cmd.vx_m_s = (float)vx_mm_s * 0.001f;
        s_cmd.vy_m_s = (float)vy_mm_s * 0.001f;
        s_cmd.wz_rad_s = (float)wz_mdeg_s * LEGACY_MDEG_TO_RAD;

        s_cmd_rx_ms = now_ms;
        s_cmd_valid = s_cmd.valid;
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
}

void legacy_bridge_feed_rx(const uint8_t *data, uint32_t len)
{
    legacy_parser_feed(&s_parser, data, len, NULL, legacy_bridge_on_frame, NULL);
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
        /* 本工程的 SPIN 分支只是速度直驱的载体，对主机而言仍是 NoFollow */
        return (uint8_t)LEGACY_CHASSIS_MODE_NO_FOLLOW;
    case CHASSIS_VECTOR_NO_MOVE:
    default:
        return (uint8_t)LEGACY_CHASSIS_MODE_STOP;
    }
}

static void legacy_send_mcu_state(uint32_t now_ms)
{
    uint8_t payload[LEGACY_MCU_STATE_LEN];
    uint16_t pos = 0u;
    uint8_t status = 0x01u; /* bit0：主机在线；bit1/bit2 本工程无 IMU/云台 */

    legacy_write_u32(&payload[pos], now_ms);
    pos += 4u;
    legacy_write_f32(&payload[pos], 0.0f); /* roll */
    pos += 4u;
    legacy_write_f32(&payload[pos], 0.0f); /* pitch */
    pos += 4u;
    legacy_write_f32(&payload[pos], 0.0f); /* yaw */
    pos += 4u;
    legacy_write_f32(&payload[pos], 0.0f); /* gimbal_yaw */
    pos += 4u;
    legacy_write_f32(&payload[pos], 0.0f); /* gimbal_pitch */
    pos += 4u;
    /* chassis_move 的 vx/vy 与主机同向，wz 为上游内部正方向（顺时针），
     * 回传前换算回主机 REP-103（逆时针为正）。 */
    legacy_write_f32(&payload[pos], chassis_move.vx);
    pos += 4u;
    legacy_write_f32(&payload[pos], chassis_move.vy);
    pos += 4u;
    legacy_write_f32(&payload[pos], HOST_CHASSIS_WZ_SIGN * chassis_move.wz);
    pos += 4u;
    payload[pos++] = legacy_mode_from_chassis(chassis_move.chassis_mode);
    payload[pos++] = status;
    legacy_write_u16(&payload[pos], 0u); /* error_flags */
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
            /* 编码器是减速箱前的转子角，换算到轮轴输出角，与 URDF 轮关节一致 */
            position_rad = ((float)measure->ecd / LEGACY_M3508_ECD_PER_REV) *
                           (2.0f * LEGACY_BRIDGE_PI) / M3508_REDUCTION_RATIO;
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

static void legacy_send_imu_state(uint32_t now_ms)
{
    uint8_t payload[LEGACY_IMU_STATE_LEN];

    /* 本工程不接 IMU：陀螺、加速度、温度、四元数全部为零，valid_mask 为 0。
     * 布局：ts(4) gyro(12) accel(12) temp(4) quat(16) status(4) mask(1)
     *       reserved(1) last_ok_ms(4)，共 58 B。 */
    memset(payload, 0, sizeof(payload));
    legacy_write_u32(&payload[0], now_ms);
    payload[52] = 0u; /* valid_mask */
    legacy_write_u32(&payload[54], 0u);

    legacy_send(LEGACY_ID_IMU_STATE, &s_seq_imu, payload, LEGACY_IMU_STATE_LEN);
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

uint8_t legacy_bridge_get_chassis_cmd(host_chassis_cmd_t *out)
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

uint32_t legacy_bridge_rx_frames(void)
{
    return s_parser.rx_frames;
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

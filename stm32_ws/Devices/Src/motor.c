/**
 ******************************************************************************
 * @file    motor.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-04-29
 * @brief   DM-MC02 电机设备实现。
 *
 * @note    本文件只做板端 FDCAN 绑定和电机反馈状态维护，协议字节序由
 *          Devices/Src 下的 DjiMotor、MitMotor、DmMotor、RsMotor 负责。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "motor.h"

#include <string.h>

/** @brief DM 速度模式发送的 TX FIFO 最大等待时间，单位 ms。 */
#define MOTOR_DM_SPEED_TX_TIMEOUT_MS 1U

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置设备驱动的协议常量、默认参数和私有类型，硬件绑定来自 Robot board 配置。
 * ========================================================================== */

/**
 * @brief 单条 FDCAN 总线上的电机反馈状态。
 */
typedef struct
{
    DjiMotorFb dji[8];             /**< DJI 0x201 到 0x208 反馈。 */
    uint8_t dji_valid[8];          /**< DJI 反馈有效标志。 */
    MitMotorFb mit[8];             /**< MIT 0x51 到 0x58 反馈。 */
    MitMotorLimit mit_limit[8];    /**< MIT 反馈解码量程。 */
    uint32_t mit_last_time[8];     /**< MIT 最近反馈时间。 */
    uint8_t mit_valid[8];          /**< MIT 反馈有效标志。 */
} motor_bus_state_t;

/**
 * @brief FDCAN 总线订阅配置。
 */
typedef struct
{
    bsp_fdcan_id_t bus;    /**< FDCAN 总线标识。 */
    uint32_t fifo;         /**< RX FIFO 编号。 */
} motor_bus_cfg_t;

/**
 * @brief 云台 DJI 高组发送槽位缓存。
 *
 * yaw、pitch 和 trigger 分别由不同 app 更新，但 DJI 高组必须一次发送四个槽位。
 */
typedef struct
{
    int16_t yaw;      /**< yaw 槽位控制量。 */
    int16_t pitch;    /**< pitch 槽位控制量。 */
    int16_t trigger;  /**< trigger 槽位控制量。 */
    int16_t rev;      /**< 预留槽位控制量。 */
} motor_gimbal_group_t;

/**
 * @brief 默认板端总线订阅配置。
 */
static const motor_bus_cfg_t motor_bus_cfg[] = {
    {MOTOR_BUS0_ID, MOTOR_BUS0_RX_FIFO},
    {MOTOR_BUS1_ID, MOTOR_BUS1_RX_FIFO},
    {MOTOR_BUS2_ID, MOTOR_BUS2_RX_FIFO},
};

/* ==========================================================================
 * 状态缓存与工作区
 * --------------------------------------------------------------------------
 * 设备层只缓存原始设备状态、诊断和收发缓冲，不写机器人控制语义。
 * ========================================================================== */

/**
 * @brief 板端电机状态表，索引与 bsp_fdcan_id_t 一致。
 */
static motor_bus_state_t motor_state[BSP_FDCAN_ID_COUNT];

/** @brief 达妙参数帧广播 CAN ID。 */
#define MOTOR_DM_PARAM_BROADCAST_ID 0x7FFU

/** @brief 达妙参数应答最近一次的电机 ID。 */
static volatile uint32_t motor_dm_param_id;

/** @brief 达妙参数应答最近一次的参数 RID。 */
static volatile uint8_t motor_dm_param_rid;

/** @brief 达妙参数应答最近一次的数值。 */
static volatile uint32_t motor_dm_param_value;

/** @brief 达妙参数应答序号，用于区分新应答与旧缓存。 */
static volatile uint32_t motor_dm_param_seq_counter;

/** @brief 云台 DJI 高组最近一次发送槽位。 */
static motor_gimbal_group_t motor_gimbal_cache;

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责帧格式、数值范围、在线时间和底层返回值检查。
 * ========================================================================== */

/**
 * @brief 判断总线标识是否有效。
 *
 * @param bus FDCAN 总线标识。
 * @return 有效返回 1，否则返回 0。
 */
static uint8_t bus_valid(bsp_fdcan_id_t bus)
{
    return ((uint32_t)bus < (uint32_t)BSP_FDCAN_ID_COUNT) ? 1U : 0U;
}

/**
 * @brief 根据 DJI 反馈 ID 计算槽位。
 *
 * @param rx_id 反馈 ID。
 * @param index 输出槽位。
 * @return 成功返回 MotorOk。
 */
static motor_status_t dji_index(uint16_t rx_id, uint8_t *index)
{
    if (index == NULL) {
        return MotorNull;
    }

    if ((rx_id < MOTOR_DJI_FB_FIRST) || (rx_id > MOTOR_DJI_FB_LAST)) {
        return MotorArg;
    }

    *index = (uint8_t)(rx_id - MOTOR_DJI_FB_FIRST);
    return MotorOk;
}

/**
 * @brief 根据 MIT 反馈 ID 计算槽位。
 *
 * @param rx_id 反馈 ID。
 * @param index 输出槽位。
 * @return 成功返回 MotorOk。
 */
static motor_status_t mit_index(uint16_t rx_id, uint8_t *index)
{
    if (index == NULL) {
        return MotorNull;
    }

    if ((rx_id < MOTOR_MIT_FB_FIRST) || (rx_id > MOTOR_MIT_FB_LAST)) {
        return MotorArg;
    }

    *index = (uint8_t)(rx_id - MOTOR_MIT_FB_FIRST);
    return MotorOk;
}

/**
 * @brief 从 FDCAN 句柄反查板端总线 ID。
 *
 * @param hfdcan FDCAN 句柄。
 * @param bus 输出总线 ID。
 * @return 成功返回 MotorOk。
 */
static motor_status_t bus_from(FDCAN_HandleTypeDef *hfdcan, bsp_fdcan_id_t *bus)
{
    uint32_t i;

    if ((hfdcan == NULL) || (bus == NULL)) {
        return MotorNull;
    }

    for (i = 0U; i < (uint32_t)BSP_FDCAN_ID_COUNT; i++) {
        if (bsp_fdcan_handle((bsp_fdcan_id_t)i) == hfdcan) {
            *bus = (bsp_fdcan_id_t)i;
            return MotorOk;
        }
    }

    return MotorArg;
}

/**
 * @brief 进入临界区并返回原 PRIMASK。
 *
 * @return 原 PRIMASK。
 */
static uint32_t critical_enter(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

/**
 * @brief 恢复临界区前的 PRIMASK。
 *
 * @param primask 原 PRIMASK。
 */
static void critical_exit(uint32_t primask)
{
    __set_PRIMASK(primask);
}

/**
 * @brief 将协议层状态转换为电机设备状态。
 *
 * @param status MIT 协议层状态。
 * @return 电机设备状态。
 */
static motor_status_t mit_status(MitMotorStatus status)
{
    if (status == MitMotorOk) {
        return MotorOk;
    }

    if (status == MitMotorNull) {
        return MotorNull;
    }

    return MotorCodec;
}

/**
 * @brief 发送 8 字节标准 CAN 帧。
 *
 * @param bus FDCAN 总线标识。
 * @param id 标准 CAN ID。
 * @param data 数据区。
 * @return 电机设备状态。
 */
static motor_status_t send_8(bsp_fdcan_id_t bus, uint16_t id, const uint8_t data[8])
{
    FDCAN_HandleTypeDef *hfdcan;

    if (data == NULL) {
        return MotorNull;
    }

    if (bus_valid(bus) == 0U) {
        return MotorArg;
    }

    hfdcan = bsp_fdcan_handle(bus);
    if (hfdcan == NULL) {
        return MotorHal;
    }

    return (bsp_fdcan_send_std(hfdcan, id, data, 8U) == HAL_OK) ? MotorOk : MotorHal;
}

/**
 * @brief 带超时的 8 字节标准 CAN 帧发送。
 *
 * @param bus FDCAN 总线标识。
 * @param id 标准 CAN ID。
 * @param data 数据区。
 * @param timeout_ms FIFO 满时的最大等待时间。
 * @return 电机设备状态。
 */
static motor_status_t send_8_timeout(bsp_fdcan_id_t bus,
                                     uint16_t id,
                                     const uint8_t data[8],
                                     uint32_t timeout_ms)
{
    FDCAN_HandleTypeDef *hfdcan;

    if (data == NULL) {
        return MotorNull;
    }

    if (bus_valid(bus) == 0U) {
        return MotorArg;
    }

    hfdcan = bsp_fdcan_handle(bus);
    if (hfdcan == NULL) {
        return MotorHal;
    }

    return (bsp_fdcan_send_std_timeout(hfdcan, id, data, 8U, timeout_ms) == HAL_OK)
               ? MotorOk
               : MotorHal;
}

/**
 * @brief 解析一帧 DJI 反馈并写入状态表。
 *
 * @param bus FDCAN 总线标识。
 * @param rx_id 标准 CAN ID。
 * @param data 数据区。
 * @param len 数据长度。
 * @param now 当前 HAL tick。
 */
static void decode_dji(bsp_fdcan_id_t bus, uint16_t rx_id, const uint8_t *data, uint8_t len, uint32_t now)
{
    uint8_t index;

    if (dji_index(rx_id, &index) != MotorOk) {
        return;
    }

    if (dji_motor_decode(&motor_state[bus].dji[index], data, len, now) == DjiMotorOk) {
        motor_state[bus].dji_valid[index] = 1U;
    }
}

/**
 * @brief 解析一帧 MIT 反馈并写入状态表。
 *
 * @param bus FDCAN 总线标识。
 * @param rx_id 标准 CAN ID。
 * @param data 数据区。
 * @param len 数据长度。
 * @param now 当前 HAL tick。
 */
static void decode_mit(bsp_fdcan_id_t bus, uint16_t rx_id, const uint8_t *data, uint8_t len, uint32_t now)
{
    uint8_t index;

    if (mit_index(rx_id, &index) != MotorOk) {
        return;
    }

    if (mit_motor_decode(&motor_state[bus].mit[index], &motor_state[bus].mit_limit[index], data, len) == MitMotorOk) {
        motor_state[bus].mit_last_time[index] = now;
        motor_state[bus].mit_valid[index] = 1U;
    }
}

/* ==========================================================================
 * 收发、解析与打包
 * --------------------------------------------------------------------------
 * 这里把字节流或寄存器数据转换成设备事实；控制含义由 Component/Input 或 Apps 解释。
 * ========================================================================== */

/**
 * @brief FDCAN RX 订阅回调。
 *
 * @param hfdcan FDCAN 句柄。
 * @param rx_header 接收帧头。
 * @param rx_data 接收数据。
 * @param user 用户上下文，本回调未使用。
 */
static void motor_rx_cb(FDCAN_HandleTypeDef *hfdcan,
                        const FDCAN_RxHeaderTypeDef *rx_header,
                        const uint8_t *rx_data,
                        void *user)
{
    bsp_fdcan_id_t bus;
    uint8_t len;
    uint32_t now;
    uint16_t rx_id;

    (void)user;

    if ((rx_header == NULL) || (rx_data == NULL)) {
        return;
    }

    if ((rx_header->IdType != FDCAN_STANDARD_ID) || (rx_header->RxFrameType != FDCAN_DATA_FRAME)) {
        return;
    }

    if (bus_from(hfdcan, &bus) != MotorOk) {
        return;
    }

    len = bsp_fdcan_dlc_to_length(rx_header->DataLength);
    rx_id = (uint16_t)rx_header->Identifier;
    now = HAL_GetTick();

    /*
     * 达妙参数应答不是正常反馈：data[1]=0 且 data[2] 为 0x33/0x55/0xAA。
     * 单独记录最近一次应答，供启动时读取 CTRL_MODE 使用。
     */
    if ((len >= 8U) &&
        (rx_id >= MOTOR_MIT_FB_FIRST) && (rx_id <= MOTOR_MIT_FB_LAST) &&
        (rx_data[0] == (uint8_t)(rx_id - 0x50U)) &&
        (rx_data[1] == 0U) &&
        ((rx_data[2] == 0x33U) || (rx_data[2] == 0x55U) || (rx_data[2] == 0xAAU))) {
        motor_dm_param_id = (uint32_t)rx_data[0];
        motor_dm_param_rid = rx_data[3];
        motor_dm_param_value = (uint32_t)rx_data[4] |
                               ((uint32_t)rx_data[5] << 8) |
                               ((uint32_t)rx_data[6] << 16) |
                               ((uint32_t)rx_data[7] << 24);
        motor_dm_param_seq_counter++;
    }

    if ((rx_id >= MOTOR_DJI_FB_FIRST) && (rx_id <= MOTOR_DJI_FB_LAST)) {
        decode_dji(bus, rx_id, rx_data, len, now);
        return;
    }

    if ((rx_id >= MOTOR_MIT_FB_FIRST) && (rx_id <= MOTOR_MIT_FB_LAST)) {
        decode_mit(bus, rx_id, rx_data, len, now);
    }
}

/**
 * @brief 设置所有 MIT 反馈槽的默认量程。
 */
static void set_default_mit(void)
{
    uint32_t bus;
    uint32_t index;

    for (bus = 0U; bus < (uint32_t)BSP_FDCAN_ID_COUNT; bus++) {
        for (index = 0U; index < 8U; index++) {
            motor_state[bus].mit_limit[index] = DmMotorLimit4310;
        }
    }
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只提供初始化、喂入、查询和发送入口，不保留旧 task/global 兼容接口。
 * ========================================================================== */
motor_status_t motor_init(void)
{
    uint32_t i;

    (void)memset(motor_state, 0, sizeof(motor_state));
    set_default_mit();

    for (i = 0U; i < (sizeof(motor_bus_cfg) / sizeof(motor_bus_cfg[0])); i++) {
        FDCAN_HandleTypeDef *hfdcan = bsp_fdcan_handle(motor_bus_cfg[i].bus);
        if (hfdcan == NULL) {
            return MotorHal;
        }

        if (bsp_fdcan_rx_bind(hfdcan, motor_bus_cfg[i].fifo, motor_rx_cb, NULL) != HAL_OK) {
            return MotorHal;
        }
    }

    return MotorOk;
}

motor_status_t motor_mit_bind(bsp_fdcan_id_t bus, uint16_t rx_id, const MitMotorLimit *limit)
{
    uint8_t index;
    uint32_t primask;

    if (limit == NULL) {
        return MotorNull;
    }

    if (bus_valid(bus) == 0U) {
        return MotorArg;
    }

    if (mit_index(rx_id, &index) != MotorOk) {
        return MotorArg;
    }

    primask = critical_enter();
    motor_state[bus].mit_limit[index] = *limit;
    motor_state[bus].mit_valid[index] = 0U;
    critical_exit(primask);

    return MotorOk;
}

motor_status_t motor_dm_bind(bsp_fdcan_id_t bus, uint16_t rx_id, DmMotorModel model)
{
    const MitMotorLimit *limit = dm_motor_limit(model);

    if (limit == NULL) {
        return MotorArg;
    }

    return motor_mit_bind(bus, rx_id, limit);
}

motor_status_t motor_dji_get(bsp_fdcan_id_t bus, uint16_t rx_id, DjiMotorFb *out)
{
    uint8_t index;
    uint32_t primask;

    if (out == NULL) {
        return MotorNull;
    }

    if (bus_valid(bus) == 0U) {
        return MotorArg;
    }

    if (dji_index(rx_id, &index) != MotorOk) {
        return MotorArg;
    }

    primask = critical_enter();
    if (motor_state[bus].dji_valid[index] == 0U) {
        critical_exit(primask);
        return MotorNoData;
    }
    *out = motor_state[bus].dji[index];
    critical_exit(primask);

    return MotorOk;
}

motor_status_t motor_mit_get(bsp_fdcan_id_t bus, uint16_t rx_id, motor_mit_fb_t *out)
{
    uint8_t index;
    uint32_t primask;

    if (out == NULL) {
        return MotorNull;
    }

    if (bus_valid(bus) == 0U) {
        return MotorArg;
    }

    if (mit_index(rx_id, &index) != MotorOk) {
        return MotorArg;
    }

    primask = critical_enter();
    if (motor_state[bus].mit_valid[index] == 0U) {
        critical_exit(primask);
        return MotorNoData;
    }
    out->fb = motor_state[bus].mit[index];
    out->last_time = motor_state[bus].mit_last_time[index];
    critical_exit(primask);

    return MotorOk;
}

motor_status_t motor_dji_send(bsp_fdcan_id_t bus,
                                      uint16_t tx_id,
                                      int16_t m1,
                                      int16_t m2,
                                      int16_t m3,
                                      int16_t m4)
{
    uint8_t data[8];

    if (dji_motor_current(data, m1, m2, m3, m4) != DjiMotorOk) {
        return MotorCodec;
    }

    return send_8(bus, tx_id, data);
}

motor_status_t motor_dm_mit(bsp_fdcan_id_t bus, uint16_t id, DmMotorModel model, const MitMotorCmd *cmd)
{
    uint8_t data[8];
    MitMotorStatus status;

    status = dm_motor_mit(data, model, cmd);
    if (status != MitMotorOk) {
        return mit_status(status);
    }

    return send_8(bus, (uint16_t)(id + MIT_MOTOR_MODE_MIT), data);
}

motor_status_t motor_dm_speed(bsp_fdcan_id_t bus, uint16_t id, float vel)
{
    uint8_t data[8];
    MitMotorStatus status;

    status = dm_motor_speed(data, vel);
    if (status != MitMotorOk) {
        return mit_status(status);
    }

    return send_8_timeout(bus, (uint16_t)(id + MIT_MOTOR_MODE_SPEED), data,
                          MOTOR_DM_SPEED_TX_TIMEOUT_MS);
}

motor_status_t motor_dm_speed_cmd(bsp_fdcan_id_t bus, uint16_t id, DmMotorCmdKind kind)
{
    uint8_t data[8];
    MitMotorStatus status;

    status = dm_motor_cmd(data, kind);
    if (status != MitMotorOk) {
        return mit_status(status);
    }

    return send_8_timeout(bus, (uint16_t)(id + MIT_MOTOR_MODE_SPEED), data,
                          MOTOR_DM_SPEED_TX_TIMEOUT_MS);
}

motor_status_t motor_dm_speed_stop(bsp_fdcan_id_t bus, uint16_t id)
{
    motor_status_t status;

    status = motor_dm_speed(bus, id, 0.0F);
    if (status != MotorOk) {
        return status;
    }

    return motor_dm_speed_cmd(bus, id, DmMotorDisable);
}

motor_status_t motor_dm_cmd(bsp_fdcan_id_t bus, uint16_t id, DmMotorCmdKind kind)
{
    uint8_t data[8];
    MitMotorStatus status;

    status = dm_motor_cmd(data, kind);
    if (status != MitMotorOk) {
        return mit_status(status);
    }

    return send_8(bus, id, data);
}

motor_status_t motor_dm_param_cmd(bsp_fdcan_id_t bus,
                                  uint16_t id,
                                  uint8_t cmd,
                                  uint8_t rid,
                                  uint32_t value)
{
    uint8_t data[8];

    data[0] = (uint8_t)id;
    data[1] = (uint8_t)(id >> 8);
    data[2] = cmd;
    data[3] = rid;
    data[4] = (uint8_t)value;
    data[5] = (uint8_t)(value >> 8);
    data[6] = (uint8_t)(value >> 16);
    data[7] = (uint8_t)(value >> 24);

    return send_8_timeout(bus, MOTOR_DM_PARAM_BROADCAST_ID, data, 10U);
}

uint32_t motor_dm_param_seq(void)
{
    return motor_dm_param_seq_counter;
}

uint8_t motor_dm_param_get(uint16_t id, uint8_t rid, uint32_t *value)
{
    if ((motor_dm_param_id != (uint32_t)id) || (motor_dm_param_rid != rid)) {
        return 0U;
    }

    if (value != NULL) {
        *value = motor_dm_param_value;
    }

    return 1U;
}

motor_status_t motor_rs_cmd(bsp_fdcan_id_t bus, uint16_t id, RsMotorCmdKind kind)
{
    uint8_t data[8];
    MitMotorStatus status;

    status = rs_motor_cmd(data, kind);
    if (status != MitMotorOk) {
        return mit_status(status);
    }

    return send_8(bus, id, data);
}

motor_status_t motor_chassis(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    return motor_dji_send(MOTOR_CHASSIS_BUS, MOTOR_CHASSIS_TX_ID, m1, m2, m3, m4);
}

motor_status_t motor_chassis_steer(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    return motor_dji_send(MOTOR_CHASSIS_STEER_BUS, MOTOR_CHASSIS_STEER_TX_ID, m1, m2, m3, m4);
}

motor_status_t motor_gimbal(int16_t yaw, int16_t pitch, int16_t trigger, int16_t rev)
{
    uint32_t primask;

    primask = critical_enter();
    motor_gimbal_cache.yaw = yaw;
    motor_gimbal_cache.pitch = pitch;
    motor_gimbal_cache.trigger = trigger;
    motor_gimbal_cache.rev = rev;
    critical_exit(primask);

    return motor_dji_send(MOTOR_GIMBAL_BUS, MOTOR_GIMBAL_TX_ID, yaw, pitch, trigger, rev);
}

motor_status_t motor_gimbal_yaw_pitch(int16_t yaw, int16_t pitch, int16_t rev)
{
    uint32_t primask;
    motor_gimbal_group_t group;

    primask = critical_enter();
    motor_gimbal_cache.yaw = yaw;
    motor_gimbal_cache.pitch = pitch;
    motor_gimbal_cache.rev = rev;
    group = motor_gimbal_cache;
    critical_exit(primask);

    return motor_dji_send(MOTOR_GIMBAL_BUS,
                          MOTOR_GIMBAL_TX_ID,
                          group.yaw,
                          group.pitch,
                          group.trigger,
                          group.rev);
}

motor_status_t motor_gimbal_trigger(int16_t trigger)
{
    uint32_t primask;
    motor_gimbal_group_t group;

    primask = critical_enter();
    motor_gimbal_cache.trigger = trigger;
    group = motor_gimbal_cache;
    critical_exit(primask);

    return motor_dji_send(MOTOR_GIMBAL_BUS,
                          MOTOR_GIMBAL_TX_ID,
                          group.yaw,
                          group.pitch,
                          group.trigger,
                          group.rev);
}

motor_status_t motor_fric(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    return motor_dji_send(MOTOR_FRIC_BUS, MOTOR_FRIC_TX_ID, m1, m2, m3, m4);
}

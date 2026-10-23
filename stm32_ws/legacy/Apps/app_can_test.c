/**
 ******************************************************************************
 * @file    app_can_test.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-10-19
 * @brief   CAN1 DMH6215 台架测试实现。
 *
 * @note    扫描 ID 1..8：先发送使能和 MIT 模式命令，再周期发送零力矩 MIT 帧。
 * @note    收到任意标准帧后记录 ID/数据，并对反馈中的电机 ID 做小幅正弦位置测试。
 * @note    本 App 只用于台架，不进入整车运行链路。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "app_can_test.h"

#include "app_can_test_config.h"
#include "bsp_fdcan.h"
#include "bsp_timebase.h"
#include "main.h"
#include "motor_dm.h"
#include "cmsis_os.h"

#include <math.h>
#include <string.h>

#define APP_CAN_TEST_BUS                          BSP_FDCAN_ID_FDCAN1
#define APP_CAN_TEST_ID_FIRST                     1U
#define APP_CAN_TEST_ID_LAST                      8U

#if (APP_CAN_TEST_CONSERVATIVE != 0) && (APP_CAN_TEST_TARGET_ID == 0)
#error "APP_CAN_TEST_CONSERVATIVE requires APP_CAN_TEST_TARGET_ID != 0"
#endif

volatile uint32_t app_can_test_rx_count;
volatile uint32_t app_can_test_tx_count;
volatile uint32_t app_can_test_last_id;
volatile uint8_t app_can_test_last_data[8];
volatile uint8_t app_can_test_seen[256];
volatile uint32_t app_can_test_phase;
volatile uint32_t app_can_test_drive_id;
volatile uint32_t app_can_test_detected_id;
volatile uint16_t app_can_test_motor_mask;
volatile float app_can_test_cmd_pos;
volatile float app_can_test_center_pos;
volatile float app_can_test_cmd_vel;
volatile uint8_t app_can_test_last_tx[8];
volatile uint32_t app_can_test_tec;
volatile uint32_t app_can_test_rec;
volatile uint32_t app_can_test_psr;
volatile uint32_t app_can_test_param_id;
volatile uint32_t app_can_test_param_rid;
volatile uint32_t app_can_test_param_value;
volatile uint8_t app_can_test_fb_data[9][8];
volatile uint32_t app_can_test_fb_count[9];
volatile float app_can_test_sweep_start_pos[5];
volatile float app_can_test_sweep_end_pos[5];
volatile float app_can_test_sweep_delta[5];
volatile float app_can_test_sweep_max_vel[5];
volatile uint32_t app_can_test_sweep_abort[5];
volatile uint32_t app_can_test_sweep_done;

volatile uint32_t app_can_test_safe_state;
volatile uint32_t app_can_test_safe_reason;
volatile uint32_t app_can_test_safe_fb_count;
volatile uint8_t app_can_test_safe_fb_data[8];
volatile float app_can_test_safe_pos;
volatile float app_can_test_safe_vel;
volatile float app_can_test_safe_start_pos;
volatile float app_can_test_safe_moved_rad;
volatile float app_can_test_safe_cmd_vel;
volatile uint32_t app_can_test_safe_fb_age_ms;
volatile uint32_t app_can_test_safe_elapsed_ms;

static osThreadId_t app_can_test_thread;
static FDCAN_HandleTypeDef *app_can_test_hfdcan;

static const osThreadAttr_t app_can_test_attr = {
    .name = "app_can_test",
    .stack_size = APP_CAN_TEST_STACK_SIZE,
    .priority = (osPriority_t)APP_CAN_TEST_PRIORITY,
};

static void app_can_test_rx_cb(FDCAN_HandleTypeDef *hfdcan,
                               const FDCAN_RxHeaderTypeDef *header,
                               const uint8_t *data,
                               void *user)
{
    uint8_t i;

    (void)hfdcan;
    (void)user;

    if ((header == NULL) || (data == NULL) || (header->IdType != FDCAN_STANDARD_ID)) {
        return;
    }

    app_can_test_last_id = header->Identifier;
    {
        uint32_t motor_id = (uint32_t)(data[0] & 0x0FU);
        uint8_t param_frame = 0U;

        /* 参数应答不是正常反馈：按数据特征识别，避免污染速度/位置快照。 */
        if ((header->DataLength >= 8U) &&
            (data[1] == 0x00U) &&
            ((data[2] == 0x33U) || (data[2] == 0x55U) || (data[2] == 0xAAU))) {
            param_frame = 1U;
            if ((header->Identifier >= 0x51U) &&
                (header->Identifier <= 0x58U) &&
                (data[0] == (uint8_t)(header->Identifier - 0x50U))) {
                app_can_test_param_id =
                    (uint32_t)data[0] | ((uint32_t)data[1] << 8);
                app_can_test_param_rid = data[3];
                app_can_test_param_value = (uint32_t)data[4] |
                                           ((uint32_t)data[5] << 8) |
                                           ((uint32_t)data[6] << 16) |
                                           ((uint32_t)data[7] << 24);
            }
        }

        if ((motor_id >= APP_CAN_TEST_ID_FIRST) &&
            (motor_id <= APP_CAN_TEST_ID_LAST)) {
            app_can_test_motor_mask |= (uint16_t)(1U << motor_id);
            if (app_can_test_detected_id == 0U) {
                app_can_test_detected_id = motor_id;
            }
            /* 参数应答不是正常位置/速度反馈，不能写入反馈快照。 */
            if (param_frame == 0U) {
                for (i = 0U; i < 8U; i++) {
                    app_can_test_fb_data[motor_id][i] =
                        (i < (uint8_t)header->DataLength) ? data[i] : 0U;
                }
                app_can_test_fb_count[motor_id]++;
            }
        }
    }
    for (i = 0U; i < 8U; i++) {
        app_can_test_last_data[i] = (i < (uint8_t)header->DataLength) ? data[i] : 0U;
    }
    if (header->Identifier < 2048U) {
        app_can_test_seen[header->Identifier >> 3] |=
            (uint8_t)(1U << (header->Identifier & 7U));
    }

#if APP_CAN_TEST_CONSERVATIVE != 0
    /* 只跟踪目标电机的反馈，避免总线上的其它帧刷新看门狗。 */
    if ((uint32_t)(data[0] & 0x0FU) == (uint32_t)APP_CAN_TEST_TARGET_ID) {
        for (i = 0U; i < 8U; i++) {
            app_can_test_safe_fb_data[i] =
                (i < (uint8_t)header->DataLength) ? data[i] : 0U;
        }
        app_can_test_safe_fb_count++;
    }
#endif

    app_can_test_rx_count++;
}

static void app_can_test_send_cmd(uint16_t id, DmMotorCmdKind kind)
{
    uint8_t data[8];

    if (dm_motor_cmd(data, kind) != MitMotorOk) {
        return;
    }
    if (bsp_fdcan_send_std(app_can_test_hfdcan, id, data, 8U) == HAL_OK) {
        app_can_test_tx_count++;
    }
}

static void app_can_test_send_mit(uint16_t id, float pos, float vel, float kp, float kd, float tor)
{
    MitMotorCmd cmd;
    uint8_t data[8];

    cmd.pos = pos;
    cmd.vel = vel;
    cmd.kp = kp;
    cmd.kd = kd;
    cmd.tor = tor;

    if (dm_motor_mit(data, DmMotor6215, &cmd) != MitMotorOk) {
        return;
    }
    (void)memcpy((void *)app_can_test_last_tx, data, sizeof(data));
    if (bsp_fdcan_send_std(app_can_test_hfdcan, id, data, 8U) == HAL_OK) {
        app_can_test_tx_count++;
    }
}

static void app_can_test_send_speed(uint16_t id, float vel)
{
    uint8_t data[8];

    if (dm_motor_speed(data, vel) != MitMotorOk) {
        return;
    }
    (void)memcpy((void *)app_can_test_last_tx, data, sizeof(data));
    if (bsp_fdcan_send_std(app_can_test_hfdcan,
                          (uint16_t)(id + MIT_MOTOR_MODE_SPEED),
                          data,
                          8U) == HAL_OK) {
        app_can_test_tx_count++;
    }
}

static void app_can_test_send_refresh(uint16_t id)
{
    uint8_t data[8];

    data[0] = (uint8_t)id;
    data[1] = (uint8_t)(id >> 8);
    data[2] = 0xCCU;
    data[3] = 0x00U;
    data[4] = 0x00U;
    data[5] = 0x00U;
    data[6] = 0x00U;
    data[7] = 0x00U;

    if (bsp_fdcan_send_std(app_can_test_hfdcan, 0x7FFU, data, 8U) == HAL_OK) {
        app_can_test_tx_count++;
    }
}

#if (APP_CAN_TEST_ALL_FORWARD != 0) || (APP_CAN_TEST_SWEEP != 0)
static void app_can_test_send_param(uint16_t id, uint8_t cmd, uint8_t rid, uint32_t value)
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

    if (bsp_fdcan_send_std(app_can_test_hfdcan, 0x7FFU, data, 8U) == HAL_OK) {
        app_can_test_tx_count++;
    }
}

static void app_can_test_read_param(uint16_t id, uint8_t rid)
{
    app_can_test_param_id = 0xFFFFFFFFU;
    app_can_test_param_rid = 0xFFU;
    app_can_test_param_value = 0U;
    app_can_test_send_param(id, 0x33U, rid, 0U);
}

static uint8_t app_can_test_param_matches(uint16_t id, uint8_t rid, uint32_t value)
{
    return ((app_can_test_param_id == (uint32_t)id) &&
            (app_can_test_param_rid == (uint32_t)rid) &&
            (app_can_test_param_value == value)) ? 1U : 0U;
}
#endif

#if APP_CAN_TEST_ALL_FORWARD != 0
/**
 * @brief 四轮原生速度模式直行测试。
 *
 * ID 1..4 对应右后/前左/左后/前右；麦克纳姆对角轮同向：
 * 右后、前左取负，左后、前右取正。限时结束后零速、MIT 零力矩并失能。
 */
static void app_can_test_all_forward(void)
{
    static const uint16_t motors[4] = {1U, 2U, 3U, 4U};
    /*
     * 左侧轮（FL/RL）正转对应车身前进，右侧轮（RR/FR）反转对应前进。
     * 2026-10-20 悬空观察确认：原 {-,-,+,+} 会使前左、前右向后。
     */
    static const float signs[4] = {-1.0F, 1.0F, 1.0F, -1.0F};
    const float target = APP_CAN_TEST_ALL_VEL_RAD_S;
    const uint64_t run_us = (uint64_t)APP_CAN_TEST_ALL_RUN_MS * 1000ULL;
    const uint64_t ramp_us = (uint64_t)APP_CAN_TEST_ALL_RAMP_MS * 1000ULL;
    uint64_t start_us;
    uint64_t last_rx_us;
    uint64_t last_enable_us;
    uint32_t last_rx;
    uint32_t i;
    uint32_t round;
    uint8_t mode_ok = 1U;
    MitMotorFb feedback;

    /* 先对所有模式偏移失能，再统一设置 CTRL_MODE=3（速度模式）。 */
    for (i = 0U; i < 4U; i++) {
        app_can_test_send_speed(motors[i], 0.0F);
        app_can_test_send_mit(motors[i], 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
        app_can_test_send_cmd(motors[i], DmMotorDisable);
        app_can_test_send_cmd((uint16_t)(motors[i] + MIT_MOTOR_MODE_POS),
                              DmMotorDisable);
        app_can_test_send_cmd((uint16_t)(motors[i] + MIT_MOTOR_MODE_SPEED),
                              DmMotorDisable);
        app_can_test_send_cmd((uint16_t)(motors[i] + 0x300U),
                              DmMotorDisable);
    }
    osDelay(200U);

    for (i = 0U; i < 4U; i++) {
        uint64_t mode_wait_us;
        uint8_t ok = 0U;

        app_can_test_send_param(motors[i], 0x55U, 0x0AU, 3U);
        osDelay(50U);
        app_can_test_read_param(motors[i], 0x0AU);
        mode_wait_us = bsp_timebase_now_us();
        while ((bsp_timebase_now_us() - mode_wait_us) < 200000ULL) {
            if (app_can_test_param_matches(motors[i], 0x0AU, 3U) != 0U) {
                ok = 1U;
                break;
            }
            osDelay(5U);
        }
        if (ok != 0U) {
            app_can_test_send_param(motors[i], 0xAAU, 0x01U, 0U);
        } else {
            mode_ok = 0U;
        }
        osDelay(100U);
    }

    if (mode_ok != 0U) {
        /* 使能帧重复两轮，避免某台电机漏收一次后一直不转。 */
        for (round = 0U; round < 2U; round++) {
            for (i = 0U; i < 4U; i++) {
                app_can_test_send_cmd((uint16_t)(motors[i] + MIT_MOTOR_MODE_SPEED),
                                      DmMotorEnable);
                app_can_test_send_speed(motors[i], 0.0F);
            }
            osDelay(300U);
        }
    }

    app_can_test_phase = 1U;
    start_us = bsp_timebase_now_us();
    last_rx = app_can_test_rx_count;
    last_rx_us = start_us;
    last_enable_us = start_us;

    for (;;) {
        uint64_t now_us = bsp_timebase_now_us();
        uint64_t elapsed_us = now_us - start_us;
        float mag;

        if (mode_ok == 0U) {
            break;
        }

        if (elapsed_us >= run_us) {
            break;
        }

        if (app_can_test_rx_count != last_rx) {
            last_rx = app_can_test_rx_count;
            last_rx_us = now_us;
        } else if ((now_us - last_rx_us) >= 500000ULL) {
            break;
        }

        /* 运行中周期性重发使能，防止个别电机漏帧后保持失能。 */
        if ((now_us - last_enable_us) >= 500000ULL) {
            for (i = 0U; i < 4U; i++) {
                app_can_test_send_cmd((uint16_t)(motors[i] + MIT_MOTOR_MODE_SPEED),
                                      DmMotorEnable);
            }
            last_enable_us = now_us;
        }

        if (elapsed_us < ramp_us) {
            mag = target * ((float)elapsed_us / (float)ramp_us);
        } else {
            mag = target;
        }

        for (i = 0U; i < 4U; i++) {
            float wheel_target = (((APP_CAN_TEST_ALL_MASK >> i) & 1U) != 0U)
                                     ? (signs[i] * mag)
                                     : 0.0F;
            app_can_test_send_speed(motors[i], wheel_target);
        }
        app_can_test_cmd_vel = mag;

        if (dm_motor_decode(&feedback, DmMotor6215,
                            (const uint8_t *)app_can_test_last_data,
                            8U) == MitMotorOk) {
            if (fabsf(feedback.vel) > APP_CAN_TEST_ALL_ABORT_VEL_RAD_S) {
                break;
            }
        }

        osDelay(20U);
    }

    app_can_test_phase = 2U;
    for (round = 0U; round < 10U; round++) {
        for (i = 0U; i < 4U; i++) {
            app_can_test_send_speed(motors[i], 0.0F);
            app_can_test_send_mit(motors[i], 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
            app_can_test_send_cmd(motors[i], DmMotorDisable);
            app_can_test_send_cmd((uint16_t)(motors[i] + MIT_MOTOR_MODE_POS),
                                  DmMotorDisable);
            app_can_test_send_cmd((uint16_t)(motors[i] + MIT_MOTOR_MODE_SPEED),
                                  DmMotorDisable);
            app_can_test_send_cmd((uint16_t)(motors[i] + 0x300U),
                                  DmMotorDisable);
        }
        osDelay(50U);
    }

    for (;;) {
        for (i = 0U; i < 4U; i++) {
            app_can_test_send_speed(motors[i], 0.0F);
            app_can_test_send_mit(motors[i], 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
            app_can_test_send_cmd(motors[i], DmMotorDisable);
            app_can_test_send_cmd((uint16_t)(motors[i] + MIT_MOTOR_MODE_POS),
                                  DmMotorDisable);
            app_can_test_send_cmd((uint16_t)(motors[i] + MIT_MOTOR_MODE_SPEED),
                                  DmMotorDisable);
            app_can_test_send_cmd((uint16_t)(motors[i] + 0x300U),
                                  DmMotorDisable);
        }
        osDelay(100U);
    }
}
#endif

#if APP_CAN_TEST_SWEEP != 0
/**
 * @brief 逐台低速速度测试，记录每台电机的起止位置、最大反馈速度和中止原因。
 *
 * 中止原因：0=正常完成，1=模式设置/读回失败，2=反馈丢失，3=反馈速度超限。
 * 每台电机只运行 APP_CAN_TEST_SWEEP_RUN_MS，结束后立即零速并失能。
 */
static void app_can_test_sweep(void)
{
    static const uint16_t motors[4] = {1U, 2U, 3U, 4U};
    const float target = APP_CAN_TEST_SWEEP_VEL_RAD_S;
    const uint64_t run_us = (uint64_t)APP_CAN_TEST_SWEEP_RUN_MS * 1000ULL;
    const uint64_t ramp_us = (uint64_t)APP_CAN_TEST_SWEEP_RAMP_MS * 1000ULL;
    uint32_t i;
    uint32_t round;

    app_can_test_sweep_done = 0U;
    for (i = 0U; i < 5U; i++) {
        app_can_test_sweep_start_pos[i] = 0.0F;
        app_can_test_sweep_end_pos[i] = 0.0F;
        app_can_test_sweep_delta[i] = 0.0F;
        app_can_test_sweep_max_vel[i] = 0.0F;
        app_can_test_sweep_abort[i] = 0U;
    }

    /* 全部失能，覆盖所有模式偏移。 */
    for (i = 0U; i < 4U; i++) {
        for (round = 0U; round < 3U; round++) {
            app_can_test_send_speed(motors[i], 0.0F);
            app_can_test_send_mit(motors[i], 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
            app_can_test_send_cmd(motors[i], DmMotorDisable);
            app_can_test_send_cmd((uint16_t)(motors[i] + MIT_MOTOR_MODE_POS),
                                  DmMotorDisable);
            app_can_test_send_cmd((uint16_t)(motors[i] + MIT_MOTOR_MODE_SPEED),
                                  DmMotorDisable);
            app_can_test_send_cmd((uint16_t)(motors[i] + 0x300U),
                                  DmMotorDisable);
        }
        app_can_test_send_refresh(motors[i]);
    }
    osDelay(300U);

    /* 统一写速度模式 3，读回确认后保存。 */
    for (i = 0U; i < 4U; i++) {
        uint64_t mode_wait_us;
        uint8_t ok = 0U;

        app_can_test_send_param(motors[i], 0x55U, 0x0AU, 3U);
        osDelay(50U);
        app_can_test_read_param(motors[i], 0x0AU);
        mode_wait_us = bsp_timebase_now_us();
        while ((bsp_timebase_now_us() - mode_wait_us) < 200000ULL) {
            if (app_can_test_param_matches(motors[i], 0x0AU, 3U) != 0U) {
                ok = 1U;
                break;
            }
            osDelay(5U);
        }
        if (ok != 0U) {
            app_can_test_send_param(motors[i], 0xAAU, 0x01U, 0U);
        } else {
            app_can_test_sweep_abort[motors[i]] = 1U;
        }
        osDelay(100U);
    }

    for (i = 0U; i < 4U; i++) {
        const uint16_t m = motors[i];
        uint64_t start_us;
        uint64_t last_fb_us;
        uint32_t last_fb_count;
        float start_pos = 0.0F;
        float end_pos = 0.0F;
        float max_vel = 0.0F;
        uint32_t abort_reason = app_can_test_sweep_abort[m];
        uint8_t have_fb = 0U;
        MitMotorFb fb;

        /* 其它电机保持零速失能。 */
        for (round = 0U; round < 4U; round++) {
            uint16_t other = motors[round];
            app_can_test_send_speed(other, 0.0F);
            app_can_test_send_mit(other, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
            app_can_test_send_cmd(other, DmMotorDisable);
            app_can_test_send_cmd((uint16_t)(other + MIT_MOTOR_MODE_SPEED),
                                  DmMotorDisable);
        }
        osDelay(200U);

        if (abort_reason == 0U) {
            app_can_test_send_cmd((uint16_t)(m + MIT_MOTOR_MODE_SPEED),
                                  DmMotorEnable);
            app_can_test_send_speed(m, 0.0F);
            osDelay(300U);

            if (dm_motor_decode(&fb, DmMotor6215,
                                (const uint8_t *)app_can_test_fb_data[m],
                                8U) == MitMotorOk) {
                start_pos = fb.pos;
                have_fb = 1U;
            } else {
                abort_reason = 2U;
            }
        }

        if (have_fb != 0U) {
            start_us = bsp_timebase_now_us();
            last_fb_us = start_us;
            last_fb_count = app_can_test_fb_count[m];

            for (;;) {
                uint64_t now_us = bsp_timebase_now_us();
                uint64_t elapsed_us = now_us - start_us;
                float mag;

                if (elapsed_us >= run_us) {
                    break;
                }

                if (app_can_test_fb_count[m] != last_fb_count) {
                    last_fb_count = app_can_test_fb_count[m];
                    last_fb_us = now_us;
                } else if ((now_us - last_fb_us) >= 300000ULL) {
                    abort_reason = 2U;
                    break;
                }

                if (elapsed_us < ramp_us) {
                    mag = target * ((float)elapsed_us / (float)ramp_us);
                } else {
                    mag = target;
                }
                app_can_test_send_speed(m, mag);
                app_can_test_cmd_vel = mag;

                if (dm_motor_decode(&fb, DmMotor6215,
                                    (const uint8_t *)app_can_test_fb_data[m],
                                    8U) == MitMotorOk) {
                    float vel = fabsf(fb.vel);
                    if (vel > max_vel) {
                        max_vel = vel;
                    }
                    if (vel > APP_CAN_TEST_SWEEP_ABORT_VEL_RAD_S) {
                        abort_reason = 3U;
                        break;
                    }
                }
                osDelay(20U);
            }

            if (dm_motor_decode(&fb, DmMotor6215,
                                (const uint8_t *)app_can_test_fb_data[m],
                                8U) == MitMotorOk) {
                end_pos = fb.pos;
            }
        }

        /* 停止当前电机，所有模式偏移都发失能。 */
        for (round = 0U; round < 5U; round++) {
            app_can_test_send_speed(m, 0.0F);
            app_can_test_send_mit(m, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
            app_can_test_send_cmd(m, DmMotorDisable);
            app_can_test_send_cmd((uint16_t)(m + MIT_MOTOR_MODE_POS),
                                  DmMotorDisable);
            app_can_test_send_cmd((uint16_t)(m + MIT_MOTOR_MODE_SPEED),
                                  DmMotorDisable);
            app_can_test_send_cmd((uint16_t)(m + 0x300U), DmMotorDisable);
            osDelay(50U);
        }

        app_can_test_sweep_start_pos[m] = start_pos;
        app_can_test_sweep_end_pos[m] = end_pos;
        app_can_test_sweep_delta[m] = end_pos - start_pos;
        app_can_test_sweep_max_vel[m] = max_vel;
        app_can_test_sweep_abort[m] = abort_reason;
        app_can_test_sweep_done |= (1U << m);
        osDelay(500U);
    }

    /* 全部完成，持续安全停止并刷新反馈。 */
    for (;;) {
        for (i = 0U; i < 4U; i++) {
            const uint16_t m = motors[i];
            app_can_test_send_speed(m, 0.0F);
            app_can_test_send_mit(m, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
            app_can_test_send_cmd(m, DmMotorDisable);
            app_can_test_send_cmd((uint16_t)(m + MIT_MOTOR_MODE_POS),
                                  DmMotorDisable);
            app_can_test_send_cmd((uint16_t)(m + MIT_MOTOR_MODE_SPEED),
                                  DmMotorDisable);
            app_can_test_send_cmd((uint16_t)(m + 0x300U), DmMotorDisable);
            app_can_test_send_refresh(m);
        }
        osDelay(100U);
    }
}
#endif

#if APP_CAN_TEST_CONSERVATIVE != 0
static void app_can_test_safe_stop(uint16_t motor, uint32_t reason)
{
    uint32_t round;

    app_can_test_safe_state = APP_CAN_TEST_SAFE_STATE_STOP;
    app_can_test_safe_reason = reason;
    app_can_test_safe_cmd_vel = 0.0F;

    /* 零速、MIT 零力矩、失能各重复若干轮，确保速度环和输出都被清掉。 */
    for (round = 0U; round < APP_CAN_TEST_SAFE_STOP_ROUNDS; round++) {
        app_can_test_send_speed(motor, 0.0F);
        app_can_test_send_mit(motor, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
        app_can_test_send_cmd(motor, DmMotorDisable);
        osDelay(50U);
    }

    app_can_test_safe_state = APP_CAN_TEST_SAFE_STATE_DONE;
}
#endif

static void app_can_test_entry(void *argument)
{
#if (APP_CAN_TEST_STOP_ONLY == 0) && (APP_CAN_TEST_CONSERVATIVE == 0)
    uint64_t start_us;
    uint64_t now_us;
    uint64_t last_enable_us;
    uint8_t id;
#endif

    (void)argument;

    if (app_can_test_hfdcan == NULL) {
        for (;;) {
            osDelay(1000U);
        }
    }

#if APP_CAN_TEST_SWEEP != 0
    app_can_test_sweep();
    for (;;) {
        osDelay(1000U);
    }
#elif APP_CAN_TEST_ALL_FORWARD != 0
    app_can_test_all_forward();
    for (;;) {
        osDelay(1000U);
    }
#elif APP_CAN_TEST_STOP_ONLY != 0
    /*
     * 达妙速度模式在失去通信后会保持最后速度设定。
     * 只对目标电机连续发送零速、MIT 零力矩和失能，避免无效 ID 占满 TX FIFO。
     */
    {
        /*
         * 停止专用构建不区分目标 ID：未知总线状态时对 1..8 持续执行
         * 零速、MIT 零力矩和失能，避免电机重新上电后保留旧速度设定。
         */
        for (;;) {
            for (uint8_t motor = APP_CAN_TEST_ID_FIRST;
                 motor <= APP_CAN_TEST_ID_LAST;
                 motor++) {
                app_can_test_send_speed(motor, 0.0F);
                app_can_test_send_mit(motor, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
                /*
                 * 失能命令必须发到当前控制模式对应的 ID 偏移：
                 * MIT=ID、位置速度=ID+0x100、速度=ID+0x200、位置力控=ID+0x300。
                 * 未知电机模式时四个偏移全部发送，确保都能停下来。
                 */
                app_can_test_send_cmd(motor, DmMotorDisable);
                app_can_test_send_cmd((uint16_t)(motor + MIT_MOTOR_MODE_POS),
                                      DmMotorDisable);
                app_can_test_send_cmd((uint16_t)(motor + MIT_MOTOR_MODE_SPEED),
                                      DmMotorDisable);
                app_can_test_send_cmd((uint16_t)(motor + 0x300U),
                                      DmMotorDisable);
                app_can_test_send_refresh(motor);
                osDelay(2U);
            }
            osDelay(50U);
        }
    }
    for (;;) {
        osDelay(1000U);
    }
#elif APP_CAN_TEST_SCAN_ONLY != 0
    /*
     * 只扫描：周期性发送使能/零 MIT/零速度，记录哪些电机 ID 有反馈。
     * 不发送任何非零速度或位置指令。
     */
    {
        uint64_t scan_now_us;
        uint64_t scan_last_enable_us = 0U;

        for (;;) {
            uint8_t motor;

            scan_now_us = bsp_timebase_now_us();
            if ((scan_now_us - scan_last_enable_us) >= 500000ULL) {
                for (motor = APP_CAN_TEST_ID_FIRST;
                     motor <= APP_CAN_TEST_ID_LAST;
                     motor++) {
                    app_can_test_send_cmd(motor, DmMotorEnable);
                    app_can_test_send_cmd(motor, DmMotorSpeedMode);
                }
                scan_last_enable_us = scan_now_us;
            }

            for (motor = APP_CAN_TEST_ID_FIRST;
                 motor <= APP_CAN_TEST_ID_LAST;
                 motor++) {
                app_can_test_send_mit(motor, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
                app_can_test_send_speed(motor, 0.0F);
            }
            osDelay(20U);
        }
    }
#elif APP_CAN_TEST_CONSERVATIVE != 0
    /*
     * 保守驱动：只操作指定 ID，MIT 模式下 Kp=0、t_ff=0，
     * 力矩只由 Kd*(v_des - v_actual) 产生，最大约 Kd*|v_des|。
     * 总时长、反馈中断、位置变化和堵转都有兜底，结束后零速+失能。
     */
    {
        const uint16_t motor = (uint16_t)APP_CAN_TEST_TARGET_ID;
        const uint64_t settle_us = (uint64_t)APP_CAN_TEST_SAFE_SETTLE_MS * 1000ULL;
        const uint64_t ramp_us = (uint64_t)APP_CAN_TEST_SAFE_RAMP_MS * 1000ULL;
        const uint64_t hold_us = (uint64_t)APP_CAN_TEST_SAFE_HOLD_MS * 1000ULL;
        const uint64_t total_us = (ramp_us * 2ULL) + hold_us;
        uint64_t now_us;
        uint64_t state_us;
        uint64_t run_us = 0U;
        uint64_t last_fb_us = 0U;
        uint32_t last_fb_count = 0U;
        float start_pos = 0.0F;
        uint8_t have_feedback = 0U;
        MitMotorFb feedback;

        (void)memset(&feedback, 0, sizeof(feedback));

        app_can_test_drive_id = (uint32_t)motor;
        app_can_test_safe_state = APP_CAN_TEST_SAFE_STATE_IDLE;
        app_can_test_safe_reason = APP_CAN_TEST_SAFE_REASON_NONE;

        /* 先清掉上一次固件可能残留的速度设定，再重新使能。 */
        app_can_test_send_speed(motor, 0.0F);
        app_can_test_send_mit(motor, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
        app_can_test_send_cmd(motor, DmMotorDisable);
        osDelay(100U);

        app_can_test_send_cmd(motor, DmMotorEnable);
        osDelay(100U);
#if APP_CAN_TEST_SAFE_NATIVE_SPEED != 0
        app_can_test_send_cmd(motor, DmMotorSpeedMode);
#else
        app_can_test_send_cmd(motor, DmMotorMitMode);
#endif
        osDelay(100U);

        state_us = bsp_timebase_now_us();
        app_can_test_safe_state = APP_CAN_TEST_SAFE_STATE_WAIT_FEEDBACK;

        for (;;) {
            now_us = bsp_timebase_now_us();

            app_can_test_tec = app_can_test_hfdcan->Instance->ECR & 0xFFU;
            app_can_test_rec = (app_can_test_hfdcan->Instance->ECR >> 8) & 0x7FU;
            app_can_test_psr = app_can_test_hfdcan->Instance->PSR;

            if (app_can_test_safe_state == APP_CAN_TEST_SAFE_STATE_DONE) {
                osDelay(1000U);
                continue;
            }

            if (app_can_test_safe_fb_count != last_fb_count) {
                last_fb_count = app_can_test_safe_fb_count;
                last_fb_us = now_us;

                if (dm_motor_decode(&feedback,
                                    DmMotor6215,
                                    (const uint8_t *)app_can_test_safe_fb_data,
                                    8U) == MitMotorOk) {
                    have_feedback = 1U;
                    app_can_test_safe_pos = feedback.pos;
                    app_can_test_safe_vel = feedback.vel;
                }
            }

            if (have_feedback != 0U) {
                app_can_test_safe_fb_age_ms =
                    (uint32_t)((now_us - last_fb_us) / 1000ULL);
            } else {
                app_can_test_safe_fb_age_ms =
                    (uint32_t)((now_us - state_us) / 1000ULL);
            }

            if (app_can_test_safe_state == APP_CAN_TEST_SAFE_STATE_WAIT_FEEDBACK) {
                app_can_test_safe_elapsed_ms =
                    (uint32_t)((now_us - state_us) / 1000ULL);
                app_can_test_send_speed(motor, 0.0F);
                app_can_test_send_mit(motor, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);

                if ((have_feedback != 0U) &&
                    (app_can_test_safe_fb_age_ms <= APP_CAN_TEST_SAFE_FB_TIMEOUT_MS)) {
                    start_pos = feedback.pos;
                    app_can_test_safe_start_pos = start_pos;
                    app_can_test_safe_moved_rad = 0.0F;
                    app_can_test_safe_state = APP_CAN_TEST_SAFE_STATE_SETTLE;
                    state_us = now_us;
                } else if (app_can_test_safe_elapsed_ms >=
                           APP_CAN_TEST_SAFE_START_FB_TIMEOUT_MS) {
                    app_can_test_safe_stop(motor, APP_CAN_TEST_SAFE_REASON_NO_FEEDBACK);
                }
            } else if (app_can_test_safe_state == APP_CAN_TEST_SAFE_STATE_SETTLE) {
                app_can_test_safe_elapsed_ms =
                    (uint32_t)((now_us - state_us) / 1000ULL);
                app_can_test_send_speed(motor, 0.0F);
                app_can_test_send_mit(motor, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);

                if (app_can_test_safe_fb_age_ms > APP_CAN_TEST_SAFE_FB_TIMEOUT_MS) {
                    app_can_test_safe_stop(motor, APP_CAN_TEST_SAFE_REASON_FEEDBACK_LOST);
                } else if ((now_us - state_us) >= settle_us) {
                    run_us = now_us;
                    app_can_test_safe_state = APP_CAN_TEST_SAFE_STATE_RUN;
                }
            } else if (app_can_test_safe_state == APP_CAN_TEST_SAFE_STATE_RUN) {
                uint64_t elapsed_us = now_us - run_us;
                float cmd_vel = 0.0F;

                app_can_test_safe_elapsed_ms = (uint32_t)(elapsed_us / 1000ULL);

                if (elapsed_us >= total_us) {
                    app_can_test_safe_stop(motor, APP_CAN_TEST_SAFE_REASON_RUN_DONE);
                } else if (app_can_test_safe_elapsed_ms >=
                           APP_CAN_TEST_SAFE_TIMEOUT_MS) {
                    app_can_test_safe_stop(motor, APP_CAN_TEST_SAFE_REASON_TIMEOUT);
                } else if (app_can_test_safe_fb_age_ms >
                           APP_CAN_TEST_SAFE_FB_TIMEOUT_MS) {
                    app_can_test_safe_stop(motor, APP_CAN_TEST_SAFE_REASON_FEEDBACK_LOST);
                } else {
                    if (elapsed_us < ramp_us) {
                        cmd_vel = APP_CAN_TEST_SAFE_VEL_RAD_S *
                                  ((float)elapsed_us / (float)ramp_us);
                    } else if (elapsed_us < (ramp_us + hold_us)) {
                        cmd_vel = APP_CAN_TEST_SAFE_VEL_RAD_S;
                    } else {
                        cmd_vel = APP_CAN_TEST_SAFE_VEL_RAD_S *
                                  ((float)(total_us - elapsed_us) / (float)ramp_us);
                    }

                    app_can_test_safe_cmd_vel = cmd_vel;
#if APP_CAN_TEST_SAFE_NATIVE_SPEED != 0
                    app_can_test_send_speed(motor, cmd_vel);
#else
                    app_can_test_send_mit(motor, 0.0F, cmd_vel, 0.0F,
                                          APP_CAN_TEST_SAFE_KD, 0.0F);
#endif

                    app_can_test_safe_moved_rad =
                        fabsf(feedback.pos - start_pos);
                    if (app_can_test_safe_moved_rad >
                        APP_CAN_TEST_SAFE_POS_LIMIT_RAD) {
                        app_can_test_safe_stop(motor,
                                               APP_CAN_TEST_SAFE_REASON_POS_LIMIT);
                    } else if ((app_can_test_safe_elapsed_ms >=
                                APP_CAN_TEST_SAFE_STALL_MS) &&
                               (fabsf(cmd_vel) > 0.1F) &&
                               (fabsf(feedback.vel) <
                                APP_CAN_TEST_SAFE_STALL_VEL_RAD_S)) {
                        app_can_test_safe_stop(motor,
                                               APP_CAN_TEST_SAFE_REASON_STALL);
                    }
                }
            } else {
                /* 尚未进入状态机或状态异常：保持零速、零力矩，等待状态切换。 */
                app_can_test_send_speed(motor, 0.0F);
                app_can_test_send_mit(motor, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
            }

            osDelay(APP_CAN_TEST_PERIOD_MS);
        }
    }
#else
    start_us = bsp_timebase_now_us();
    last_enable_us = 0U;

#if APP_CAN_TEST_TARGET_ID != 0
    /*
     * 已知 ID 时跳过扫描：先使能、延时、切速度模式，再直接给速度。
     * 避免扫描阶段的 MIT 零帧和模式反复切换把电机打断。
     */
    app_can_test_send_cmd((uint16_t)APP_CAN_TEST_TARGET_ID, DmMotorEnable);
    osDelay(100U);
    app_can_test_send_cmd((uint16_t)APP_CAN_TEST_TARGET_ID, DmMotorSpeedMode);
    osDelay(100U);
    app_can_test_send_speed((uint16_t)APP_CAN_TEST_TARGET_ID, 0.0F);
    osDelay(50U);
    app_can_test_drive_id = (uint32_t)APP_CAN_TEST_TARGET_ID;
    app_can_test_phase = 1U;
    start_us = bsp_timebase_now_us();
#endif

    for (;;) {
        now_us = bsp_timebase_now_us();

        app_can_test_tec = app_can_test_hfdcan->Instance->ECR & 0xFFU;
        app_can_test_rec = (app_can_test_hfdcan->Instance->ECR >> 8) & 0x7FU;
        app_can_test_psr = app_can_test_hfdcan->Instance->PSR;

        if (app_can_test_phase == 0U) {
            /* 使能命令周期性重发，避免电机上电较慢时漏掉。 */
            if ((now_us - last_enable_us) >= 500000ULL) {
                for (id = APP_CAN_TEST_ID_FIRST; id <= APP_CAN_TEST_ID_LAST; id++) {
                    app_can_test_send_cmd(id, DmMotorEnable);
                    app_can_test_send_cmd(id, DmMotorSpeedMode);
                }
                last_enable_us = now_us;
            }

            /* 所有 ID 周期发送零力矩 MIT 帧，等待反馈。 */
            for (id = APP_CAN_TEST_ID_FIRST; id <= APP_CAN_TEST_ID_LAST; id++) {
                app_can_test_send_mit(id, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
            }

            if ((now_us - start_us) >= ((uint64_t)APP_CAN_TEST_SCAN_MS * 1000ULL)) {
                if (app_can_test_rx_count > 0U) {
                    MitMotorFb feedback;
                    uint32_t selected = (uint32_t)APP_CAN_TEST_TARGET_ID;

                    if (selected == 0U) {
                        selected = app_can_test_detected_id;
                    }
                    if ((selected < APP_CAN_TEST_ID_FIRST) ||
                        (selected > APP_CAN_TEST_ID_LAST)) {
                        selected = 1U;
                    }
                    app_can_test_drive_id = selected;

                    /* 以当前反馈位置为中心，避免上电位置未知造成大阶跃。 */
                    if (dm_motor_decode(&feedback,
                                        DmMotor6215,
                                        (const uint8_t *)app_can_test_last_data,
                                        8U) == MitMotorOk) {
                        app_can_test_center_pos = feedback.pos;
                    } else {
                        app_can_test_center_pos = 0.0F;
                    }

                    app_can_test_phase = 1U;
                    start_us = now_us;
                } else {
                    /* 没反馈就继续扫描，TEC/REC/PSR 供 GDB 判断接线和波特率。 */
                    start_us = now_us;
                }
            }
        } else if (app_can_test_phase == 1U) {
            /*
             * 持续速度运行：Kp=0 不做位置保持，仅用 Kd 对速度误差出力。
             * 电机悬空，速度目标较小，保持到断电或烧回 bring-up 固件。
             */
            app_can_test_cmd_pos = 0.0F;
            app_can_test_cmd_vel = APP_CAN_TEST_VEL_RAD_S;
            app_can_test_send_speed((uint16_t)app_can_test_drive_id,
                                    APP_CAN_TEST_VEL_RAD_S);

#if APP_CAN_TEST_RUN_MS > 0
            if ((now_us - start_us) >= ((uint64_t)APP_CAN_TEST_RUN_MS * 1000ULL)) {
                app_can_test_phase = 2U;
            }
#endif
        } else {
            app_can_test_cmd_pos = 0.0F;
            app_can_test_cmd_vel = 0.0F;
            app_can_test_send_speed((uint16_t)app_can_test_drive_id, 0.0F);
            app_can_test_send_mit((uint16_t)app_can_test_drive_id,
                                  0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
        }

        osDelay(APP_CAN_TEST_PERIOD_MS);
    }
#endif
}

void app_can_test_init(void)
{
    app_can_test_rx_count = 0U;
    app_can_test_tx_count = 0U;
    app_can_test_last_id = 0U;
    (void)memset((void *)app_can_test_last_data, 0, sizeof(app_can_test_last_data));
    (void)memset((void *)app_can_test_seen, 0, sizeof(app_can_test_seen));
    app_can_test_phase = 0U;
    app_can_test_drive_id = 0U;
    app_can_test_detected_id = 0U;
    app_can_test_motor_mask = 0U;
    app_can_test_cmd_pos = 0.0F;
    app_can_test_center_pos = 0.0F;
    app_can_test_cmd_vel = 0.0F;
    (void)memset((void *)app_can_test_last_tx, 0, sizeof(app_can_test_last_tx));
    app_can_test_tec = 0U;
    app_can_test_rec = 0U;
    app_can_test_psr = 0U;

    app_can_test_safe_state = APP_CAN_TEST_SAFE_STATE_IDLE;
    app_can_test_safe_reason = APP_CAN_TEST_SAFE_REASON_NONE;
    app_can_test_safe_fb_count = 0U;
    (void)memset((void *)app_can_test_safe_fb_data, 0, sizeof(app_can_test_safe_fb_data));
    app_can_test_safe_pos = 0.0F;
    app_can_test_safe_vel = 0.0F;
    app_can_test_safe_start_pos = 0.0F;
    app_can_test_safe_moved_rad = 0.0F;
    app_can_test_safe_cmd_vel = 0.0F;
    app_can_test_safe_fb_age_ms = 0U;
    app_can_test_safe_elapsed_ms = 0U;

    app_can_test_hfdcan = bsp_fdcan_handle(APP_CAN_TEST_BUS);
    if (app_can_test_hfdcan == NULL) {
        return;
    }

    (void)bsp_fdcan_rx_bind(app_can_test_hfdcan,
                            FDCAN_RX_FIFO0,
                            app_can_test_rx_cb,
                            NULL);

    app_can_test_thread = osThreadNew(app_can_test_entry, NULL, &app_can_test_attr);
    if (app_can_test_thread == NULL) {
        return;
    }
}

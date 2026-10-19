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

        if ((motor_id >= APP_CAN_TEST_ID_FIRST) &&
            (motor_id <= APP_CAN_TEST_ID_LAST)) {
            app_can_test_motor_mask |= (uint16_t)(1U << motor_id);
            if (app_can_test_detected_id == 0U) {
                app_can_test_detected_id = motor_id;
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

static void app_can_test_entry(void *argument)
{
#if APP_CAN_TEST_STOP_ONLY == 0
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

#if APP_CAN_TEST_STOP_ONLY != 0
    /*
     * 达妙速度模式在失去通信后会保持最后速度设定。
     * 只对目标电机连续发送零速、MIT 零力矩和失能，避免无效 ID 占满 TX FIFO。
     */
    {
        uint8_t motor = (APP_CAN_TEST_TARGET_ID != 0U)
                            ? (uint8_t)APP_CAN_TEST_TARGET_ID
                            : 1U;

        for (uint8_t round = 0U; round < 10U; round++) {
            app_can_test_send_speed(motor, 0.0F);
            app_can_test_send_mit(motor, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
            app_can_test_send_cmd(motor, DmMotorDisable);
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

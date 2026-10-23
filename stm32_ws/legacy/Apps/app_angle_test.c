/**
 ******************************************************************************
 * @file    app_angle_test.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-10-20
 * @brief   单电机角度环 PID 台架测试实现。
 *
 * @note    只使用一台 DMH6215，速度模式输出角度 PID 结果，测试目标阶跃
 *          0 -> +0.3 -> 0 -> -0.3 -> 0 rad。测试结束零速 + 失能。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "app_angle_test.h"

#include "angle_pid.h"
#include "bsp_timebase.h"
#include "main.h"
#include "motor.h"
#include "robot_board.h"

#include "cmsis_os.h"
#include "cmsis_os2.h"

#include <math.h>
#include <stddef.h>

/** @brief 测试电机基础 ID。 */
#ifndef APP_ANGLE_TEST_MOTOR_ID
#define APP_ANGLE_TEST_MOTOR_ID 1U
#endif

/** @brief 测试电机反馈 ID。 */
#define APP_ANGLE_TEST_FB_ID (0x50U + APP_ANGLE_TEST_MOTOR_ID)

/** @brief 任务周期，单位 ms。 */
#define APP_ANGLE_TEST_PERIOD_MS 2U

/** @brief 每个阶跃的保持时间，单位 ms。 */
#define APP_ANGLE_TEST_HOLD_MS 1200U

/** @brief 总运行时间上限，单位 ms。 */
#define APP_ANGLE_TEST_TIMEOUT_MS 8000U

/** @brief 反馈丢失阈值，单位 ms。 */
#define APP_ANGLE_TEST_FB_TIMEOUT_MS 20U

/** @brief 速度输出限幅，rad/s。 */
#define APP_ANGLE_TEST_MAX_OUT_RAD_S 1.0F

/** @brief 位置测试范围，rad。 */
#define APP_ANGLE_TEST_POS_LIMIT_RAD 1.0F

/** @brief 角度 PID 参数。 */
#define APP_ANGLE_TEST_KP 2.0F
#define APP_ANGLE_TEST_KI 0.5F
#define APP_ANGLE_TEST_KD 0.0F
#define APP_ANGLE_TEST_MAX_IOUT 0.5F
#define APP_ANGLE_TEST_DEADBAND 0.01F

volatile uint32_t app_angle_test_state;
volatile uint32_t app_angle_test_fault;
volatile float app_angle_test_target;
volatile float app_angle_test_actual;
volatile float app_angle_test_err;
volatile float app_angle_test_out;
volatile uint32_t app_angle_test_fb_count;

static osThreadId_t app_angle_test_thread;

static const osThreadAttr_t app_angle_test_attr = {
    .name = "app_angle_test",
    .stack_size = 1024U,
    .priority = osPriorityNormal,
};

static void app_angle_test_stop_sequence(void)
{
    uint32_t round;

    app_angle_test_state = APP_ANGLE_TEST_STATE_STOP;
    app_angle_test_out = 0.0F;
    for (round = 0U; round < 10U; round++) {
        (void)motor_dm_speed_stop(MOTOR_CHASSIS_BUS, APP_ANGLE_TEST_MOTOR_ID);
        osDelay(20U);
    }
    app_angle_test_state = APP_ANGLE_TEST_STATE_DONE;
}

static void app_angle_test_entry(void *argument)
{
    static const float targets[5] = {0.0F, 0.3F, 0.0F, -0.3F, 0.0F};
    angle_pid_cfg_t cfg;
    angle_pid_t pid;
    motor_mit_fb_t fb;
    uint32_t phase = 0U;
    uint64_t start_us;
    uint64_t phase_us;
    uint64_t last_fb_us;
    uint32_t round;

    (void)argument;

    app_angle_test_state = APP_ANGLE_TEST_STATE_IDLE;
    app_angle_test_fault = APP_ANGLE_TEST_FAULT_NONE;
    app_angle_test_target = 0.0F;
    app_angle_test_actual = 0.0F;
    app_angle_test_err = 0.0F;
    app_angle_test_out = 0.0F;
    app_angle_test_fb_count = 0U;

    cfg.kp = APP_ANGLE_TEST_KP;
    cfg.ki = APP_ANGLE_TEST_KI;
    cfg.kd = APP_ANGLE_TEST_KD;
    cfg.max_out = APP_ANGLE_TEST_MAX_OUT_RAD_S;
    cfg.max_iout = APP_ANGLE_TEST_MAX_IOUT;
    cfg.deadband = APP_ANGLE_TEST_DEADBAND;
    cfg.freeze_err = 0.0F;
    cfg.freeze_d = 0.0F;
    cfg.active_err = 0.5F;
    cfg.leak = 1.0F;
    if (angle_pid_init(&pid, &cfg) != PID_CORE_OK) {
        app_angle_test_fault = APP_ANGLE_TEST_FAULT_NO_FEEDBACK;
        app_angle_test_stop_sequence();
        for (;;) {
            osDelay(1000U);
        }
    }

    /* 等电机上电自检完成，再开始使能序列。 */
    osDelay(2000U);

    /* 先失能，等待，再使能。 */
    for (round = 0U; round < 5U; round++) {
        (void)motor_dm_speed_stop(MOTOR_CHASSIS_BUS, APP_ANGLE_TEST_MOTOR_ID);
        osDelay(20U);
    }
    osDelay(500U);
    (void)motor_dm_speed(MOTOR_CHASSIS_BUS, APP_ANGLE_TEST_MOTOR_ID, 0.0F);
    (void)motor_dm_speed_cmd(MOTOR_CHASSIS_BUS, APP_ANGLE_TEST_MOTOR_ID, DmMotorEnable);
    osDelay(500U);

    app_angle_test_state = APP_ANGLE_TEST_STATE_WAIT_FEEDBACK;
    start_us = bsp_timebase_now_us();
    while ((bsp_timebase_now_us() - start_us) < 2000000ULL) {
        if ((motor_mit_get(MOTOR_CHASSIS_BUS, APP_ANGLE_TEST_FB_ID, &fb) == MotorOk) &&
            ((uint32_t)(HAL_GetTick() - fb.last_time) <= APP_ANGLE_TEST_FB_TIMEOUT_MS)) {
            break;
        }
        osDelay(5U);
    }
    if ((motor_mit_get(MOTOR_CHASSIS_BUS, APP_ANGLE_TEST_FB_ID, &fb) != MotorOk) ||
        ((uint32_t)(HAL_GetTick() - fb.last_time) > APP_ANGLE_TEST_FB_TIMEOUT_MS)) {
        app_angle_test_fault = APP_ANGLE_TEST_FAULT_NO_FEEDBACK;
        app_angle_test_stop_sequence();
        for (;;) {
            (void)motor_dm_speed_stop(MOTOR_CHASSIS_BUS, APP_ANGLE_TEST_MOTOR_ID);
            osDelay(100U);
        }
    }

    app_angle_test_state = APP_ANGLE_TEST_STATE_RUN;
    angle_pid_set_target(&pid, targets[0]);
    start_us = bsp_timebase_now_us();
    phase_us = start_us;
    last_fb_us = start_us;

    while ((bsp_timebase_now_us() - start_us) <
           ((uint64_t)APP_ANGLE_TEST_TIMEOUT_MS * 1000ULL)) {
        uint64_t now_us = bsp_timebase_now_us();
        float out;

        if ((now_us - phase_us) >=
            ((uint64_t)APP_ANGLE_TEST_HOLD_MS * 1000ULL)) {
            phase++;
            if (phase >= 5U) {
                break;
            }
            angle_pid_set_target(&pid, targets[phase]);
            phase_us = now_us;
        }

        if (motor_mit_get(MOTOR_CHASSIS_BUS, APP_ANGLE_TEST_FB_ID, &fb) != MotorOk) {
            if ((now_us - last_fb_us) > ((uint64_t)APP_ANGLE_TEST_FB_TIMEOUT_MS * 1000ULL)) {
                app_angle_test_fault = APP_ANGLE_TEST_FAULT_FEEDBACK_LOST;
                break;
            }
        } else {
            if ((uint32_t)(HAL_GetTick() - fb.last_time) >
                APP_ANGLE_TEST_FB_TIMEOUT_MS) {
                app_angle_test_fault = APP_ANGLE_TEST_FAULT_FEEDBACK_LOST;
                break;
            }
            last_fb_us = now_us;
            app_angle_test_fb_count++;
            app_angle_test_actual = fb.fb.pos;

            if (fabsf(fb.fb.pos) > APP_ANGLE_TEST_POS_LIMIT_RAD) {
                app_angle_test_fault = APP_ANGLE_TEST_FAULT_RANGE;
                break;
            }

            app_angle_test_target = angle_pid_get_target(&pid);
            app_angle_test_err =
                angle_pid_wrap(app_angle_test_target - app_angle_test_actual);
            out = angle_pid_calc(&pid, app_angle_test_actual);
            if (out > APP_ANGLE_TEST_MAX_OUT_RAD_S) {
                out = APP_ANGLE_TEST_MAX_OUT_RAD_S;
            } else if (out < -APP_ANGLE_TEST_MAX_OUT_RAD_S) {
                out = -APP_ANGLE_TEST_MAX_OUT_RAD_S;
            }
            app_angle_test_out = out;
            (void)motor_dm_speed(MOTOR_CHASSIS_BUS, APP_ANGLE_TEST_MOTOR_ID, out);
        }

        osDelay(APP_ANGLE_TEST_PERIOD_MS);
    }

    if ((bsp_timebase_now_us() - start_us) >=
        ((uint64_t)APP_ANGLE_TEST_TIMEOUT_MS * 1000ULL)) {
        app_angle_test_fault = APP_ANGLE_TEST_FAULT_TIMEOUT;
    }

    app_angle_test_stop_sequence();
    for (;;) {
        (void)motor_dm_speed(MOTOR_CHASSIS_BUS, APP_ANGLE_TEST_MOTOR_ID, 0.0F);
        (void)motor_dm_speed_cmd(MOTOR_CHASSIS_BUS, APP_ANGLE_TEST_MOTOR_ID,
                                 DmMotorDisable);
        osDelay(100U);
    }
}

void app_angle_test_init(void)
{
    app_angle_test_thread = osThreadNew(app_angle_test_entry, NULL,
                                        &app_angle_test_attr);
}

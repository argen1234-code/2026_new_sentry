/**
 ******************************************************************************
 * @file    app_board_test.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   光板板载资源测试应用层实现。
 *
 * @note    本文件只串联板载 Flash、IMU、蜂鸣器、WS2812 和 USB CDC 日志，
 *          不启动或访问任何电机控制链路。
 * @note    Flash 测试只擦写专用 4KB 测试扇区一次，避免周期性磨损。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "app_board_test.h"

#include "app_board_test_config.h"
#include "bsp_cdc.h"
#include "bsp_dts.h"
#include "bsp_timebase.h"
#include "buzzer.h"
#include "communication_vofa.h"
#include "debug.h"
#include "flash.h"
#include "imu.h"
#include "main.h"
#include "observer.h"
#include "ws2812.h"

#include <math.h>
#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 这里放置测试任务的局部常量和编译期约束，避免把板测语义扩散到 Devices 层。
 * ========================================================================== */

/** @brief BoardTest 需要全部通过的板载资源集合，不把 CDC 打开状态算作硬件失败。 */
#define APP_BOARD_TEST_REQUIRED_MASK \
    (APP_BOARD_TEST_BIT_FLASH | APP_BOARD_TEST_BIT_IMU | APP_BOARD_TEST_BIT_WS2812 | APP_BOARD_TEST_BIT_BUZZER)

/** @brief 毫秒转换为 CMSIS-RTOS2 tick，向上取整避免短周期被截断。 */
#define APP_BOARD_TEST_MS_TO_TICKS(ms) \
    ((uint32_t)((((uint32_t)(ms) * osKernelGetTickFreq()) + 999U) / 1000U))

#if ((APP_BOARD_TEST_FLASH_ADDR % ROBOT_FLASH_DEVICE_ERASE_SIZE) != 0U)
#error "APP_BOARD_TEST_FLASH_ADDR must align to ROBOT_FLASH_DEVICE_ERASE_SIZE."
#endif

#if (APP_BOARD_TEST_FLASH_PATTERN_LEN > ROBOT_FLASH_DEVICE_PAGE_SIZE)
#error "APP_BOARD_TEST_FLASH_PATTERN_LEN must fit in one Flash page."
#endif

#if (APP_BOARD_TEST_FLASH_LEN < ROBOT_FLASH_DEVICE_ERASE_SIZE)
#error "APP_BOARD_TEST_FLASH_LEN must cover at least one erase sector."
#endif

/** @brief BoardTest VOFA JustFloat 通道数量。 */
#define APP_BOARD_TEST_VOFA_CHANNELS 20U

/** @brief BoardTest VOFA JustFloat 单帧字节数。 */
#define APP_BOARD_TEST_VOFA_FRAME_BYTES ((APP_BOARD_TEST_VOFA_CHANNELS * 4U) + 4U)

#if (APP_BOARD_TEST_VOFA_FRAME_BYTES > COMM_VOFA_TX_SIZE)
#error "APP_BOARD_TEST_VOFA_CHANNELS exceeds COMM_VOFA_TX_SIZE."
#endif

/**
 * @brief 蜂鸣器提示序列。
 */
typedef struct
{
    uint32_t phase_ms;  /**< 当前鸣叫或静音相位已持续时间。 */
    uint32_t gap_ms;    /**< 失败重复提示剩余静音时间。 */
    uint8_t left;       /**< 当前提示组剩余鸣叫次数。 */
    uint8_t total;      /**< 当前提示组总鸣叫次数。 */
    uint8_t active;     /**< 非 0 表示提示序列正在运行。 */
    uint8_t sounding;   /**< 非 0 表示蜂鸣器当前打开。 */
} AppBoardTone;

/* ==========================================================================
 * 任务对象与状态缓存
 * --------------------------------------------------------------------------
 * 所有运行时状态只保存在本文件内，对外通过快照接口读取，避免测试任务泄漏全局对象。
 * ========================================================================== */

/** @brief BoardTest 线程控制块。 */
static osThreadId_t app_board_thread;

/** @brief BoardTest 线程属性。 */
static const osThreadAttr_t app_board_attr = {
    .name = "app_board",
    .stack_size = APP_BOARD_TEST_STACK_SIZE,
    .priority = (osPriority_t)APP_BOARD_TEST_PRIORITY,
};

/** @brief BoardTest 状态快照缓存。 */
static AppBoardTestState app_board_state;

/** @brief BoardTest 诊断快照缓存。 */
static AppBoardTestDiag app_board_diag;

/** @brief 蜂鸣器提示状态。 */
static AppBoardTone app_board_tone;

/** @brief Flash 写入测试模式。 */
static uint8_t app_board_flash_tx[APP_BOARD_TEST_FLASH_PATTERN_LEN];

/** @brief Flash 读回测试模式。 */
static uint8_t app_board_flash_rx[APP_BOARD_TEST_FLASH_PATTERN_LEN];

/** @brief VOFA JustFloat 波形通道缓存。 */
static float app_board_vofa[APP_BOARD_TEST_VOFA_CHANNELS];

/** @brief 光板 IMU 姿态解算 Observer 上下文。 */
static Observer app_board_observer;

/** @brief 最近一次光板 IMU 姿态解算状态。 */
static ObserverState app_board_observer_state;

/** @brief 最近一次 Observer 输入时间戳，单位 us。 */
static uint32_t app_board_observer_last_us;

/** @brief ready 后用于评估 yaw 漂移的基准连续角。 */
static float app_board_yaw_ref;

/** @brief ready 后用于评估 yaw 漂移的基准时间，单位 ms。 */
static uint32_t app_board_yaw_ref_ms;

/** @brief 非 0 表示姿态解算已完成初始化。 */
static uint8_t app_board_observer_ready;

/** @brief 非 0 表示 yaw 漂移基准已捕获。 */
static uint8_t app_board_yaw_ref_valid;

/* ==========================================================================
 * 临界区与时间工具
 * --------------------------------------------------------------------------
 * 快照复制使用短临界区；板测逻辑本身不在临界区内执行 IO 或耗时操作。
 * ========================================================================== */

/**
 * @brief 进入短临界区。
 *
 * @return 进入前 PRIMASK。
 */
static uint32_t app_board_enter(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

/**
 * @brief 退出短临界区。
 *
 * @param primask 进入前 PRIMASK。
 */
static void app_board_exit(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief 获取当前毫秒时间戳。
 *
 * @return 当前时间低 32 位，单位 ms。
 */
static uint32_t app_board_now_ms(void)
{
    return (uint32_t)(bsp_timebase_now_us() / 1000ULL);
}

/* ==========================================================================
 * 状态位与诊断更新
 * --------------------------------------------------------------------------
 * 失败状态锁存，避免现场只看到后续偶发恢复而漏掉启动阶段硬件异常。
 * ========================================================================== */

/**
 * @brief 标记一个测试项通过。
 *
 * @param bit 测试项状态位。
 */
static void app_board_pass(uint32_t bit)
{
    if ((app_board_state.fail_mask & bit) == 0UL) {
        app_board_state.pass_mask |= bit;
    }
}

/**
 * @brief 标记一个测试项失败。
 *
 * @param bit 测试项状态位。
 */
static void app_board_fail(uint32_t bit)
{
    app_board_state.fail_mask |= bit;
    app_board_state.pass_mask &= ~bit;
}

/**
 * @brief 判断所有必要板载资源是否已经通过测试。
 *
 * @return 全部通过返回 1，否则返回 0。
 */
static uint8_t app_board_all_pass(void)
{
    return (((app_board_state.pass_mask & APP_BOARD_TEST_REQUIRED_MASK) == APP_BOARD_TEST_REQUIRED_MASK) &&
            ((app_board_state.fail_mask & APP_BOARD_TEST_REQUIRED_MASK) == 0UL))
         ? 1U
         : 0U;
}

/**
 * @brief 获取第一个失败测试项对应的提示次数。
 *
 * @return 失败项编号，从 1 开始；无失败时返回 0。
 */
static uint8_t app_board_fail_code(void)
{
    uint8_t i;

    for (i = 0U; i < 8U; i++) {
        if ((app_board_state.fail_mask & (1UL << i)) != 0UL) {
            return (uint8_t)(i + 1U);
        }
    }

    return 0U;
}

/* ==========================================================================
 * 输出执行
 * --------------------------------------------------------------------------
 * WS2812 和蜂鸣器只表达板测结果，不参与任何控制链路安全钳位。
 * ========================================================================== */

/**
 * @brief 设置 WS2812 单灯颜色并立即发送。
 *
 * @param r 红色通道。
 * @param g 绿色通道。
 * @param b 蓝色通道。
 */
static void app_board_led(uint8_t r, uint8_t g, uint8_t b)
{
    Ws2812Status status;

    if ((app_board_state.fail_mask & APP_BOARD_TEST_BIT_WS2812) != 0UL) {
        return;
    }

    status = ws2812_set_rgb(0U, r, g, b);
    if (status == Ws2812Ok) {
        status = ws2812_show();
    }

    app_board_state.ws2812_status = (int32_t)status;
    if (status == Ws2812Ok) {
        app_board_pass(APP_BOARD_TEST_BIT_WS2812);
    } else if (status == Ws2812Busy) {
        app_board_diag.ws2812_busy_count++;
    } else {
        app_board_diag.ws2812_error_count++;
        app_board_fail(APP_BOARD_TEST_BIT_WS2812);
    }
}

/**
 * @brief 关闭蜂鸣器并记录错误。
 */
static void app_board_buzzer_off(void)
{
    HAL_StatusTypeDef status = buzzer_off();

    app_board_state.buzzer_status = (int32_t)status;
    if (status != HAL_OK) {
        app_board_diag.buzzer_error_count++;
        app_board_fail(APP_BOARD_TEST_BIT_BUZZER);
    }
    app_board_tone.sounding = 0U;
}

/**
 * @brief 打开蜂鸣器并记录错误。
 */
static void app_board_buzzer_on(void)
{
    HAL_StatusTypeDef status = buzzer_start_duty(APP_BOARD_TEST_BUZZER_FREQ_HZ, APP_BOARD_TEST_BUZZER_DUTY);

    app_board_state.buzzer_status = (int32_t)status;
    if (status != HAL_OK) {
        app_board_diag.buzzer_error_count++;
        app_board_fail(APP_BOARD_TEST_BIT_BUZZER);
        app_board_tone.sounding = 0U;
        return;
    }

    app_board_pass(APP_BOARD_TEST_BIT_BUZZER);
    app_board_tone.sounding = 1U;
}

/**
 * @brief 启动一组蜂鸣器提示音。
 *
 * @param count 提示鸣叫次数。
 */
static void app_board_tone_start(uint8_t count)
{
    if (count == 0U) {
        return;
    }

    (void)memset(&app_board_tone, 0, sizeof(app_board_tone));
    app_board_tone.left = count;
    app_board_tone.total = count;
    app_board_tone.active = 1U;
}

/**
 * @brief 推进蜂鸣器提示状态机。
 */
static void app_board_tone_step(void)
{
    uint8_t fail_code;

    if (app_board_tone.active == 0U) {
        fail_code = app_board_fail_code();
        if ((fail_code != 0U) && (app_board_tone.gap_ms == 0U)) {
            app_board_tone_start(fail_code);
        } else if (app_board_tone.gap_ms > APP_BOARD_TEST_PERIOD_MS) {
            app_board_tone.gap_ms -= APP_BOARD_TEST_PERIOD_MS;
        } else {
            app_board_tone.gap_ms = 0U;
        }
        return;
    }

    if (app_board_tone.left == 0U) {
        app_board_tone.active = 0U;
        app_board_tone.gap_ms = APP_BOARD_TEST_FAIL_GAP_MS;
        app_board_buzzer_off();
        return;
    }

    app_board_tone.phase_ms += APP_BOARD_TEST_PERIOD_MS;
    if (app_board_tone.phase_ms <= APP_BOARD_TEST_BEEP_ON_MS) {
        if (app_board_tone.sounding == 0U) {
            app_board_buzzer_on();
        }
        return;
    }

    app_board_buzzer_off();
    if (app_board_tone.phase_ms >= (APP_BOARD_TEST_BEEP_ON_MS + APP_BOARD_TEST_BEEP_OFF_MS)) {
        app_board_tone.phase_ms = 0U;
        app_board_tone.left--;
    }
}

/* ==========================================================================
 * 测试步骤
 * --------------------------------------------------------------------------
 * 各测试项只调用 Devices/Component 的公开 API，不访问 CubeMX 句柄或旧工程全局。
 * ========================================================================== */

/**
 * @brief 填充 Flash 写入测试模式。
 */
static void app_board_flash_pattern(void)
{
    uint16_t i;

    for (i = 0U; i < APP_BOARD_TEST_FLASH_PATTERN_LEN; i++) {
        app_board_flash_tx[i] = (uint8_t)(0xA5U ^ (uint8_t)(i * 13U));
    }
}

/**
 * @brief 执行一次 Flash 擦写读回测试。
 */
static void app_board_flash_test(void)
{
    flash_status_t status;

    if (app_board_state.flash_done != 0U) {
        return;
    }

    app_board_state.flash_done = 1U;
    app_board_flash_pattern();
    (void)memset(app_board_flash_rx, 0, sizeof(app_board_flash_rx));

    status = flash_init();
    if (status == FLASH_OK) {
        status = flash_erase(APP_BOARD_TEST_FLASH_ADDR, APP_BOARD_TEST_FLASH_LEN);
    }
    if (status == FLASH_OK) {
        status = flash_prog(APP_BOARD_TEST_FLASH_ADDR, app_board_flash_tx, APP_BOARD_TEST_FLASH_PATTERN_LEN);
    }
    if (status == FLASH_OK) {
        status = flash_read(APP_BOARD_TEST_FLASH_ADDR, app_board_flash_rx, APP_BOARD_TEST_FLASH_PATTERN_LEN);
    }
    if ((status == FLASH_OK) &&
        (memcmp(app_board_flash_tx, app_board_flash_rx, sizeof(app_board_flash_tx)) != 0)) {
        status = FLASH_ERR_IO;
    }

    app_board_state.flash_status = (int32_t)status;
    if (status == FLASH_OK) {
        app_board_pass(APP_BOARD_TEST_BIT_FLASH);
    } else {
        app_board_diag.flash_error_count++;
        app_board_fail(APP_BOARD_TEST_BIT_FLASH);
    }
}

/**
 * @brief 判断 IMU 单轴数据是否为有限值。
 *
 * @param value 待检查数值。
 * @return 有限值返回 1，否则返回 0。
 */
static uint8_t app_board_float_ok(float value)
{
    return isfinite(value) ? 1U : 0U;
}

/**
 * @brief 初始化光板 IMU 姿态解算器。
 */
static void app_board_observer_init(void)
{
    ObserverCfg cfg;
    ObserverStatus status;

    if (APP_BOARD_TEST_IMU_SOLVE_ENABLE == 0U) {
        return;
    }

    (void)memset(&cfg, 0, sizeof(cfg));
    cfg.mahony.kp = 1.0F;
    cfg.mahony.ki = 0.0F;
    cfg.mahony.dt = (float32_t)APP_BOARD_TEST_IMU_PERIOD_MS * 0.001F;
    cfg.gravity = OBSERVER_GRAVITY_DEFAULT;
    cfg.ready_time = APP_BOARD_TEST_IMU_READY_TIME;
    cfg.stable_gyro = APP_BOARD_TEST_IMU_STABLE_GYRO_TH;
    cfg.stable_accel = APP_BOARD_TEST_IMU_STABLE_ACCEL_TH;
    cfg.accel_lpf = OBSERVER_ACCEL_LPF_DEFAULT;
    cfg.accel_dead_xy = OBSERVER_ACCEL_DEAD_XY_DEFAULT;
    cfg.accel_dead_z = OBSERVER_ACCEL_DEAD_Z_DEFAULT;
    cfg.gyro_cali_samples = APP_BOARD_TEST_IMU_CALI_SAMPLES;
    cfg.cali_gyro_th = APP_BOARD_TEST_IMU_CALI_GYRO_TH;
    cfg.cali_accel_th = APP_BOARD_TEST_IMU_CALI_ACCEL_TH;
    cfg.kf_p = OBSERVER_KF_P_DEFAULT;
    cfg.kf_q = OBSERVER_KF_Q_DEFAULT;
    cfg.kf_r = OBSERVER_KF_R_DEFAULT;
    cfg.kf_dt = cfg.mahony.dt;
    cfg.ground_threshold = OBSERVER_GROUND_TH_DEFAULT;
    cfg.wheel_weight = OBSERVER_WHEEL_WEIGHT_DEFAULT;

    status = observer_init(&app_board_observer, &cfg);
    app_board_state.observer_status = (int32_t)status;
    app_board_observer_ready = (status == ObserverOk) ? 1U : 0U;
    if (status != ObserverOk) {
        app_board_diag.observer_error_count++;
    }
}

/**
 * @brief 用最新 IMU 采样推进姿态解算，并更新漂移观察量。
 *
 * @param sample 已校验为有限值的 IMU 采样。
 */
static void app_board_observer_step(const imu_sample_t *sample)
{
    ObserverInput input;
    ObserverStatus status;
    uint32_t now_us;
    uint32_t now_ms;
    uint32_t delta_us;
    uint32_t drift_ms;
    float32_t dt;

    if ((APP_BOARD_TEST_IMU_SOLVE_ENABLE == 0U) ||
        (app_board_observer_ready == 0U) ||
        (sample == NULL)) {
        return;
    }

    now_us = (uint32_t)bsp_timebase_now_us();
    delta_us = now_us - app_board_observer_last_us;
    app_board_observer_last_us = now_us;
    dt = (float32_t)delta_us * 0.000001F;
    if ((delta_us == 0U) || (dt <= 0.0F) || (dt > 0.1F) || (isfinite(dt) == 0)) {
        dt = (float32_t)APP_BOARD_TEST_IMU_PERIOD_MS * 0.001F;
    }

    (void)memset(&input, 0, sizeof(input));
    input.imu.gyro.x = sample->gyro[0];
    input.imu.gyro.y = sample->gyro[1];
    input.imu.gyro.z = sample->gyro[2];
    input.imu.accel.x = sample->accel[0];
    input.imu.accel.y = sample->accel[1];
    input.imu.accel.z = sample->accel[2];
    input.imu.temperature = sample->temperature;
    input.imu.dt = dt;
    input.imu.timestamp_us = now_us;
    input.imu.online = 1U;
    input.timestamp_us = now_us;

    status = observer_step(&app_board_observer, &input, &app_board_observer_state);
    app_board_state.observer_status = (int32_t)status;
    if (status != ObserverOk) {
        app_board_diag.observer_error_count++;
        return;
    }

    app_board_diag.observer_ok_count++;
    app_board_state.ins_valid = app_board_observer_state.ins.valid;
    app_board_state.ins_ready = app_board_observer_state.ins.ready;
    app_board_state.ins_stable = app_board_observer_state.ins.stable;
    app_board_state.euler[0] = app_board_observer_state.ins.euler.roll;
    app_board_state.euler[1] = app_board_observer_state.ins.euler.pitch;
    app_board_state.euler[2] = app_board_observer_state.ins.euler.yaw;
    app_board_state.yaw_total = app_board_observer_state.ins.yaw_total;
    app_board_state.gyro_bias[0] = app_board_observer_state.ins.gyro_bias.x;
    app_board_state.gyro_bias[1] = app_board_observer_state.ins.gyro_bias.y;
    app_board_state.gyro_bias[2] = app_board_observer_state.ins.gyro_bias.z;

    if (app_board_observer_state.ins.ready == 0U) {
        return;
    }

    now_ms = app_board_now_ms();
    if (app_board_yaw_ref_valid == 0U) {
        app_board_yaw_ref_valid = 1U;
        app_board_yaw_ref = app_board_observer_state.ins.yaw_total;
        app_board_yaw_ref_ms = now_ms;
    }

    app_board_state.yaw_drift = app_board_observer_state.ins.yaw_total - app_board_yaw_ref;
    drift_ms = now_ms - app_board_yaw_ref_ms;
    if (drift_ms > 0U) {
        app_board_state.yaw_drift_deg_min =
            app_board_state.yaw_drift * COMMON_RAD2DEG * 60000.0F / (float32_t)drift_ms;
    } else {
        app_board_state.yaw_drift_deg_min = 0.0F;
    }
}

/**
 * @brief 周期读取 IMU 并更新测试状态。
 */
static void app_board_imu_step(void)
{
    imu_sample_t sample;
    imu_status_t status;
    uint8_t ok;

    status = imu_read();
    if (status == IMU_OK) {
        status = imu_sample(&sample);
    }

    ok = (status == IMU_OK) ? 1U : 0U;
    if (ok != 0U) {
        ok &= app_board_float_ok(sample.temperature);
        ok &= app_board_float_ok(sample.accel[0]);
        ok &= app_board_float_ok(sample.accel[1]);
        ok &= app_board_float_ok(sample.accel[2]);
        ok &= app_board_float_ok(sample.gyro[0]);
        ok &= app_board_float_ok(sample.gyro[1]);
        ok &= app_board_float_ok(sample.gyro[2]);
    }

    app_board_state.imu_status = (int32_t)status;
    if (ok != 0U) {
        app_board_state.imu_temp = sample.temperature;
        app_board_state.accel[0] = sample.accel[0];
        app_board_state.accel[1] = sample.accel[1];
        app_board_state.accel[2] = sample.accel[2];
        app_board_state.gyro[0] = sample.gyro[0];
        app_board_state.gyro[1] = sample.gyro[1];
        app_board_state.gyro[2] = sample.gyro[2];
        app_board_observer_step(&sample);
        app_board_diag.imu_ok_count++;
        app_board_pass(APP_BOARD_TEST_BIT_IMU);
    } else {
        app_board_diag.imu_error_count++;
        app_board_fail(APP_BOARD_TEST_BIT_IMU);
    }
}

/**
 * @brief 周期读取 MCU 内部 DTS 温度。
 */
static void app_board_dts_step(void)
{
    DTS_HandleTypeDef *hdts;
    HAL_StatusTypeDef status = HAL_ERROR;
    int32_t raw_temp = 0;
    float temp;

    hdts = bsp_dts_handle(BSP_DTS_ID_DTS);
    if (hdts != NULL) {
        status = bsp_dts_start(hdts);
        if (status == HAL_OK) {
            status = bsp_dts_get_temperature(hdts, &raw_temp);
            (void)bsp_dts_stop(hdts);
        }
    }

    app_board_state.dts_status = (int32_t)status;
    temp = (float)raw_temp;
    if ((status == HAL_OK) && (isfinite(temp) != 0)) {
        app_board_state.mcu_temp = temp;
        app_board_diag.dts_ok_count++;
    } else {
        app_board_diag.dts_error_count++;
    }
}

/**
 * @brief 输出一帧 VOFA JustFloat 波形数据。
 *
 * 通道顺序固定，方便 VOFA 工程直接按序命名：
 * roll/pitch/yaw/yaw_total、gyro、bias、accel、IMU 温度、MCU 温度、yaw 漂移和状态位。
 */
static void app_board_vofa_step(void)
{
    CommunicationStatus status;

    if (APP_BOARD_TEST_VOFA_ENABLE == 0U) {
        return;
    }

    app_board_vofa[0] = app_board_state.euler[0] * COMMON_RAD2DEG;
    app_board_vofa[1] = app_board_state.euler[1] * COMMON_RAD2DEG;
    app_board_vofa[2] = app_board_state.euler[2] * COMMON_RAD2DEG;
    app_board_vofa[3] = app_board_state.yaw_total * COMMON_RAD2DEG;
    app_board_vofa[4] = app_board_state.gyro[0] * COMMON_RAD2DEG;
    app_board_vofa[5] = app_board_state.gyro[1] * COMMON_RAD2DEG;
    app_board_vofa[6] = app_board_state.gyro[2] * COMMON_RAD2DEG;
    app_board_vofa[7] = app_board_state.gyro_bias[0] * COMMON_RAD2DEG;
    app_board_vofa[8] = app_board_state.gyro_bias[1] * COMMON_RAD2DEG;
    app_board_vofa[9] = app_board_state.gyro_bias[2] * COMMON_RAD2DEG;
    app_board_vofa[10] = app_board_state.accel[0];
    app_board_vofa[11] = app_board_state.accel[1];
    app_board_vofa[12] = app_board_state.accel[2];
    app_board_vofa[13] = app_board_state.imu_temp;
    app_board_vofa[14] = app_board_state.mcu_temp;
    app_board_vofa[15] = app_board_state.yaw_drift_deg_min;
    app_board_vofa[16] = (float)app_board_state.pass_mask;
    app_board_vofa[17] = (float)app_board_state.fail_mask;
    app_board_vofa[18] = (float)app_board_state.observer_status;
    app_board_vofa[19] = (float)app_board_state.cdc_ready;

    status = comm_vofa_float(app_board_vofa, APP_BOARD_TEST_VOFA_CHANNELS);
    if (status == CommunicationOk) {
        app_board_diag.vofa_ok_count++;
    } else if (status == CommunicationBusy) {
        app_board_diag.vofa_busy_count++;
    } else {
        app_board_diag.vofa_error_count++;
    }
}

/**
 * @brief 输出一帧 CDC 文本日志。
 */
static void app_board_log(void)
{
    CommunicationStatus status;
    int32_t temp_centi;
    int32_t accel_x_milli;
    int32_t gyro_z_milli;
    int32_t roll_cdeg;
    int32_t pitch_cdeg;
    int32_t yaw_cdeg;
    int32_t bias_x_mdps;
    int32_t bias_y_mdps;
    int32_t bias_z_mdps;
    int32_t drift_cdeg_min;

    if (APP_BOARD_TEST_TEXT_LOG_ENABLE == 0U) {
        return;
    }

    temp_centi = (int32_t)(app_board_state.imu_temp * 100.0F);
    accel_x_milli = (int32_t)(app_board_state.accel[0] * 1000.0F);
    gyro_z_milli = (int32_t)(app_board_state.gyro[2] * 1000.0F);
    roll_cdeg = (int32_t)(app_board_state.euler[0] * COMMON_RAD2DEG * 100.0F);
    pitch_cdeg = (int32_t)(app_board_state.euler[1] * COMMON_RAD2DEG * 100.0F);
    yaw_cdeg = (int32_t)(app_board_state.euler[2] * COMMON_RAD2DEG * 100.0F);
    bias_x_mdps = (int32_t)(app_board_state.gyro_bias[0] * COMMON_RAD2DEG * 1000.0F);
    bias_y_mdps = (int32_t)(app_board_state.gyro_bias[1] * COMMON_RAD2DEG * 1000.0F);
    bias_z_mdps = (int32_t)(app_board_state.gyro_bias[2] * COMMON_RAD2DEG * 1000.0F);
    drift_cdeg_min = (int32_t)(app_board_state.yaw_drift_deg_min * 100.0F);

    status = comm_vofa_fire("[board] pass=0x%08lX fail=0x%08lX cdc=%u flash=%ld imu=%ld obs=%ld rdy=%u st=%u temp=%ld.%02ld ax=%ld gz=%ld rpy=%ld/%ld/%ld gb=%ld/%ld/%ld dpm=%ld\r\n",
                            (unsigned long)app_board_state.pass_mask,
                            (unsigned long)app_board_state.fail_mask,
                            (unsigned int)app_board_state.cdc_ready,
                            (long)app_board_state.flash_status,
                            (long)app_board_state.imu_status,
                            (long)app_board_state.observer_status,
                            (unsigned int)app_board_state.ins_ready,
                            (unsigned int)app_board_state.ins_stable,
                            (long)(temp_centi / 100),
                            (long)((temp_centi < 0) ? -(temp_centi % 100) : (temp_centi % 100)),
                            (long)accel_x_milli,
                            (long)gyro_z_milli,
                            (long)roll_cdeg,
                            (long)pitch_cdeg,
                            (long)yaw_cdeg,
                            (long)bias_x_mdps,
                            (long)bias_y_mdps,
                            (long)bias_z_mdps,
                            (long)drift_cdeg_min);

    if (status == CommunicationOk) {
        app_board_diag.log_ok_count++;
    } else if (status == CommunicationBusy) {
        app_board_diag.log_busy_count++;
    } else {
        app_board_diag.log_error_count++;
    }

    LOGI_TAG("BOARD",
             "pass=0x%08lX fail=0x%08lX cdc=%u flash=%ld imu=%ld obs=%ld ready=%u temp=%ld.%02ld",
             (unsigned long)app_board_state.pass_mask,
             (unsigned long)app_board_state.fail_mask,
             (unsigned int)app_board_state.cdc_ready,
             (long)app_board_state.flash_status,
             (long)app_board_state.imu_status,
             (long)app_board_state.observer_status,
             (unsigned int)app_board_state.ins_ready,
             (long)(temp_centi / 100),
             (long)((temp_centi < 0) ? -(temp_centi % 100) : (temp_centi % 100)));
}

/* ==========================================================================
 * 周期状态机
 * --------------------------------------------------------------------------
 * 周期里只做测试推进和状态输出；所有失败都记录诊断，不阻塞任务。
 * ========================================================================== */

/**
 * @brief 刷新 CDC ready 状态。
 */
/**
 * @brief 在线程上下文中初始化被测板载设备。
 *
 * USB CDC 由 CubeMX 默认任务启动。板载外设初始化放在线程里执行，避免在调度器
 * 启动前被 IMU/Flash 等 HAL_Delay 路径阻塞，导致 USB 设备栈没有机会初始化。
 */
static void app_board_device_init(void)
{
    HAL_StatusTypeDef buzzer_status;
    Ws2812Status ws_status;
    flash_status_t flash_status;
    imu_status_t imu_status;

    (void)debug_init();
    (void)comm_vofa_init_cdc();
    (void)comm_vofa_bind_log();

    buzzer_status = buzzer_init();
    app_board_state.buzzer_status = (int32_t)buzzer_status;
    if (buzzer_status == HAL_OK) {
        app_board_pass(APP_BOARD_TEST_BIT_BUZZER);
        (void)buzzer_off();
    } else {
        app_board_fail(APP_BOARD_TEST_BIT_BUZZER);
        app_board_diag.buzzer_error_count++;
    }

    ws_status = ws2812_init((uint16_t)APP_BOARD_TEST_WS2812_COUNT);
    app_board_state.ws2812_status = (int32_t)ws_status;
    if (ws_status == Ws2812Ok) {
        app_board_pass(APP_BOARD_TEST_BIT_WS2812);
    } else {
        app_board_fail(APP_BOARD_TEST_BIT_WS2812);
        app_board_diag.ws2812_error_count++;
    }

    flash_status = flash_init();
    app_board_state.flash_status = (int32_t)flash_status;
    if (flash_status != FLASH_OK) {
        app_board_state.flash_done = 1U;
        app_board_fail(APP_BOARD_TEST_BIT_FLASH);
        app_board_diag.flash_error_count++;
    }

    imu_status = imu_init(0U);
    app_board_state.imu_status = (int32_t)imu_status;
    if (imu_status != IMU_OK) {
        app_board_fail(APP_BOARD_TEST_BIT_IMU);
        app_board_diag.imu_error_count++;
    } else {
        app_board_observer_init();
    }
}

static void app_board_cdc_step(void)
{
    app_board_state.cdc_ready = bsp_cdc_is_ready();
    if (app_board_state.cdc_ready != 0U) {
        app_board_state.pass_mask |= APP_BOARD_TEST_BIT_CDC;
    } else {
        app_board_state.pass_mask &= ~APP_BOARD_TEST_BIT_CDC;
    }
}

/**
 * @brief 刷新 WS2812 状态显示。
 *
 * @param blink 非 0 表示当前处于闪烁亮相。
 */
static void app_board_led_step(uint8_t blink)
{
    if ((app_board_state.fail_mask & APP_BOARD_TEST_REQUIRED_MASK) != 0UL) {
        if (blink != 0U) {
            app_board_led(40U, 0U, 0U);
        } else {
            app_board_led(0U, 0U, 0U);
        }
    } else if (app_board_all_pass() != 0U) {
        if (blink != 0U) {
            app_board_led(0U, 30U, 0U);
        } else {
            app_board_led(0U, 8U, 0U);
        }
    } else {
        app_board_led(30U, 20U, 0U);
    }
}

/**
 * @brief BoardTest 任务入口。
 *
 * @param argument CMSIS-RTOS2 参数，未使用。
 */
static void app_board_entry(void *argument)
{
    uint32_t elapsed_imu = APP_BOARD_TEST_IMU_PERIOD_MS;
    uint32_t elapsed_log = APP_BOARD_TEST_LOG_PERIOD_MS;
    uint32_t elapsed_vofa = APP_BOARD_TEST_VOFA_PERIOD_MS;
    uint32_t elapsed_dts = APP_BOARD_TEST_DTS_PERIOD_MS;
    uint32_t elapsed_led = APP_BOARD_TEST_LED_PERIOD_MS;
    uint8_t blink = 0U;
    uint8_t pass_tone_done = 0U;
    uint32_t primask;

    (void)argument;

    osDelay(APP_BOARD_TEST_MS_TO_TICKS(500U));
    app_board_device_init();
    app_board_tone_start(1U);
    app_board_led(0U, 0U, 30U);
    app_board_dts_step();
    app_board_vofa_step();
    app_board_log();

    while (1) {
        app_board_cdc_step();
        app_board_flash_test();

        elapsed_imu += APP_BOARD_TEST_PERIOD_MS;
        if (elapsed_imu >= APP_BOARD_TEST_IMU_PERIOD_MS) {
            elapsed_imu = 0U;
            app_board_imu_step();
        }

        elapsed_dts += APP_BOARD_TEST_PERIOD_MS;
        if (elapsed_dts >= APP_BOARD_TEST_DTS_PERIOD_MS) {
            elapsed_dts = 0U;
            app_board_dts_step();
        }

        elapsed_vofa += APP_BOARD_TEST_PERIOD_MS;
        if (elapsed_vofa >= APP_BOARD_TEST_VOFA_PERIOD_MS) {
            elapsed_vofa = 0U;
            app_board_vofa_step();
        }

        elapsed_led += APP_BOARD_TEST_PERIOD_MS;
        if (elapsed_led >= APP_BOARD_TEST_LED_PERIOD_MS) {
            elapsed_led = 0U;
            blink = (blink == 0U) ? 1U : 0U;
            app_board_led_step(blink);
        }

        if ((pass_tone_done == 0U) && (app_board_all_pass() != 0U)) {
            pass_tone_done = 1U;
            app_board_tone_start(2U);
        }
        app_board_tone_step();

        elapsed_log += APP_BOARD_TEST_PERIOD_MS;
        if (elapsed_log >= APP_BOARD_TEST_LOG_PERIOD_MS) {
            elapsed_log = 0U;
            app_board_log();
        }

        primask = app_board_enter();
        app_board_state.loop_count++;
        app_board_state.last_tick_ms = app_board_now_ms();
        app_board_exit(primask);

        osDelay(APP_BOARD_TEST_MS_TO_TICKS(APP_BOARD_TEST_PERIOD_MS));
    }
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 初始化先绑定 Debug/VOFA，再逐项初始化板载设备；测试结果由后台任务持续刷新。
 * ========================================================================== */

uint8_t app_board_test_init(void)
{
    (void)memset(&app_board_state, 0, sizeof(app_board_state));
    (void)memset(&app_board_diag, 0, sizeof(app_board_diag));
    (void)memset(&app_board_tone, 0, sizeof(app_board_tone));
    (void)memset(&app_board_observer, 0, sizeof(app_board_observer));
    (void)memset(&app_board_observer_state, 0, sizeof(app_board_observer_state));
    (void)memset(app_board_vofa, 0, sizeof(app_board_vofa));
    app_board_observer_last_us = 0U;
    app_board_yaw_ref = 0.0F;
    app_board_yaw_ref_ms = 0U;
    app_board_observer_ready = 0U;
    app_board_yaw_ref_valid = 0U;

    app_board_thread = osThreadNew(app_board_entry, NULL, &app_board_attr);
    app_board_state.ready = (app_board_thread != NULL) ? 1U : 0U;
    return (app_board_thread != NULL) ? 1U : 0U;
}

uint8_t app_board_test_get_state(AppBoardTestState *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_board_enter();
    *out = app_board_state;
    app_board_exit(primask);
    return 1U;
}

uint8_t app_board_test_get_diag(AppBoardTestDiag *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_board_enter();
    *out = app_board_diag;
    app_board_exit(primask);
    return 1U;
}

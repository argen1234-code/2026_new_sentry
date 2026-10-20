/**
 ******************************************************************************
 * @file    app_calibration.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   Calibration 应用层实现。
 *
 * @note    Flash 记录采用新布局：magic、version、target_id、record_mask、
 *          payload_len、crc 和固定 payload；CRC 错或 target 不匹配时不应用。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "app_calibration.h"

#include "app_calibration_config.h"
#include "app_gimbal.h"
#include "app_input.h"
#include "bsp_dts.h"
#include "buzzer.h"
#include "flash.h"
#include "main.h"
#include "cmsis_os.h"

#include <math.h>
#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置本 App 的私有宏、任务属性和内部状态类型，便于先建立文件边界。
 * ========================================================================== */
/** @brief 校准记录 magic。 */
#define APP_CALI_MAGIC ((uint32_t)0x43414C31UL)

/** @brief 校准记录版本。 */
#define APP_CALI_VERSION ((uint16_t)2U)

/** @brief 毫秒转 OS tick，向上取整。 */
#define APP_CALI_MS_TO_TICKS(ms) \
    ((uint32_t)((((uint32_t)(ms) * osKernelGetTickFreq()) + 999U) / 1000U))

/** @brief Head 温控目标温度下限，低于该值视为无效。 */
#define APP_CALI_TEMP_MIN 30.0F

/** @brief 旧 HeadCali 语义中的 MCU 温度到 IMU 控温目标偏移。 */
#define APP_CALI_DTS_TEMP_OFFSET 10.0F

/**
 * @brief 校准记录头。
 */
typedef struct
{
    uint32_t magic;       /**< 记录 magic。 */
    uint16_t version;     /**< 记录版本。 */
    uint16_t payload_len; /**< payload 长度。 */
    uint32_t target_id;   /**< 绑定 target id。 */
    uint32_t record_mask; /**< 有效记录位图。 */
    uint16_t crc;         /**< payload CRC16。 */
    uint16_t reserved;    /**< 对齐保留。 */
} AppCaliHeader;

/**
 * @brief 校准记录 payload。
 */
typedef struct
{
    GimbalCali gimbal; /**< 云台校准记录。 */
    ImuCali imu;       /**< IMU 校准记录。 */
    AppCaliHead head;  /**< Head 元信息记录。 */
} AppCaliPayload;

/**
 * @brief 校准记录完整块。
 */
typedef struct
{
    AppCaliHeader header;  /**< 记录头。 */
    AppCaliPayload payload;/**< 记录载荷。 */
} AppCaliBlock;

/* ==========================================================================
 * 任务对象与状态缓存
 * --------------------------------------------------------------------------
 * 校准与持久化 的全局状态只在本文件内保存，对外通过快照 API 暴露。
 * ========================================================================== */
/** @brief Calibration 任务控制块。 */
static osThreadId_t app_cali_thread;

/** @brief Calibration 任务属性。 */
static const osThreadAttr_t app_cali_attr = {
    .name = "app_cali",
    .stack_size = APP_CALI_STACK_SIZE,
    .priority = (osPriority_t)APP_CALI_PRIORITY,
};

/** @brief 校准记录缓存。 */
static AppCaliBlock app_cali_block;

/** @brief 应用层状态缓存。 */
static AppCalibrationState app_cali_state;

/** @brief 应用层诊断缓存。 */
static AppCalibrationDiag app_cali_diag;

/** @brief 最近一次已处理命令有效状态。 */
static uint8_t app_cali_last_cmd_valid;

/* ==========================================================================
 * 输出执行
 * --------------------------------------------------------------------------
 * 本区只调用设备或下层组件执行输出，失败时记录诊断并保持非阻塞语义。
 * ========================================================================== */
/** @brief 蜂鸣器关闭倒计数。 */
static uint16_t app_cali_buzzer_count;

/* ==========================================================================
 * 配置生成与参数展开
 * --------------------------------------------------------------------------
 * 本区把 Robot target 宏展开成 Component/Device 的 Cfg，运行期控制逻辑不写在配置头里。
 * ========================================================================== */
/**
 * @brief 判断温控目标温度是否可用。
 *
 * @param temp 温度值，单位摄氏度。
 * @return 可用返回 1，否则返回 0。
 */
static uint8_t app_cali_temp_valid(float32_t temp)
{
    if ((isfinite(temp) == 0) ||
        (temp < APP_CALI_TEMP_MIN) ||
        (temp > ROBOT_HEAD_IMU_TEMP_MAX)) {
        return 0U;
    }

    return 1U;
}

/* ==========================================================================
 * 输入采集与状态更新
 * --------------------------------------------------------------------------
 * Calibration 负责 Flash 记录、校准命令消费和具体 App/Device 校准入口调度。 这里把外部快照转换为本 App 周期内使用的输入结构。
 * ========================================================================== */
/**
 * @brief 读取 MCU DTS 温度。
 *
 * @param out 温度输出，单位摄氏度。
 * @return 成功返回 1，DTS 不可用或读数非法返回 0。
 */
static uint8_t app_cali_read_mcu_temp(float32_t *out)
{
    DTS_HandleTypeDef *hdts;
    int32_t raw_temp = 0;

    if (out == NULL) {
        return 0U;
    }

    hdts = bsp_dts_handle(BSP_DTS_ID_DTS);
    if (hdts == NULL) {
        return 0U;
    }

    if (bsp_dts_start(hdts) != HAL_OK) {
        return 0U;
    }
    if (bsp_dts_get_temperature(hdts, &raw_temp) != HAL_OK) {
        (void)bsp_dts_stop(hdts);
        return 0U;
    }
    (void)bsp_dts_stop(hdts);

    *out = (float32_t)raw_temp;
    return (isfinite(*out) != 0) ? 1U : 0U;
}

/**
 * @brief 生成 Head 记录默认温控目标。
 *
 * @return 温控目标温度，单位摄氏度。
 */
static float32_t app_cali_default_temp(void)
{
    float32_t temp;
    float32_t target;

    if (app_cali_read_mcu_temp(&temp) != 0U) {
        target = temp + APP_CALI_DTS_TEMP_OFFSET;
        if (target > ROBOT_HEAD_IMU_TEMP_MAX) {
            target = ROBOT_HEAD_IMU_TEMP_MAX;
        }
        if (app_cali_temp_valid(target) != 0U) {
            return target;
        }
    }

    if (app_cali_temp_valid(ROBOT_THERMAL_IMU_TARGET_TEMP) != 0U) {
        return ROBOT_THERMAL_IMU_TARGET_TEMP;
    }

    return ROBOT_HEAD_IMU_TEMP_MAX;
}

/**
 * @brief 生成运行期默认 Head 记录。
 *
 * @param head Head 记录输出。
 */
static void app_cali_make_default_head(AppCaliHead *head)
{
    if (head == NULL) {
        return;
    }

    head->self_id = (uint8_t)ROBOT_HEAD_SELF_ID;
    head->imu_target_temp = (int8_t)app_cali_default_temp();
    head->firmware_version = (uint16_t)ROBOT_FIRMWARE_VERSION;
    head->latitude = ROBOT_LOCATION_LATITUDE_DEG;
}

/**
 * @brief 判断 Head 记录是否可用。
 *
 * @param head Head 记录。
 * @return 可用返回 1，否则返回 0。
 */
static uint8_t app_cali_head_valid(const AppCaliHead *head)
{
    if (head == NULL) {
        return 0U;
    }
    if (app_cali_temp_valid((float32_t)head->imu_target_temp) == 0U) {
        return 0U;
    }
    if (isfinite(head->latitude) == 0) {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 确保当前缓存中存在可用 Head 记录。
 *
 * @note 该函数只更新 RAM 缓存和记录位，不主动写 Flash。
 */
static void app_cali_ensure_head(void)
{
    if (((app_cali_block.header.record_mask & APP_CALI_RECORD_HEAD) == 0UL) ||
        (app_cali_head_valid(&app_cali_block.payload.head) == 0U)) {
        app_cali_make_default_head(&app_cali_block.payload.head);
    }

    app_cali_block.header.record_mask |= APP_CALI_RECORD_HEAD;
}

/**
 * @brief 重置 RAM 校准块并生成运行期默认 Head。
 */
static void app_cali_reset_block(void)
{
    (void)memset(&app_cali_block, 0, sizeof(app_cali_block));
    app_cali_ensure_head();
}

/* ==========================================================================
 * 临界区与时间工具
 * --------------------------------------------------------------------------
 * 短临界区只保护快照复制和轻量状态切换，避免任务读写同一缓存时撕裂。
 * ========================================================================== */
/**
 * @brief 进入临界区。
 *
 * @return 进入前 PRIMASK。
 */
static uint32_t app_cali_enter(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

/**
 * @brief 退出临界区。
 *
 * @param primask 进入前 PRIMASK。
 */
static void app_cali_exit(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief CRC16-CCITT 逐字节更新。
 *
 * @param crc 当前 CRC。
 * @param byte 输入字节。
 * @return 更新后的 CRC。
 */
static uint16_t app_cali_crc16_byte(uint16_t crc, uint8_t byte)
{
    uint8_t i;

    crc ^= (uint16_t)byte << 8;
    for (i = 0U; i < 8U; i++) {
        if ((crc & 0x8000U) != 0U) {
            crc = (uint16_t)((crc << 1) ^ 0x1021U);
        } else {
            crc = (uint16_t)(crc << 1);
        }
    }

    return crc;
}

/**
 * @brief 计算 CRC16-CCITT。
 *
 * @param data 数据首地址。
 * @param length 数据长度。
 * @return CRC16。
 */
static uint16_t app_cali_crc16(const void *data, uint16_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint16_t crc = 0xFFFFU;
    uint16_t i;

    if (bytes == NULL) {
        return 0U;
    }

    for (i = 0U; i < length; i++) {
        crc = app_cali_crc16_byte(crc, bytes[i]);
    }

    return crc;
}

/**
 * @brief 刷新状态快照。
 */
static void app_cali_refresh_state(void)
{
    app_cali_state.ready = 1U;
    app_cali_state.target_id = ROBOT_TARGET_ID;
    app_cali_state.record_mask = app_cali_block.header.record_mask;
}

/**
 * @brief 校验当前记录块。
 *
 * @return 合法返回 1，否则返回 0。
 */
static uint8_t app_cali_block_valid(void)
{
    if (app_cali_block.header.magic != APP_CALI_MAGIC) {
        return 0U;
    }
    if (app_cali_block.header.version != APP_CALI_VERSION) {
        return 0U;
    }
    if (app_cali_block.header.target_id != ROBOT_TARGET_ID) {
        return 0U;
    }
    if (app_cali_block.header.payload_len != (uint16_t)sizeof(AppCaliPayload)) {
        return 0U;
    }
    if (app_cali_crc16(&app_cali_block.payload, (uint16_t)sizeof(AppCaliPayload)) != app_cali_block.header.crc) {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 从 Flash 加载记录。
 *
 * @return 成功返回 1，否则返回 0。
 */
static uint8_t app_cali_load(void)
{
    flash_status_t status;

    status = flash_read(APP_CALI_FLASH_ADDR, &app_cali_block, (uint32_t)sizeof(app_cali_block));
    app_cali_diag.flash_status = (int32_t)status;
    if ((status != FLASH_OK) || (app_cali_block_valid() == 0U)) {
        app_cali_state.storage_valid = 0U;
        app_cali_diag.load_fail_count++;
        return 0U;
    }

    app_cali_state.storage_valid = 1U;
    app_cali_diag.load_count++;
    app_cali_ensure_head();
    if ((app_cali_block.header.record_mask & APP_CALI_RECORD_IMU) != 0UL) {
        app_cali_diag.imu_status = (int32_t)imu_set_cali(&app_cali_block.payload.imu);
    }
    app_cali_refresh_state();
    return 1U;
}

/**
 * @brief 保存当前记录块到 Flash。
 *
 * @return 成功返回 1，否则返回 0。
 */
static uint8_t app_cali_save(void)
{
    flash_status_t status;

    app_cali_ensure_head();
    app_cali_block.header.magic = APP_CALI_MAGIC;
    app_cali_block.header.version = APP_CALI_VERSION;
    app_cali_block.header.payload_len = (uint16_t)sizeof(AppCaliPayload);
    app_cali_block.header.target_id = ROBOT_TARGET_ID;
    app_cali_block.header.crc = app_cali_crc16(&app_cali_block.payload, (uint16_t)sizeof(AppCaliPayload));
    app_cali_block.header.reserved = 0U;

    status = flash_write(APP_CALI_FLASH_ADDR, &app_cali_block, (uint32_t)sizeof(app_cali_block));
    app_cali_diag.flash_status = (int32_t)status;
    if (status != FLASH_OK) {
        app_cali_diag.save_fail_count++;
        return 0U;
    }

    app_cali_state.storage_valid = 1U;
    app_cali_refresh_state();
    app_cali_diag.save_count++;
    (void)buzzer_start(APP_CALI_BUZZER_HZ);
    app_cali_buzzer_count = APP_CALI_BUZZER_OFF_COUNT;
    return 1U;
}

/**
 * @brief 执行 IMU 校准并保存记录。
 */
static volatile uint8_t app_cali_imu_busy;

uint8_t app_calibration_is_busy(void)
{
    return app_cali_imu_busy;
}

static void app_cali_do_imu(void)
{
    imu_status_t status;

    app_cali_imu_busy = 1U;
    status = imu_calibrate();
    app_cali_imu_busy = 0U;
    app_cali_diag.imu_status = (int32_t)status;
    if (status != IMU_OK) {
        app_cali_diag.save_fail_count++;
        return;
    }
    status = imu_get_cali(&app_cali_block.payload.imu);
    app_cali_diag.imu_status = (int32_t)status;
    if (status != IMU_OK) {
        app_cali_diag.save_fail_count++;
        return;
    }

    app_cali_block.header.record_mask |= APP_CALI_RECORD_IMU;
    app_cali_diag.imu_cali_count++;
    (void)app_cali_save();
}

/**
 * @brief 检查云台校准是否完成并保存。
 *
 * @param last_done_count 上一次云台完成计数。
 */
static void app_cali_check_gimbal(uint32_t *last_done_count)
{
    AppGimbalDiag diag;
    GimbalCali cali;

    if (last_done_count == NULL) {
        return;
    }
    if (app_gimbal_get_diag(&diag) == 0U) {
        return;
    }
    if ((diag.cali_done_count == 0U) || (diag.cali_done_count == *last_done_count)) {
        return;
    }
    *last_done_count = diag.cali_done_count;

    if (app_gimbal_get_cali(&cali) == 0U) {
        app_cali_diag.save_fail_count++;
        return;
    }
    app_cali_block.payload.gimbal = cali;
    app_cali_block.header.record_mask |= APP_CALI_RECORD_GIMBAL;
    (void)app_cali_save();
}

/**
 * @brief 消费 Input 校准命令。
 */
static void app_cali_consume_cmd(void)
{
    InputCaliCmd cmd;

    if (app_input_get_cali_cmd(&cmd) == 0U) {
        app_cali_last_cmd_valid = 0U;
        return;
    }
    if ((cmd.valid == 0U) || (cmd.online == 0U)) {
        app_cali_last_cmd_valid = 0U;
        return;
    }
    if (app_cali_last_cmd_valid != 0U) {
        return;
    }

    app_cali_last_cmd_valid = 1U;
    app_cali_diag.last_cmd_action = (uint32_t)cmd.action;
    if (cmd.action == InputCaliGimbal) {
        if (app_gimbal_start_cali() != 0U) {
            app_cali_diag.gimbal_start_count++;
            (void)buzzer_start(APP_CALI_BUZZER_HZ);
        }
    } else if (cmd.action == InputCaliImu) {
        app_cali_do_imu();
    }
}

/**
 * @brief 处理蜂鸣器关闭倒计数。
 */
static void app_cali_buzzer_step(void)
{
    if (app_cali_buzzer_count == 0U) {
        return;
    }

    app_cali_buzzer_count--;
    if (app_cali_buzzer_count == 0U) {
        (void)buzzer_off();
    }
}

/* ==========================================================================
 * 任务入口
 * --------------------------------------------------------------------------
 * 任务入口只按固定周期串起采集、step、输出和诊断，不在这里新增分支协议。
 * ========================================================================== */
/**
 * @brief Calibration 任务入口。
 *
 * @param argument CMSIS-RTOS2 参数。
 */
static void app_cali_entry(void *argument)
{
    uint32_t last_gimbal_done = 0U;

    (void)argument;

    while (1) {
        app_cali_consume_cmd();
        app_cali_check_gimbal(&last_gimbal_done);
        app_cali_buzzer_step();
        app_cali_diag.loop_count++;
        osDelay(APP_CALI_MS_TO_TICKS(APP_CALI_PERIOD_MS));
    }
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 提供初始化、快照读取和显式注入，保持 App 之间的依赖可追踪。
 * ========================================================================== */
uint8_t app_calibration_init(void)
{
    (void)memset(&app_cali_state, 0, sizeof(app_cali_state));
    (void)memset(&app_cali_diag, 0, sizeof(app_cali_diag));
    app_cali_reset_block();
    app_cali_last_cmd_valid = 0U;
    app_cali_buzzer_count = 0U;

    app_cali_diag.flash_status = (int32_t)flash_init();
    if (app_cali_diag.flash_status != (int32_t)FLASH_OK) {
        app_cali_diag.load_fail_count++;
    } else {
        if (app_cali_load() == 0U) {
            app_cali_reset_block();
        }
    }
    app_cali_refresh_state();

    app_cali_thread = osThreadNew(app_cali_entry, NULL, &app_cali_attr);
    return (app_cali_thread != NULL) ? 1U : 0U;
}

uint8_t app_calibration_apply_gimbal(void)
{
    uint32_t primask;
    uint8_t ok;

    if ((app_cali_state.storage_valid == 0U) ||
        ((app_cali_block.header.record_mask & APP_CALI_RECORD_GIMBAL) == 0UL)) {
        return 0U;
    }

    ok = app_gimbal_set_cali(&app_cali_block.payload.gimbal);
    primask = app_cali_enter();
    app_cali_diag.gimbal_applied = ok;
    app_cali_exit(primask);
    return ok;
}

uint8_t app_calibration_get_head(AppCaliHead *out)
{
    uint32_t primask;
    AppCaliHead head;

    if (out == NULL) {
        return 0U;
    }

    primask = app_cali_enter();
    head = app_cali_block.payload.head;
    app_cali_exit(primask);

    if (app_cali_head_valid(&head) == 0U) {
        return 0U;
    }

    *out = head;
    return 1U;
}

uint8_t app_calibration_get_state(AppCalibrationState *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_cali_enter();
    *out = app_cali_state;
    app_cali_exit(primask);
    return 1U;
}

uint8_t app_calibration_get_diag(AppCalibrationDiag *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_cali_enter();
    *out = app_cali_diag;
    app_cali_exit(primask);
    return 1U;
}

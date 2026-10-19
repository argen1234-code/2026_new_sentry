/**
 ******************************************************************************
 * @file    app_calibration.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   Calibration 应用层接口。
 *
 * @note    本模块负责校准记录加载、保存和校准命令消费；具体校准动作仍由
 *          对应 Device/App 暴露的接口执行。
 * @note    本模块不兼容旧任务式校准入口和旧 Flash 记录格式。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_CALIBRATION_H
#define PRINTK_APPS_CALIBRATION_H

#include <stdint.h>

#include "arm_math.h"
#include "gimbal.h"
#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 校准记录：云台校准有效。 */
#define APP_CALI_RECORD_GIMBAL ((uint32_t)0x00000001UL)

/** @brief 校准记录：IMU 校准有效。 */
#define APP_CALI_RECORD_IMU ((uint32_t)0x00000002UL)

/** @brief 校准记录：Head 元信息有效。 */
#define APP_CALI_RECORD_HEAD ((uint32_t)0x00000004UL)

/* ==========================================================================
 * 状态与诊断类型
 * --------------------------------------------------------------------------
 * App 头文件只暴露快照、诊断和必要的注入类型，让下游按稳定接口读取状态。
 * ========================================================================== */
/**
 * @brief Head 校准元信息。
 *
 * 该记录只保存当前 target 的身份、固件版本、IMU 控温目标和地理纬度。
 * 它不恢复旧版 Head 结构，也不作为控制链路中的全局数据源。
 */
typedef struct
{
    uint8_t self_id;             /**< 当前 target 自身 ID。 */
    int8_t imu_target_temp;      /**< IMU 控温目标温度，单位摄氏度。 */
    uint16_t firmware_version;   /**< 固件版本号。 */
    float32_t latitude;          /**< 地理纬度，单位 degree。 */
} AppCaliHead;

/**
 * @brief Calibration 应用层状态。
 */
typedef struct
{
    uint8_t ready;          /**< 非 0 表示 app 已初始化。 */
    uint8_t storage_valid;  /**< 非 0 表示 Flash 记录有效。 */
    uint32_t record_mask;   /**< 当前有效记录位图。 */
    uint32_t target_id;     /**< 记录绑定的 target id。 */
} AppCalibrationState;

/**
 * @brief Calibration 应用层诊断。
 */
typedef struct
{
    uint32_t loop_count;        /**< 任务循环次数。 */
    uint32_t load_count;        /**< 成功加载记录次数。 */
    uint32_t load_fail_count;   /**< 加载失败次数。 */
    uint32_t save_count;        /**< 成功保存次数。 */
    uint32_t save_fail_count;   /**< 保存失败次数。 */
    uint32_t gimbal_start_count;/**< 触发云台校准次数。 */
    uint32_t imu_cali_count;    /**< 触发 IMU 校准次数。 */
    uint32_t last_cmd_action;   /**< 最近一次消费的校准动作。 */
    int32_t flash_status;       /**< 最近一次 Flash 状态。 */
    int32_t imu_status;         /**< 最近一次 IMU 状态。 */
    uint8_t gimbal_applied;     /**< 非 0 表示已向 Gimbal App 应用校准。 */
} AppCalibrationDiag;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 公开接口按初始化、快照读取和外部注入排列；旧 task/global API 不再作为入口。
 * ========================================================================== */
/**
 * @brief 初始化并启动 Calibration 任务。
 *
 * @return 成功返回 1，失败返回 0。
 */
uint8_t app_calibration_init(void);

/**
 * @brief 向 Gimbal App 应用已加载的云台校准。
 *
 * @return 成功应用返回 1，无有效记录或应用失败返回 0。
 */
uint8_t app_calibration_apply_gimbal(void);

/**
 * @brief 读取 Head 校准元信息。
 *
 * @param out Head 元信息输出。
 * @return 成功返回 1，参数无效或记录无效返回 0。
 */
uint8_t app_calibration_get_head(AppCaliHead *out);

/**
 * @brief 读取 Calibration 应用层状态。
 *
 * @param out 状态输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_calibration_get_state(AppCalibrationState *out);

/**
 * @brief 读取 Calibration 应用层诊断。
 *
 * @param out 诊断输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_calibration_get_diag(AppCalibrationDiag *out);

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_CALIBRATION_H */

/**
  * @file       comm_app_config.h
  * @brief      设备侧通信应用可配置参数
  * @details    将任务属性、通道ID、GPIO 触发等易变参数统一集中，便于工程按需覆盖。
  *             使用方式：在包含 comm_app.h 之前，先包含本文件并按需修改宏定义；
  *             或在编译器宏里用 -DKEY=VALUE 的方式覆盖默认值。
  */

#ifndef COMM_APP_CONFIG_H
#define COMM_APP_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 任务相关（可在 CubeMX/OS 层适配） ========== */
#ifndef COMM_APP_STACK
#define COMM_APP_STACK            640u            /**< 任务栈大小（字） */
#endif

#ifndef COMM_APP_PRIO
#define COMM_APP_PRIO             osPriorityBelowNormal /**< 任务优先级 */
#endif

#ifndef COMM_APP_LOOP_DELAY_MS
#define COMM_APP_LOOP_DELAY_MS    1u              /**< 主循环延时（毫秒） */
#endif

#ifndef COMM_APP_USB_ENUM_TIMEOUT_MS
#define COMM_APP_USB_ENUM_TIMEOUT_MS 5000u        /**< 等待 USB 配置枚举的最长时间（毫秒） */
#endif

/* ========== 自定义流通道（演示用） ========== */
#ifndef COMM_APP_STREAM_CH_ID
#define COMM_APP_STREAM_CH_ID     2u              /**< 自定义“回环”流通道的 MUX Channel ID */
#endif

#ifndef COMM_APP_STREAM_SID_DATA
#define COMM_APP_STREAM_SID_DATA  0x0101u         /**< 自定义流通道的数据 SID */
#endif

/* ========== 可选：相机触发 GPIO 轮询配置 ========== */
#ifndef CAM_TRIGGER_ENABLE
#define CAM_TRIGGER_ENABLE        0               /**< 1 启用 GPIO 轮询触发，0 禁用 */
#endif

#ifndef CAM_TRIGGER_ACTIVE_HIGH
#define CAM_TRIGGER_ACTIVE_HIGH   1               /**< 1 上升沿有效，0 下降沿有效 */
#endif

/* 若启用 GPIO 轮询触发，请在工程中定义以下两个宏（由 CubeMX 提供）：
 *   - CAM_TRIGGER_GPIO_PORT  （例如 GPIOC）
 *   - CAM_TRIGGER_GPIO_PIN   （例如 GPIO_PIN_13）
 */

/* ========== TFmini 上报配置 ========== */
#ifndef TFMINI_ENABLE
#define TFMINI_ENABLE 1                   /**< 1 启用TFmini上报，0 禁用 */
#endif

#ifndef TFMINI_PUB_PERIOD_MS
#define TFMINI_PUB_PERIOD_MS 20u          /**< TFmini发布周期（毫秒） */
#endif

#ifdef __cplusplus
}
#endif

#endif /* COMM_APP_CONFIG_H */

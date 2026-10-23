/**
 ******************************************************************************
 * @file    communication_usb.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-10-19
 * @brief   Jetson <-> STM32 USB CDC 桥接协议。
 *
 * @note    帧格式：AA 55 | id | seq | len | payload[len] | xor
 * @note    xor = id ^ seq ^ len ^ payload[0..len-1]
 * @note    多字节字段统一小端，浮点使用 IEEE-754 float32。
 * @note    本模块只做 USB 帧收发和命令缓存，不直接驱动电机。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMMUNICATION_USB_H
#define PRINTK_COMMUNICATION_USB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 帧常量
 * ========================================================================== */
#define COMMUNICATION_USB_SOF0                       0xAAU
#define COMMUNICATION_USB_SOF1                       0x55U
#define COMMUNICATION_USB_MAX_PAYLOAD                64U

#define COMMUNICATION_USB_ID_HOST_HEARTBEAT          0x01U
#define COMMUNICATION_USB_ID_UNIFIED_CMD             0x10U
#define COMMUNICATION_USB_ID_MCU_STATE               0x81U
#define COMMUNICATION_USB_ID_MCU_HEARTBEAT           0x82U
#define COMMUNICATION_USB_ID_WHEEL_STATE             0x83U
#define COMMUNICATION_USB_ID_IMU_STATE               0x84U

#define COMMUNICATION_USB_UNIFIED_CMD_LEN            45U
#define COMMUNICATION_USB_MCU_STATE_LEN              40U
#define COMMUNICATION_USB_MCU_HEARTBEAT_LEN          8U
#define COMMUNICATION_USB_WHEEL_STATE_LEN            58U
#define COMMUNICATION_USB_IMU_STATE_LEN              58U

#define COMMUNICATION_USB_HOST_TIMEOUT_MS            500U

/* chassis_status bits in UnifiedCmd */
#define COMMUNICATION_USB_CMD_VALID                  0x0001U
#define COMMUNICATION_USB_CMD_MOTION_ENABLE          0x0002U
#define COMMUNICATION_USB_CMD_FRESH_MS               100U

/* ==========================================================================
 * 命令与状态
 * ========================================================================== */

/**
 * @brief Jetson 下发的统一控制命令。
 */
typedef struct
{
    int32_t delta_yaw_udeg;    /**< 云台 yaw 增量，单位 udeg。 */
    int32_t delta_pitch_udeg;  /**< 云台 pitch 增量，单位 udeg。 */
    uint16_t gimbal_status;    /**< 云台状态位。 */
    int32_t vx_mm_s;           /**< 底盘 X 速度，单位 mm/s。 */
    int32_t vy_mm_s;           /**< 底盘 Y 速度，单位 mm/s。 */
    int32_t wz_mdeg_s;         /**< 底盘角速度，单位 mdeg/s。 */
    int32_t chassis_mode;      /**< 底盘模式。 */
    uint16_t chassis_status;   /**< 底盘状态位。 */
    uint8_t fire_on;           /**< 非 0 表示请求开火。 */
    int32_t fire_mode;         /**< 射击模式。 */
    int32_t burst_count;       /**< 连发数量。 */
    uint64_t host_ts_ns;       /**< 主机时间戳，单位 ns。 */
    uint32_t rx_ms;            /**< MCU 接收时间，单位 ms。 */
    uint8_t valid;             /**< 非 0 表示命令有效且未超时。 */
} CommunicationUsbCmd;

/**
 * @brief 上报给 Jetson 的机器人状态。
 */
typedef struct
{
    uint32_t mcu_ts_ms;        /**< MCU 时间戳，单位 ms。 */
    float imu_roll;            /**< IMU roll，单位 rad。 */
    float imu_pitch;           /**< IMU pitch，单位 rad。 */
    float imu_yaw;             /**< IMU yaw，单位 rad。 */
    float gimbal_yaw;          /**< 云台 yaw，单位 rad。 */
    float gimbal_pitch;        /**< 云台 pitch，单位 rad。 */
    float vx;                  /**< 底盘 X 速度，单位 m/s。 */
    float vy;                  /**< 底盘 Y 速度，单位 m/s。 */
    float wz;                  /**< 底盘角速度，单位 rad/s。 */
    uint8_t mode;              /**< 当前模式。 */
    uint8_t status;            /**< 状态位：bit0 主机在线，bit1 INS 有效，bit2 云台反馈在线。 */
    uint16_t error_flags;      /**< 错误标志。 */
} CommunicationUsbState;

/**
 * @brief 四轮底盘动态状态。
 *
 * 轮序固定为 RR, FL, RL, FR，与 app_chassis_diag 的 drive_* 数组一致。
 */
typedef struct
{
    uint32_t mcu_ts_ms;        /**< MCU 时间戳，单位 ms。 */
    uint8_t online_mask;       /**< 驱动轮在线掩码，bit0..3 = RR, FL, RL, FR。 */
    uint8_t mode;              /**< ChassisMode 枚举。 */
    uint16_t chassis_status;   /**< ChassisStatus 枚举。 */
    uint16_t fault_flags;      /**< 故障位，见实现。 */
    float speed_mps[4];        /**< 四轮反馈速度，m/s。 */
    float position_rad[4];     /**< 四轮位置，rad（电机侧，直驱 1:1）。 */
    uint8_t temp_mos_c[4];     /**< MOS 温度，摄氏度。 */
    uint8_t temp_motor_c[4];   /**< 电机温度，摄氏度。 */
    int16_t torque_mnm[4];     /**< 力矩，mN*m。 */
} CommunicationUsbWheelState;

/**
 * @brief IMU 动态状态。
 */
typedef struct
{
    uint32_t mcu_ts_ms;        /**< MCU 时间戳，单位 ms。 */
    float gyro[3];             /**< 角速度，rad/s。 */
    float accel[3];            /**< 加速度，m/s^2。 */
    float temperature_c;       /**< 温度，摄氏度。 */
    float quat_wxyz[4];        /**< 四元数 w,x,y,z；valid_mask bit1 为 0 时无效。 */
    int32_t imu_status;        /**< IMU/Observer 状态。 */
    uint8_t valid_mask;        /**< bit0 gyro+accel，bit1 attitude，bit2 temperature。 */
    uint8_t reserved;          /**< 保留。 */
    uint32_t last_ok_ms;       /**< 最近一次有效读取时间，单位 ms。 */
} CommunicationUsbImuState;

/**
 * @brief 桥接诊断。
 */
typedef struct
{
    uint32_t rx_frames;        /**< 有效接收帧数。 */
    uint32_t rx_errors;        /**< CRC/长度/格式错误数。 */
    uint32_t tx_frames;        /**< 成功入队发送帧数。 */
    uint32_t tx_errors;        /**< 发送失败次数。 */
    uint32_t rx_overflow;      /**< 接收环形缓冲溢出字节数。 */
    uint8_t ready;             /**< 非 0 表示已初始化。 */
    uint8_t link_online;       /**< 非 0 表示主机心跳在线。 */
} CommunicationUsbDiag;

/* ==========================================================================
 * 公开接口
 * ========================================================================== */

void communication_usb_init(void);
void communication_usb_poll(uint32_t now_ms);
void communication_usb_send_state(const CommunicationUsbState *state);
void communication_usb_send_wheel_state(const CommunicationUsbWheelState *state);
void communication_usb_send_imu_state(const CommunicationUsbImuState *state);
void communication_usb_send_heartbeat(uint32_t now_ms, uint8_t mode, uint8_t status);
uint8_t communication_usb_get_cmd(CommunicationUsbCmd *out);
uint8_t communication_usb_link_online(uint32_t now_ms);
void communication_usb_get_diag(CommunicationUsbDiag *out);

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_COMMUNICATION_USB_H */

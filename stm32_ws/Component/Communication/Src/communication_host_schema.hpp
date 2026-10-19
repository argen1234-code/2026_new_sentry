/**
 ******************************************************************************
 * @file    communication_host_schema.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   485 host 实时通讯协议事实源。
 *
 * @note    本文件只描述 MCU 与 host 之间的紧凑实时帧，不绑定 UART、USB、
 *          任务调度或 RTT 周期。
 * @note    生成代码由 Tools/Printk-Protocol 展开到 Src/Generate。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */


/* ==========================================================================
 * 协议事实源
 * --------------------------------------------------------------------------
 * 本文件只描述 RT 核消息结构、ID 和方向，具体 transport 绑定留给
 * Component/Communication 与 App 层。
 * ========================================================================== */
namespace proto2::rt;

/**
 * @brief host 下发的云台自瞄参考。
 */
message HostAim @msg_id(0x01) @direction(host_to_device) {
    float yaw_rad;
    float pitch_rad;
    uint8_t valid_mask;
    uint8_t enable;
    uint32_t host_time_us;
}

/**
 * @brief MCU 上报的云台状态快照。
 */
message GimbalState @msg_id(0x02) @direction(device_to_host) {
    float yaw_rad;
    float pitch_rad;
    float yaw_rate_rad_s;
    float pitch_rate_rad_s;
    uint8_t status;
    uint32_t time_us;
}

/**
 * @brief 空闲 RTT 探测请求。
 */
message RtProbe @msg_id(0x03) @direction(bidirectional) {
    uint16_t token;
    uint32_t time_us;
}

/**
 * @brief 空闲 RTT 探测应答。
 */
message RtProbeAck @msg_id(0x04) @direction(bidirectional) {
    uint16_t token;
    uint32_t request_time_us;
    uint32_t reply_time_us;
}

/**
 * @brief 云台板发往底盘板的状态快照。
 */
message CommunicationBoardState @msg_id(0x05) @direction(bidirectional) {
    float yaw_motor_rad;
    float ins_yaw_rad;
    float ins_pitch_rad;
    float ins_roll_rad;
    uint8_t gimbal_mode;
    uint8_t shoot_mode;
    int16_t rc_ch0;
    int16_t rc_ch1;
    int16_t rc_ch2;
    int16_t rc_ch3;
    uint8_t rc_sw0;
    uint8_t rc_sw1;
    uint8_t status;
    uint32_t time_us;
}

/**
 * @brief 底盘板发往云台板的状态快照。
 */
message CommunicationChassisState @msg_id(0x06) @direction(bidirectional) {
    float yaw_rad;
    float pitch_rad;
    float roll_rad;
    uint16_t robot_id;
    uint8_t status;
    uint32_t time_us;
}

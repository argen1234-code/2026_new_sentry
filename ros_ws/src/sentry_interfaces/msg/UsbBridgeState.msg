# Jetson <-> STM32 USB CDC 桥接状态（MCU 0x81 帧）

std_msgs/Header header

uint32 mcu_ts_ms

float32 imu_roll
float32 imu_pitch
float32 imu_yaw

float32 gimbal_yaw
float32 gimbal_pitch

float32 vx
float32 vy
float32 wz

uint8 mode
uint8 status
uint16 error_flags

bool link_online
uint32 rx_frames
uint32 crc_errors

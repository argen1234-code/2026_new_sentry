# Dynamic four-wheel chassis state from the STM32 USB bridge.
# Wheel order is RR, FL, RL, FR (matching AppChassisDiag drive_* arrays).
std_msgs/Header header
uint32 seq
uint32 mcu_ts_ms
uint8 online_mask
uint8 mode
uint16 chassis_status
uint16 fault_flags
float32[4] speed_mps
float32[4] position_rad
float32[4] torque_nm
uint8[4] temp_mos_c
uint8[4] temp_motor_c

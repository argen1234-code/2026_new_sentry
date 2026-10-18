# 视觉自瞄命令消息（对应 DM-02 协议的 GimbalDelta）
std_msgs/Header header

# 云台角度增量（单位：微度 udeg，1度 = 1,000,000 微度）
int32 delta_yaw_udeg      # 偏航角增量
int32 delta_pitch_udeg    # 俯仰角增量

# 状态位
# bit0: target_valid (上位机有有效目标)
uint16 status

# 时间戳（纳秒）
uint64 host_ts_ns

# 射击控制（可选，用于统一命令模式）
int32 fire_on             # 射击开关 (0=关闭, 1=开启)
int32 fire_mode           # 射击模式 (0=单发, 1=连发)
int32 burst_count         # 连发数量

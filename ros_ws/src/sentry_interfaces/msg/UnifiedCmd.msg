# 统一命令消息：包含视觉云台控制和导航底盘控制
# 用于将视觉和导航的控制命令合并为一帧发送给下位机
std_msgs/Header header

# ========== 云台控制部分（来自视觉自瞄） ==========
# 云台角度增量（单位：微度 udeg，1度 = 1,000,000 微度）
int32 delta_yaw_udeg      # 偏航角增量
int32 delta_pitch_udeg    # 俯仰角增量

# 云台控制状态位
# bit0: target_valid (上位机有有效目标)
uint16 gimbal_status

# ========== 底盘控制部分（来自导航） ==========
# 底盘速度命令（单位：mm/s 和 mdeg/s）
int32 vx_mm_s             # X方向速度 (mm/s)
int32 vy_mm_s             # Y方向速度 (mm/s)
int32 wz_mdeg_s           # 旋转角速度 (mdeg/s)

# 底盘控制模式和状态
int32 chassis_mode        # 底盘模式
uint16 chassis_status     # bit0: 命令有效

# ========== 射击控制部分 ==========
bool fire_on              # 是否开火
int32 fire_mode           # 射击模式
int32 burst_count         # 连发数量

# ========== 时间戳 ==========
uint64 host_ts_ns         # 主机时间戳（纳秒）

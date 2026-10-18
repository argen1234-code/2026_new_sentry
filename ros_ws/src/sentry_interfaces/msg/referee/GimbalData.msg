# 云台裁判数据 (SID=0x0204, 下位机通过 uproto/MUX 帧发送)
std_msgs/Header header

uint8 game_progress          # 比赛进度 (0=未开始 1=准备 2=自检 3=倒计时 4=比赛中 5=结算)
uint16 stage_remain_time     # 阶段剩余时间（秒）
uint8 game_result            # 比赛结果 (0=平局 1=红方胜 2=蓝方胜)
uint16 current_hp            # 当前血量
uint16 shooter_42mm_heat     # 42mm发射热量
uint64 ts_us                 # 下位机时间戳（微秒）

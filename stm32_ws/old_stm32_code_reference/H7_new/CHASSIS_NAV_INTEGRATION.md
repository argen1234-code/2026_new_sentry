# 底盘导航命令集成指南

## 概述

本文档说明如何在 H7 下位机中集成来自上位机导航系统的底盘速度命令。

## 数据流

```
ROS2 导航 -> cmd_vel (Twist) -> unified_serial_bridge -> DM-02协议 -> 
H7串口 -> gimbal_channel -> chassis_task -> 底盘电机
```

## 已完成的修改

### 1. 通信层 (gimbal_channel)

**文件**: `User/Communication/channel/gimbal/gimbal_channel.h`

添加了：
- `GIMBAL_SID_CHASSIS` (0x0206) 定义
- `gimbal_chassis_cmd_t` 结构体
- `on_chassis` 回调函数指针

**文件**: `User/Communication/channel/gimbal/gimbal_channel.c`

在 `on_rx()` 函数中添加了 CHASSIS 命令解析：
```c
if(sid == GIMBAL_SID_CHASSIS) {
    if(len < 2 + 4 + 4 + 4 + 4 + 2 + 8)
        return;
    gimbal_chassis_cmd_t cmd;
    cmd.vx_mm_s = comm_read_i32_le(&payload[2]);
    cmd.vy_mm_s = comm_read_i32_le(&payload[6]);
    cmd.wz_mdeg_s = comm_read_i32_le(&payload[10]);
    cmd.mode = comm_read_i32_le(&payload[14]);
    cmd.status = comm_read_u16_le(&payload[18]);
    cmd.ts_us = comm_read_u64_le(&payload[20]);
    
    // 调用底盘控制函数
    if(gc->hooks.on_chassis) gc->hooks.on_chassis(&cmd, gc->hooks.user);
    return;
}
```

## 需要实现的部分

### 2. 底盘任务集成

**文件**: `User/APP/chassis_task.c` 或相应的底盘控制文件

#### 2.1 添加全局变量存储导航命令

```c
// 导航命令缓存（线程安全）
static volatile struct {
    int32_t vx_mm_s;
    int32_t vy_mm_s;
    int32_t wz_mdeg_s;
    int32_t mode;
    uint16_t status;
    uint64_t ts_us;
    uint32_t version;
} g_nav_cmd = {0};
```

#### 2.2 实现命令接收函数

```c
/**
 * @brief 应用导航底盘命令
 * @param vx_mm_s 前向速度 mm/s
 * @param vy_mm_s 侧向速度 mm/s
 * @param wz_mdeg_s 角速度 mdeg/s
 * @param mode 底盘模式
 * @param status 状态位 (bit0=命令有效)
 */
void chassis_apply_nav_cmd(int32_t vx_mm_s, int32_t vy_mm_s, int32_t wz_mdeg_s, 
                           int32_t mode, uint16_t status, uint64_t ts_us)
{
    g_nav_cmd.vx_mm_s = vx_mm_s;
    g_nav_cmd.vy_mm_s = vy_mm_s;
    g_nav_cmd.wz_mdeg_s = wz_mdeg_s;
    g_nav_cmd.mode = mode;
    g_nav_cmd.status = status;
    g_nav_cmd.ts_us = ts_us;
    g_nav_cmd.version++;
}

/**
 * @brief 读取最新导航命令
 * @param out 输出命令缓冲区
 * @param io_version 版本号（用于检测新命令）
 * @return true=有新命令，false=无新命令
 */
bool chassis_get_nav_cmd(int32_t *vx_mm_s, int32_t *vy_mm_s, int32_t *wz_mdeg_s,
                         uint32_t *io_version)
{
    if (!vx_mm_s || !vy_mm_s || !wz_mdeg_s || !io_version)
        return false;
    
    uint32_t v = g_nav_cmd.version;
    if (v == *io_version)
        return false;  // 没有新命令
    
    *vx_mm_s = g_nav_cmd.vx_mm_s;
    *vy_mm_s = g_nav_cmd.vy_mm_s;
    *wz_mdeg_s = g_nav_cmd.wz_mdeg_s;
    *io_version = v;
    
    return true;
}
```

#### 2.3 在底盘任务中使用导航命令

```c
void chassis_task(void *pvParameters)
{
    static uint32_t nav_cmd_version = 0;
    int32_t vx_mm_s, vy_mm_s, wz_mdeg_s;
    
    while(1) {
        // 检查是否有新的导航命令
        if (chassis_get_nav_cmd(&vx_mm_s, &vy_mm_s, &wz_mdeg_s, &nav_cmd_version)) {
            // 单位转换：mm/s -> m/s, mdeg/s -> deg/s
            float vx_m_s = vx_mm_s / 1000.0f;
            float vy_m_s = vy_mm_s / 1000.0f;
            float wz_deg_s = wz_mdeg_s / 1000.0f;
            
            // 应用到底盘控制
            // 具体实现取决于你的底盘控制架构
            chassis_set_velocity(vx_m_s, vy_m_s, wz_deg_s);
        }
        
        // 其他底盘控制逻辑...
        
        vTaskDelay(pdMS_TO_TICKS(2));  // 2ms 周期
    }
}
```

### 3. 注册回调函数

**文件**: `User/Communication/example/device/comm_app.c` 或初始化文件

```c
static void on_chassis_cmd(const gimbal_chassis_cmd_t *cmd, void *user)
{
    (void)user;
    if (!cmd) return;
    
    // 调用底盘应用函数
    chassis_apply_nav_cmd(cmd->vx_mm_s, cmd->vy_mm_s, cmd->wz_mdeg_s,
                          cmd->mode, cmd->status, cmd->ts_us);
}

// 在 setup_channels() 中注册
void setup_channels(void)
{
    // ... 其他初始化代码 ...
    
    gimbal_hooks_t gimbal_hooks = {
        .on_delta = gimbal_on_delta,
        .on_big_yaw_delta = gimbal_on_big_yaw_delta,
        .on_chassis = on_chassis_cmd,  // 新增
        .user = NULL
    };
    
    // ... 其他初始化代码 ...
}
```

## 单位转换说明

| 物理量 | ROS2 (Twist) | DM-02 协议 | 转换 |
|--------|--------------|------------|------|
| 前向速度 | linear.x (m/s) | vx_mm_s | × 1000 |
| 侧向速度 | linear.y (m/s) | vy_mm_s | × 1000 |
| 角速度 | angular.z (rad/s) | wz_mdeg_s | × 1000 × 180/π |

## 调试建议

1. 在 `on_rx()` 中添加断点，确认收到 CHASSIS 命令
2. 在 `chassis_apply_nav_cmd()` 中打印接收到的速度值
3. 检查 `g_nav_cmd.status` bit0 是否为 1（命令有效）
4. 确认底盘控制模式允许接收导航命令

## 注意事项

1. 导航命令和手动遥控命令需要做模式切换管理
2. 建议添加超时检测：如果长时间未收到新命令，停止底盘
3. 速度限幅：根据机器人性能限制最大速度
4. 坐标系对齐：确保 ROS2 和下位机的坐标系定义一致

# 哨兵决策节点 (Sentry Decision Node)

## 功能描述

这是一个用于RoboMaster哨兵机器人的决策节点，根据机器人的血量状态自动进行导航决策：

- **血量健康时**：机器人会按照配置的巡逻点进行巡逻
- **血量低时**：机器人会自动返回基地（家的位置）

## 功能特性

- ✅ 实时监控机器人血量状态
- ✅ 自动切换巡逻和回家模式
- ✅ 支持多个巡逻点配置
- ✅ 支持途径点配置（可选）
- ✅ 可配置血量阈值
- ✅ 使用Nav2导航框架

## 编译

```bash
cd ~/2026_sentry_ws
colcon build --packages-select sentry_decision
source install/setup.bash
```

## 配置参数

编辑配置文件 `config/sentry_decision_params.yaml`：

```yaml
sentry_decision_node:
  ros__parameters:
    # 血量阈值比例（低于此比例回家，默认0.3即30%）
    hp_threshold_ratio: 0.3
    
    # 机器人状态话题
    robot_status_topic: "/robot_status"
    
    # 检查间隔（秒）
    check_interval: 1.0
    
    # 家的位置 [x, y, yaw]
    home_pose: [0.0, 0.0, 0.0]
    
    # 巡逻点位 [x1, y1, yaw1, x2, y2, yaw2, ...]
    patrol_poses: [
      2.0, 1.0, 0.0,    # 第一个巡逻点
      2.0, -1.0, 3.14   # 第二个巡逻点
    ]
    
    # 是否使用途径点
    use_waypoints: false
    
    # 途径点 [x1, y1, yaw1, x2, y2, yaw2, ...]
    waypoints: [
      1.0, 0.0, 0.0
    ]
```

### 参数说明

- `hp_threshold_ratio`: 血量阈值比例，当 `current_hp / maximum_hp < hp_threshold_ratio` 时触发回家
- `robot_status_topic`: 订阅机器人状态的话题名称
- `check_interval`: 检查血量状态的时间间隔（秒）
- `home_pose`: 基地位置，格式为 [x, y, yaw]
- `patrol_poses`: 巡逻点列表，每3个数字表示一个点 [x, y, yaw]
- `use_waypoints`: 是否启用途径点功能
- `waypoints`: 途径点列表，机器人会先经过这些点再到达目标点

## 运行

### 单独运行决策节点

```bash
ros2 launch sentry_decision sentry_decision_launch.py
```

### 使用自定义参数文件

```bash
ros2 launch sentry_decision sentry_decision_launch.py params_file:=/path/to/your/params.yaml
```

### 直接运行节点（用于调试）

```bash
ros2 run sentry_decision sentry_decision_node --ros-args --params-file config/sentry_decision_params.yaml
```

## 话题接口

### 订阅的话题

- `/robot_status` (pb_rm_interfaces/msg/RobotStatus): 机器人状态信息，包含血量数据

### 使用的Action

- `navigate_through_poses` (nav2_msgs/action/NavigateThroughPoses): Nav2导航action

## 工作流程

1. 节点启动后，开始订阅机器人状态话题
2. 定时检查当前血量与最大血量的比例
3. 根据血量比例决策：
   - 如果血量 >= 阈值：进入巡逻模式，按照配置的巡逻点导航
   - 如果血量 < 阈值：进入回家模式，导航到基地位置
4. 状态切换时会自动发送新的导航目标

## 使用示例

### 场景1：基本巡逻

配置两个巡逻点，血量低于30%时回家：

```yaml
hp_threshold_ratio: 0.3
home_pose: [0.0, 0.0, 0.0]
patrol_poses: [
  3.0, 2.0, 0.0,
  3.0, -2.0, 3.14
]
use_waypoints: false
```

### 场景2：带途径点的巡逻

先经过途径点，再到达巡逻点：

```yaml
hp_threshold_ratio: 0.4
home_pose: [0.0, 0.0, 0.0]
patrol_poses: [
  5.0, 3.0, 0.0,
  5.0, -3.0, 3.14
]
use_waypoints: true
waypoints: [
  2.0, 0.0, 0.0,
  4.0, 0.0, 0.0
]
```

## 注意事项

1. 确保Nav2导航栈已经正常运行
2. 确保机器人状态话题正常发布
3. 所有坐标都是在map坐标系下
4. yaw角度使用弧度制
5. 建议先在仿真环境中测试配置的点位是否合理

## 依赖

- ROS 2 (Humble或更高版本)
- Nav2
- pb_rm_interfaces
- rclcpp
- geometry_msgs
- nav2_msgs

## 故障排查

### 节点启动但不工作

- 检查是否收到机器人状态信息：`ros2 topic echo /robot_status`
- 检查Nav2是否正常运行：`ros2 action list`

### 导航目标被拒绝

- 确认Nav2导航服务器正在运行
- 检查配置的点位是否在地图范围内
- 确认点位周围没有障碍物

### 血量正常但一直回家

- 检查 `hp_threshold_ratio` 配置是否过高
- 确认机器人状态话题中的血量数据是否正确

## 开发者

- 包名: sentry_decision
- 版本: 1.0.0
- 许可证: Apache-2.0

# sentry_localization

定位层，包含 Point-LIO 源码和 small_gicp 重定位源码，并提供统一的启动入口。

```bash
colcon build --symlink-install --packages-up-to sentry_localization
source install/setup.bash
```

建图：

```bash
ros2 launch sentry_localization mapping.launch.py
```

已有地图重定位：

```bash
ros2 launch sentry_localization localization.launch.py \
  map:=/path/to/map.yaml prior_pcd_file:=/path/to/map.pcd
```

`slam_toolbox`、`nav2_map_server` 和 `nav2_lifecycle_manager` 使用 ROS 2 系统依赖，
不在本目录复制源码。

# sentry_perception

感知层将 Point-LIO 输出的注册点云处理为地形地图和二维障碍物扫描，供
`sentry_localization` 与 Nav2 使用。

```bash
ros2 launch sentry_perception perception.launch.py
```

关键输出：`terrain_map`、`terrain_map_ext`、`obstacle_scan`。

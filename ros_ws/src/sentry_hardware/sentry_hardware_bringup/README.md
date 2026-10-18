# sentry_hardware

Hardware-layer bringup for the sentry robot.

This package provides a stable sensor launch interface. The vendor-maintained
driver remains the independent `livox_ros_driver2` package beside this package.

## Start the Mid-360

```bash
colcon build --symlink-install --packages-up-to sentry_hardware
source install/setup.bash
ros2 launch sentry_hardware lidar.launch.py
```

The default network configuration is `config/MID360_config.json`. Override it
for a different robot or network with `config:=/path/to/config.json`.

#!/usr/bin/env bash
# Stop the sentry ROS 2 stack, including orphaned nodes left behind by a
# previously killed `ros2 launch`.
#
# Why: killing a launch with Ctrl+Z / SIGKILL can leave usb_bridge,
# cmd_vel_to_unified and joint_state_publisher running.  Two bridges then
# publish /unified_cmd with different enable_motion settings, which makes the
# chassis mode alternate between Stop and NoFollow (wheels stutter).
set +e

COMMS=(
  rviz2
  robot_state_pub
  joint_state_pub
  livox_ros_drive
  pointlio_mappin
  sync_slam_toolb
  terrainAnalysis
  pointcloud_to_l
  controller_serv
  planner_server
  bt_navigator
  behavior_server
  velocity_smooth
  smoother_server
  waypoint_follow
  map_saver_serve
  lifecycle_manag
  fake_vel_transf
  usb_bridge
  cmd_vel_to_unif
  # The five below were missing and survived as orphans (2026-10-24): a stop
  # printed "ROS stack stopped" while ten of them were still alive in two
  # generations.  referee_serial_ also kept /dev/ttyACM0 locked, so the next
  # bringup lost the referee serial port.
  joy_node
  pb_teleop_twist
  referee_serial_
  sentry_decision
  static_transfor
)

for name in "${COMMS[@]}"; do
  pkill -x "$name" 2>/dev/null
done

# The launch wrapper itself.
for pid in $(pgrep -f "ros2 launch sentry_bringup" 2>/dev/null); do
  [ "$pid" != "$$" ] && kill "$pid" 2>/dev/null
done

sleep 2

for name in "${COMMS[@]}"; do
  pkill -9 -x "$name" 2>/dev/null
done
for pid in $(pgrep -f "ros2 launch sentry_bringup" 2>/dev/null); do
  [ "$pid" != "$$" ] && kill -9 "$pid" 2>/dev/null
done

sleep 1

# Health check: every name in the list must be gone, not just the two that
# matter most.  The old two-name check is what let the orphaned set above go
# unnoticed.
leftover=()
for name in "${COMMS[@]}"; do
  pgrep -x "$name" >/dev/null 2>&1 && leftover+=("$name")
done
if pgrep -f "ros2 launch sentry_bringup" >/dev/null 2>&1; then
  leftover+=("ros2 launch sentry_bringup")
fi

if [ ${#leftover[@]} -gt 0 ]; then
  echo "warning: still running: ${leftover[*]}"
  echo "these are the pids:"
  for name in "${leftover[@]}"; do
    pgrep -a -f "$name" 2>/dev/null
  done
  exit 1
fi
echo "ROS stack stopped"

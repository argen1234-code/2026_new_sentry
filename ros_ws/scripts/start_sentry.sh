#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f "$workspace/install/local_setup.bash" ]]; then
    printf 'Workspace not built: %s\n' "$workspace" >&2
    exit 1
fi

# Avoid inheriting another workspace's packages from the caller's terminal.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset LD_LIBRARY_PATH PYTHONPATH ROS_PACKAGE_PATH SDF_PATH GAZEBO_MODEL_PATH
export PATH=/usr/local/bin:/usr/bin:/bin
set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "$workspace/install/local_setup.bash"
set -u

exec ros2 launch sentry_bringup sentry.launch.py "$@"

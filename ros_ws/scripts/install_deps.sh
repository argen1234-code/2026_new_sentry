#!/usr/bin/env bash
# Install the system/ROS dependencies that are not present on this image.
# Run with sudo:
#
#   sudo ./scripts/install_deps.sh
#
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "run with sudo: sudo $0" >&2
    exit 1
fi

export DEBIAN_FRONTEND=noninteractive

ROS_DISTRO="${ROS_DISTRO:-humble}"

apt-get update
apt-get install -y --no-install-recommends \
    "ros-${ROS_DISTRO}-pcl-ros" \
    "ros-${ROS_DISTRO}-serial-driver" \
    "ros-${ROS_DISTRO}-asio-cmake-module"

# Optional but useful when debugging serial links or rebuilding Livox.
apt-get install -y --no-install-recommends \
    libapr1-dev \
    libgoogle-glog-dev \
    libgflags-dev

echo
echo "installed. verify:"
for p in pcl_ros serial_driver asio_cmake_module; do
    if [ -d "/opt/ros/${ROS_DISTRO}/share/$p" ]; then
        echo "  OK   $p"
    else
        echo "  MISS $p"
    fi
done

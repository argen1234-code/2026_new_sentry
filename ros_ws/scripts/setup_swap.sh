#!/usr/bin/env bash
# Create a swap file for the low-memory colcon build. Run with sudo:
#
#   sudo ./scripts/setup_swap.sh 6G
#
# The Jetson module has ~8 GiB of unified memory and no swap by default.
# A single PCL/Point-LIO translation unit can exhaust it while the desktop
# keeps running, which freezes the board. Swap gives the memory guard room
# to finish instead of triggering a global OOM freeze.
set -euo pipefail

size="${1:-6G}"
swapfile="${SWAPFILE:-/swapfile}"

if [ "$(id -u)" -ne 0 ]; then
    echo "run with sudo: sudo $0 $size" >&2
    exit 1
fi

if swapon --show=NAME --noheadings | grep -qx "$swapfile"; then
    echo "$swapfile is already active"
    swapon --show
    exit 0
fi

if [ ! -f "$swapfile" ]; then
    # fallocate is fast, but some filesystems dislike it for swap; fall back
    # to dd on eMMC/ext4 if mkswap rejects the file.
    fallocate -l "$size" "$swapfile" || dd if=/dev/zero of="$swapfile" bs=1M count=6144 status=progress
    chmod 600 "$swapfile"
    mkswap "$swapfile"
fi

swapon "$swapfile"
if ! grep -qE "^[[:space:]]*$swapfile[[:space:]]" /etc/fstab; then
    printf '%s none swap sw 0 0\n' "$swapfile" >> /etc/fstab
fi

# Keep the desktop responsive under pressure; the build guard does the
# heavier lifting, this only makes the kernel reclaim earlier.
sysctl -w vm.swappiness=20

echo
swapon --show
free -h

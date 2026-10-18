#!/bin/bash

# CH340 串口设备修复脚本
# 用于解决 CH340 USB 转串口芯片未创建 /dev/ttyUSB 设备的问题

set -e

echo "=========================================="
echo "CH340 串口设备修复脚本"
echo "=========================================="
echo ""

# 检查是否以 root 权限运行
if [ "$EUID" -ne 0 ]; then 
    echo "错误: 此脚本需要 root 权限运行"
    echo "请使用: sudo bash $0"
    exit 1
fi

echo "1. 检查 USB 设备..."
if lsusb | grep -q "1a86:7523"; then
    echo "   ? 找到 CH340 设备 (1a86:7523)"
else
    echo "   ? 未找到 CH340 设备"
    echo "   请检查设备是否已连接"
    exit 1
fi

echo ""
echo "2. 检查 CH341 驱动模块..."
if lsmod | grep -q "ch341"; then
    echo "   ? CH341 驱动已加载"
else
    echo "   ! CH341 驱动未加载，正在加载..."
    modprobe ch341
    sleep 1
    if lsmod | grep -q "ch341"; then
        echo "   ? CH341 驱动加载成功"
    else
        echo "   ? CH341 驱动加载失败"
        exit 1
    fi
fi

echo ""
echo "3. 检查当前串口设备..."
if ls /dev/ttyUSB* 2>/dev/null; then
    echo "   ? 已存在 ttyUSB 设备"
    echo ""
    echo "设备列表:"
    ls -l /dev/ttyUSB*
    echo ""
    echo "如果设备已存在，可能不需要进一步操作"
    read -p "是否继续重新绑定驱动? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "操作已取消"
        exit 0
    fi
fi

echo ""
echo "4. 解绑并重新绑定 USB 设备..."

# 查找设备路径
USB_DEVICE=$(find /sys/bus/usb/devices/ -name "3-*" -type d | while read dev; do
    if [ -f "$dev/idVendor" ] && [ -f "$dev/idProduct" ]; then
        vendor=$(cat "$dev/idVendor")
        product=$(cat "$dev/idProduct")
        if [ "$vendor" = "1a86" ] && [ "$product" = "7523" ]; then
            echo $(basename "$dev")
            break
        fi
    fi
done)

if [ -z "$USB_DEVICE" ]; then
    echo "   ? 无法找到 USB 设备路径"
    exit 1
fi

echo "   找到设备: $USB_DEVICE"

# 解绑设备
echo "   正在解绑设备..."
if [ -f "/sys/bus/usb/drivers/usb/$USB_DEVICE/driver/unbind" ]; then
    echo "$USB_DEVICE" > /sys/bus/usb/drivers/usb/unbind 2>/dev/null || true
fi

sleep 1

# 重新绑定设备
echo "   正在重新绑定设备..."
echo "$USB_DEVICE" > /sys/bus/usb/drivers/usb/bind 2>/dev/null || true

sleep 2

echo ""
echo "5. 验证串口设备..."
if ls /dev/ttyUSB* 2>/dev/null; then
    echo "   ? 串口设备创建成功!"
    echo ""
    echo "可用的串口设备:"
    ls -l /dev/ttyUSB*
    
    # 获取第一个 ttyUSB 设备
    FIRST_TTY=$(ls /dev/ttyUSB* 2>/dev/null | head -1)
    
    echo ""
    echo "=========================================="
    echo "修复完成!"
    echo "=========================================="
    echo ""
    echo "串口设备: $FIRST_TTY"
    echo ""
    echo "下一步操作:"
    echo "1. 如果设备不是 /dev/ttyACM0，需要修改配置文件"
    echo "   配置文件: src/standard_robot_pp_ros2/config/standard_robot_pp_ros2.yaml"
    echo "   将 device_name 改为: $FIRST_TTY"
    echo ""
    echo "2. 或者创建符号链接:"
    echo "   sudo ln -sf $FIRST_TTY /dev/ttyACM0"
    echo ""
    echo "3. 设置设备权限:"
    echo "   sudo chmod 666 $FIRST_TTY"
    echo ""
else
    echo "   ? 串口设备创建失败"
    echo ""
    echo "请尝试以下操作:"
    echo "1. 拔出并重新插入 USB 设备"
    echo "2. 重新运行此脚本"
    echo "3. 检查系统日志: dmesg | tail -20"
    exit 1
fi
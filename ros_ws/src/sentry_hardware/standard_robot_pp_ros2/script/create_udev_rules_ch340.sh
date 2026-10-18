#!/bin/bash

set -e

echo ""
echo "=========================================="
echo "CH340 串口设备 udev 规则配置脚本"
echo "=========================================="
echo ""
echo "此脚本将为 CH340 USB 转串口设备创建 udev 规则"
echo "设备 ID: 1a86:7523 (QinHeng Electronics CH340)"
echo ""

UDEV_RULES_FILE="/etc/udev/rules.d/99-CH340-Serial.rules"

# CH340 设备的 udev 规则
# 这将创建一个符号链接 /dev/ttyACM0 指向实际的 ttyUSB 设备
UDEV_RULE='SUBSYSTEMS=="usb", KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ttyACM0", MODE="0666", GROUP="dialout"'

echo "当前配置:"
echo "  - 设备类型: CH340 USB 转串口"
echo "  - Vendor ID: 1a86"
echo "  - Product ID: 7523"
echo "  - 符号链接: /dev/ttyACM0 -> /dev/ttyUSB*"
echo "  - 权限: 0666 (所有用户可读写)"
echo "  - 用户组: dialout"
echo ""

read -p "是否继续? (y/N): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "操作已取消"
    exit 0
fi

echo ""
echo "1. 设置 udev 规则..."
echo "$UDEV_RULE" | sudo tee $UDEV_RULES_FILE > /dev/null

if [ $? -eq 0 ]; then
    echo "   ? udev 规则已写入: $UDEV_RULES_FILE"
else
    echo "   ? 写入 udev 规则失败"
    exit 1
fi

echo ""
echo "2. 重新加载 udev 规则..."
sudo udevadm control --reload-rules
if [ $? -eq 0 ]; then
    echo "   ? udev 规则已重新加载"
else
    echo "   ? 重新加载 udev 规则失败"
    exit 1
fi

echo ""
echo "3. 触发 udev 规则应用..."
sudo udevadm trigger
sleep 2

CURRENT_USER=$(whoami)

echo ""
echo "4. 检查用户组..."
if groups $CURRENT_USER | grep -q "dialout"; then
    echo "   ? 用户 $CURRENT_USER 已在 dialout 组中"
else
    echo "   ! 将用户 $CURRENT_USER 添加到 dialout 组..."
    sudo usermod -aG dialout $CURRENT_USER
    echo "   ? 用户已添加到 dialout 组"
    echo ""
    echo "   ??  重要: 需要注销并重新登录才能使组权限生效"
fi

echo ""
echo "5. 验证配置..."
if [ -e "/dev/ttyUSB0" ]; then
    echo "   ? 找到设备: /dev/ttyUSB0"
    ls -l /dev/ttyUSB0
    
    if [ -L "/dev/ttyACM0" ]; then
        echo "   ? 符号链接已创建: /dev/ttyACM0"
        ls -l /dev/ttyACM0
    else
        echo "   ! 符号链接尚未创建，请拔出并重新插入 USB 设备"
    fi
else
    echo "   ! 未找到 /dev/ttyUSB0 设备"
    echo "   请拔出并重新插入 USB 设备"
fi

echo ""
echo "=========================================="
echo "配置完成!"
echo "=========================================="
echo ""
echo "下一步操作:"
echo "1. 拔出并重新插入 USB 设备"
echo "2. 检查符号链接: ls -l /dev/ttyACM0"
echo "3. 如果刚添加到 dialout 组，需要注销并重新登录"
echo "4. 运行 ROS2 节点测试"
echo ""
echo "测试命令:"
echo "  ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py"
echo ""
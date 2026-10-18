#!/bin/bash

# SBUS 协议修改应用脚本
# 此脚本会自动修改必要的文件以支持 SBUS 协议

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "=========================================="
echo "SBUS 协议修改应用脚本"
echo "=========================================="
echo ""
echo "项目目录: $PROJECT_DIR"
echo ""

# 检查是否在正确的目录
if [ ! -f "$PROJECT_DIR/package.xml" ]; then
    echo "错误: 未找到 package.xml，请确保在正确的目录运行此脚本"
    exit 1
fi

echo "此脚本将进行以下修改："
echo "1. 修改 CMakeLists.txt 添加 sbus_protocol.cpp"
echo "2. 修改配置文件添加 SBUS 参数"
echo "3. 创建备份文件"
echo ""

read -p "是否继续? (y/N): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "操作已取消"
    exit 0
fi

# 创建备份
BACKUP_DIR="$PROJECT_DIR/backup_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP_DIR"
echo ""
echo "1. 创建备份到: $BACKUP_DIR"

if [ -f "$PROJECT_DIR/CMakeLists.txt" ]; then
    cp "$PROJECT_DIR/CMakeLists.txt" "$BACKUP_DIR/"
    echo "   ? 备份 CMakeLists.txt"
fi

if [ -f "$PROJECT_DIR/config/standard_robot_pp_ros2.yaml" ]; then
    cp "$PROJECT_DIR/config/standard_robot_pp_ros2.yaml" "$BACKUP_DIR/"
    echo "   ? 备份 standard_robot_pp_ros2.yaml"
fi

# 修改 CMakeLists.txt
echo ""
echo "2. 修改 CMakeLists.txt"

if grep -q "src/sbus_protocol.cpp" "$PROJECT_DIR/CMakeLists.txt"; then
    echo "   ! sbus_protocol.cpp 已经在 CMakeLists.txt 中"
else
    # 在 add_library 中添加 sbus_protocol.cpp
    sed -i '/src\/gimbal_manager.cpp/a\  src\/sbus_protocol.cpp' "$PROJECT_DIR/CMakeLists.txt"
    echo "   ? 添加 sbus_protocol.cpp 到 CMakeLists.txt"
fi

# 修改配置文件
echo ""
echo "3. 修改配置文件"

CONFIG_FILE="$PROJECT_DIR/config/standard_robot_pp_ros2.yaml"

if grep -q "use_sbus_protocol" "$CONFIG_FILE"; then
    echo "   ! SBUS 参数已存在于配置文件中"
else
    # 在 baud_rate 后添加 SBUS 相关参数
    cat >> "$CONFIG_FILE" << 'EOF'

    # SBUS 协议配置
    use_sbus_protocol: false  # true=SBUS协议, false=原有协议
    max_linear_velocity: 3.0   # 最大线速度 (m/s)
    max_angular_velocity: 3.14 # 最大角速度 (rad/s)
EOF
    echo "   ? 添加 SBUS 参数到配置文件"
fi

echo ""
echo "=========================================="
echo "修改完成!"
echo "=========================================="
echo ""
echo "下一步操作:"
echo ""
echo "1. 查看修改指南:"
echo "   cat $PROJECT_DIR/docs/SBUS_MODIFICATION_GUIDE.md"
echo ""
echo "2. 手动修改以下文件（参考指南）:"
echo "   - include/standard_robot_pp_ros2/standard_robot_pp_ros2.hpp"
echo "   - src/standard_robot_pp_ros2.cpp"
echo ""
echo "3. 编译项目:"
echo "   cd ~/printk_ws"
echo "   colcon build --packages-select standard_robot_pp_ros2"
echo ""
echo "4. 启用 SBUS 协议:"
echo "   编辑 config/standard_robot_pp_ros2.yaml"
echo "   设置 use_sbus_protocol: true"
echo ""
echo "备份文件位置: $BACKUP_DIR"
echo ""
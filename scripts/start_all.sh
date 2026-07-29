#!/bin/bash
# 全链路启动脚本 — 踝关节外骨骼控制系统
# 用法: bash scripts/start_all.sh [log_dir]
#
# 启动顺序: 传感器 → 足底开关 → 控制节点
# 每个节点用 setsid 隔离，防止终端关闭误杀

set -e

LOG_DIR="${1:-/tmp/ankle_logs}"
mkdir -p "$LOG_DIR"

SOURCE_ROS="/opt/ros/humble/setup.bash"
SOURCE_WS="$HOME/git_/Ankle_Exoskeleton_ustb/install/setup.bash"

if [ ! -f "$SOURCE_ROS" ]; then
    echo "❌ ROS2 Humble 未找到: $SOURCE_ROS"
    exit 1
fi
if [ ! -f "$SOURCE_WS" ]; then
    echo "❌ 工作空间未编译: $SOURCE_WS"
    echo "   请先运行: cd ~/git_/Ankle_Exoskeleton_ustb && colcon build --symlink-install"
    exit 1
fi

source "$SOURCE_ROS"
source "$SOURCE_WS"

echo "============================================"
echo "  踝关节外骨骼 — 全链路启动"
echo "  日志目录: $LOG_DIR"
echo "============================================"

# ── T0: 检查设备 ──
echo ""
echo "[T0] 检查硬件设备..."
for dev in /dev/ankle_encoder /dev/ankle_switch /dev/ankle_force; do
    if [ -e "$dev" ]; then
        echo "  ✅ $dev 存在"
    else
        echo "  ⚠️  $dev 不存在 — 请检查 USB 连接或 udev 规则"
    fi
done
# CAN 设备检查
if lsusb 2>/dev/null | grep -q "04d8:0053"; then
    echo "  ✅ CANalyst-II (04d8:0053) 已连接"
else
    echo "  ⚠️  CANalyst-II 未检测到"
fi

# ── T1: 力传感器 (必须先启，控制节点预紧需要力读数) ──
echo ""
echo "[T1] 启动力传感器节点..."
setsid ros2 run can_ankle serial_ForceSensor_node --ros-args \
    -p port:=/dev/ankle_force \
    -p debug:=false \
    > "$LOG_DIR/force_sensor.log" 2>&1 &
FORCE_PID=$!
echo "  PID=$FORCE_PID"

# ── T2: 编码器 ──
echo "[T2] 启动编码器节点..."
setsid ros2 run can_ankle serial_encoder_node \
    > "$LOG_DIR/encoder.log" 2>&1 &
ENCODER_PID=$!
echo "  PID=$ENCODER_PID"

# ── T3: 足底开关 ──
echo "[T3] 启动足底开关节点..."
setsid ros2 run can_ankle serial_sendCommand_node \
    > "$LOG_DIR/switch.log" 2>&1 &
SWITCH_PID=$!
echo "  PID=$SWITCH_PID"

# ── 等待传感器稳定 ──
echo ""
echo "[等待] 传感器初始化中 (2秒)..."
sleep 2

# ── T4: 控制节点 (核心) ──
echo ""
echo "[T4] 启动控制节点 (含开机预紧)..."
echo "  参数: mode=1 weight=60 force_limit=5.0 max_speed=180"

# 控制节点前台运行 — Ctrl+C 只会停它，传感器继续跑
ros2 run can_ankle can_ankleControl_node --ros-args \
    -p control_mode:=1 \
    -p user_weight:=60.0 \
    -p force_limit:=5.0 \
    -p torque_per_force:=6.0 \
    -p motor_dir:=1.0 \
    -p preload_speed:=10.0 \
    -p preload_force:=0.2 \
    -p preload_timeout:=4.0 \
    -p max_speed:=180.0

# 控制节点退出后的清理
echo ""
echo "控制节点已退出，后台传感器仍在运行。"
echo "停止所有节点: bash scripts/stop_all.sh $FORCE_PID $ENCODER_PID $SWITCH_PID"

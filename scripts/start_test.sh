#!/bin/bash
# 测试启动脚本 — 自动记录 launch 日志和 rosbag 数据包
# 用法: bash scripts/start_test.sh
# 停止: Ctrl+C (日志和数据包自动保存)
#
# 输出位置:
#   data/test_logs/launch_<时间戳>.log   — 终端全部输出
#   data/test_logs/bag_<时间戳>/         — rosbag (/command_topic /switch_state /torque_info /Force /angle)

cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash

TS=$(date +%Y%m%d_%H%M%S)
OUTDIR="data/test_logs"
mkdir -p "$OUTDIR"
LOG="$OUTDIR/launch_${TS}.log"
BAG="$OUTDIR/bag_${TS}"

echo "日志: $LOG"
echo "录包: $BAG"

# 后台录包
ros2 bag record -o "$BAG" /command_topic /switch_state /torque_info /Force /angle > /dev/null 2>&1 &
BAG_PID=$!

# 退出时先停录包 (SIGINT 让 rosbag 正常 flush)
cleanup() {
    kill -INT $BAG_PID 2>/dev/null
    wait $BAG_PID 2>/dev/null
    echo "数据已保存: $LOG , $BAG"
}
trap cleanup EXIT

ros2 launch can_ankle start_node.launch.py 2>&1 | tee "$LOG"

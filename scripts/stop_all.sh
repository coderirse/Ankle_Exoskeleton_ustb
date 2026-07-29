#!/bin/bash
# 停止所有踝关节外骨骼节点

PIDS="$@"

if [ -z "$PIDS" ]; then
    echo "用法: bash scripts/stop_all.sh [PID1 PID2 ...]"
    echo "      或自动查找所有 ankle 相关进程:"
    echo "      bash scripts/stop_all.sh --auto"
    echo ""
    echo "正在自动查找..."
    PIDS=$(ps aux | grep -E 'can_ankle|serial_.*node' | grep -v grep | awk '{print $2}')
    if [ -z "$PIDS" ]; then
        echo "  没有找到运行中的节点。"
        exit 0
    fi
fi

if [ "$1" = "--auto" ]; then
    PIDS=$(ps aux | grep -E 'can_ankle|serial_.*node' | grep -v grep | awk '{print $2}')
fi

echo "停止节点..."
for pid in $PIDS; do
    if kill -0 "$pid" 2>/dev/null; then
        cmd=$(ps -p "$pid" -o comm= 2>/dev/null || echo "?")
        echo "  停止 PID=$pid ($cmd)"
        kill "$pid" 2>/dev/null || true
    fi
done

sleep 0.5

# 强制杀掉残留
LEFTOVER=$(ps aux | grep -E 'can_ankle|serial_.*node' | grep -v grep | awk '{print $2}')
if [ -n "$LEFTOVER" ]; then
    echo "  强制停止残留进程: $LEFTOVER"
    kill -9 $LEFTOVER 2>/dev/null || true
fi

echo "全部停止 ✅"

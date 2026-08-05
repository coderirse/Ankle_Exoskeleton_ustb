#!/usr/bin/env python3
"""分析步态录包: 每个步态周期内 指令→速度→力 的时序关系"""
import sqlite3, struct, sys

import sys
db = sqlite3.connect(sys.argv[1] if len(sys.argv) > 1 else '/tmp/gait_test.db3')
topics = {r[0]: r[1] for r in db.execute("SELECT id, name FROM topics")}
print("topics:", topics, file=sys.stderr)

def parse(topic_name, data):
    # CDR: 4-byte header, then aligned fields
    if topic_name in ('/command_topic', '/switch_state'):  # UInt8
        return data[4]
    if topic_name == '/Force':   # Float32
        return struct.unpack_from('<f', data, 4)[0]
    if topic_name == '/angle':   # Float64
        off = 4 if len(data) == 12 else 8
        return struct.unpack_from('<d', data, off)[0]
    if topic_name == '/torque_info':
        # 5 x float64 then float32; CDR 无头部填充 (总长 48)
        tv, rtv, vv, rv, pdv = struct.unpack_from('<5d', data, 4)
        fst, = struct.unpack_from('<f', data, 44)
        return (tv, vv, fst)

data = {n: [] for n in topics.values()}
for tid, ts, blob in db.execute("SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"):
    name = topics[tid]
    data[name].append((ts / 1e9, parse(name, blob)))

t0 = data['/command_topic'][0][0] if data['/command_topic'] else 0
cmds = [(t - t0, int(v)) for t, v in data['/command_topic']]
tq = [(t - t0, v[0], v[1], v[2]) for t, v in data['/torque_info']]  # t, target_Nm, vel, sensorTorque

print(f"\n指令数: {len(cmds)}, torque_info 样本: {len(tq)}")
print("\n=== 步态指令序列 (t, cmd) ===")
print(' '.join(f"{t:.2f}:{hex(c)}" for t, c in cmds[:40]))

# 找每个 0x43 (蹬地开始), 分析之后 0.6s 内的速度和力
print("\n=== 每个蹬地周期分析 (0x43 后) ===")
for i, (tc, c) in enumerate(cmds):
    if c != 0x43:
        continue
    win = [(t, tgt, vel, st) for t, tgt, vel, st in tq if tc <= t <= tc + 0.7]
    if not win:
        continue
    vmax = max(w[2] for w in win)
    tgt_max = max(w[1] for w in win)
    st_max = max(w[3] for w in win)
    st_min = min(w[3] for w in win)
    # 首次达到 90% 峰值速度的时间
    t_v90 = next((w[0] - tc for w in win if w[2] >= 0.9 * vmax), None) if vmax > 10 else None
    # 速度积分 ≈ 电机转角
    ang = sum(w[2] * 0.005 for w in win)
    print(f"t={tc:.2f}s | 目标扭矩峰值={tgt_max:5.1f}Nm | 速度峰值={vmax:6.1f}°/s "
          f"| 到90%速度延迟={t_v90 if t_v90 else float('nan'):.3f}s "
          f"| SensorTorque范围=[{st_min:.1f},{st_max:.1f}]Nm | 速度积分≈{ang:.0f}°")

# 摆动相分析: 0x44 后 0.8s
print("\n=== 每个摆动相分析 (0x44 后) ===")
for tc, c in cmds:
    if c != 0x44:
        continue
    win = [(t, tgt, vel, st) for t, tgt, vel, st in tq if tc <= t <= tc + 0.8]
    if not win:
        continue
    vmin = min(w[2] for w in win)
    st_max = max(w[3] for w in win)
    ang = sum(w[2] * 0.005 for w in win)
    print(f"t={tc:.2f}s | 放线速度最低={vmin:6.1f}°/s | SensorTorque峰值={st_max:.1f}Nm | 速度积分≈{ang:.0f}°")

# 全局统计
if tq:
    vels = [abs(w[2]) for w in tq]
    sts = [w[3] for w in tq]
    print(f"\n=== 全局 ===")
    print(f"|速度指令| 均值={sum(vels)/len(vels):.1f} 最大={max(vels):.1f} °/s")
    print(f"SensorTorque 最大={max(sts):.1f} Nm (≈{max(sts)/1.18:.1f}kg 线张力)")

#!/usr/bin/env python3
"""单步态周期细粒度追踪: angle/force/velocity 随时间, 验证线松紧与关节运动方向的关系"""
import sqlite3, struct

import sys
db = sqlite3.connect(sys.argv[1] if len(sys.argv) > 1 else '/tmp/gait_test.db3')
topics = {r[0]: r[1] for r in db.execute("SELECT id, name FROM topics")}

def parse(name, data):
    if name in ('/command_topic', '/switch_state'): return data[4]
    if name == '/Force': return struct.unpack_from('<f', data, 4)[0]
    if name == '/angle':
        off = 4 if len(data) == 12 else 8
        return struct.unpack_from('<d', data, off)[0]
    if name == '/torque_info':
        tv, rtv, vv, rv, pdv = struct.unpack_from('<5d', data, 4)
        fst, = struct.unpack_from('<f', data, 44)
        return (tv, vv, fst)

data = {n: [] for n in topics.values()}
for tid, ts, blob in db.execute("SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"):
    data[topics[tid]].append((ts / 1e9, parse(topics[tid], blob)))

# 取一个完整周期: 0x41 且后续依次出现 0x42/0x43/0x44
cmds = data['/command_topic']
t_begin = t_end = None
for i, (t, c) in enumerate(cmds):
    if c != 0x41: continue
    seq = [c2 for _, c2 in cmds[i:i+8]]
    if seq[:4] == [0x41, 0x42, 0x43, 0x44]:
        t_begin = t
        t_end = cmds[i + 4][0] if i + 4 < len(cmds) else t + 1.4
        break
assert t_begin is not None, "没找到完整周期"

print(f"周期窗口: {t_end - t_begin:.2f}s")
marks = [(t - t_begin, c) for t, c in cmds if t_begin <= t <= t_end]
print("指令: " + '  '.join(f"{t:.2f}s={hex(c)}" for t, c in marks))
print(f"\n{'t':>6} {'angle':>8} {'force_kg':>8} {'vel_cmd':>8} {'tgt_Nm':>6}  (每20ms采样)")

ang = data['/angle']; frc = data['/Force']; tq = data['/torque_info']
import bisect
def nearest(series, t):
    ts = [x[0] for x in series]
    i = bisect.bisect_left(ts, t)
    if i >= len(series): i = len(series) - 1
    return series[i][1]

t = t_begin
while t <= t_end:
    a = nearest(ang, t)
    f = nearest(frc, t)
    tv, vv, st = nearest(tq, t)
    bar = '#' * int(max(0, min(f, 20)))
    print(f"{t - t_begin:6.2f} {a:8.2f} {f:8.2f} {vv:8.1f} {tv:6.1f}  {bar}")
    t += 0.02

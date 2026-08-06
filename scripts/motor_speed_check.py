#!/usr/bin/env python3
"""电机速度标定测量 (离线调试工具)

用 SDO 读 6064h 位置反馈, 精确测定 60FF 速度指令的换算比例。
流程: 使能 → 指令 36000 uu 转 5s → 读位置差 → 打印真实转速。
负载端一圈 = 131072 pulse (608F:01)。

用法: python3 scripts/motor_speed_check.py [uu] [时长s]
  默认 36000 uu (= 当前代码假设的 60°/s), 5 秒
注意: 会强杀占用 CAN 的进程, 不可与控制节点同时运行!
"""
import can, time, struct, sys, subprocess, os, re

N = 0x53
UU  = int(sys.argv[1]) if len(sys.argv) > 1 else 36000
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
PULSES_PER_REV = 131072   # 608F:01 负载端位置分辨率

def find_and_kill_can_users():
    try:
        import usb.core
        dev = usb.core.find(idVendor=0x04d8, idProduct=0x0053)
        if not dev: return
        path = f"/dev/bus/usb/{dev.bus:03d}/{dev.address:03d}"
        out = subprocess.run(["fuser", path], capture_output=True, text=True).stdout
        for pid in re.findall(r'\d+', out):
            os.kill(int(pid), 9)
        time.sleep(0.5)
    except Exception:
        pass

bus = None
def sdo_read(idx, sub, retries=4):
    for _ in range(retries):
        # 清空接收缓冲, 避免读到旧帧
        while bus.recv(0) is not None:
            pass
        bus.send(can.Message(arbitration_id=0x600+N, data=[0x40, idx&0xFF, idx>>8, sub, 0,0,0,0], is_extended_id=False))
        t0 = time.time()
        while time.time() - t0 < 0.3:
            msg = bus.recv(0.05)
            if msg and msg.arbitration_id == 0x580+N:
                d = list(msg.data)
                # 校验回显的索引, 不匹配说明是旧帧/错帧, 继续等
                if d[1] != (idx & 0xFF) or d[2] != (idx >> 8) or d[3] != sub:
                    continue
                nbytes = {0x43:4, 0x4B:2, 0x4F:1}.get(d[0], 4)
                return int.from_bytes(bytes(d[4:4+nbytes]), 'little', signed=True)
        time.sleep(0.1)
    return None

def sdo_write(idx, sub, val, nbytes=4):
    cmd = {1:0x2F, 2:0x2B, 4:0x23}[nbytes]
    payload = list(struct.pack('<i', val))[:nbytes]
    bus.send(can.Message(arbitration_id=0x600+N,
        data=[cmd, idx&0xFF, idx>>8, sub] + payload + [0]*(4-nbytes), is_extended_id=False))

find_and_kill_can_users()
bus = can.interface.Bus(interface='canalystii', channel=0, bitrate=1000000)

# 使能 + 关斜坡
bus.send(can.Message(arbitration_id=0x000, data=[0x80, N], is_extended_id=False)); time.sleep(0.2)
sdo_write(0x6060, 0, 3, 1)
sdo_write(0x6083, 0, 0x3FFFFFFF); time.sleep(0.05)
sdo_write(0x6084, 0, 0x3FFFFFFF); time.sleep(0.05)
sdo_write(0x60FF, 0, 0)
sdo_write(0x6040, 0, 6, 2); time.sleep(0.1)
sdo_write(0x6040, 0, 7, 2); time.sleep(0.1)
sdo_write(0x6040, 0, 0xF, 2); time.sleep(0.1)
print(f"6083 读回 = {sdo_read(0x6083, 0)}")

p0 = sdo_read(0x6064, 0)
print(f"起始位置 = {p0}")
print(f">>> 指令 {UU} uu, 转 {DUR}s ...")
sdo_write(0x60FF, 0, UU)
t0 = time.time()
time.sleep(DUR)
sdo_write(0x60FF, 0, 0)
dt = time.time() - t0
time.sleep(0.3)
p1 = sdo_read(0x6064, 0)
print(f"结束位置 = {p1}")

if p0 is not None and p1 is not None:
    dp = p1 - p0
    revs = dp / PULSES_PER_REV
    deg = revs * 360.0
    deg_s = deg / dt
    print(f"\nΔ位置 = {dp} pulse = {revs:.3f} 圈 = {deg:.1f}°")
    print(f"真实负载速度 = {deg_s:.1f} °/s  ({deg_s/6:.1f} rpm)")
    print(f"指令 {UU} uu → 真实 {deg_s:.1f} °/s  →  换算比例 1 uu = {deg_s/UU*1000:.4f} ×10⁻³ °/s")
    print(f"当前代码假设: 600 uu = 1 °/s; 实测: {UU/deg_s:.1f} uu = 1 °/s")

sdo_write(0x6040, 0, 6, 2)
bus.shutdown()

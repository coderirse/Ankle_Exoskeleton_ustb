#!/usr/bin/env python3
"""脚底开关数据监控"""
import serial, sys

CMD = {0x41:'后跟着地', 0x42:'全掌接触', 0x43:'后跟离地(助力!)', 0x44:'脚尖离地'}

# 扫描找到开关端口
PORT = None
for p in ['/dev/ttyUSB2','/dev/ttyUSB1','/dev/ttyUSB0','/dev/ttyACM0']:
    try:
        s = serial.Serial(p, 9600, timeout=0.15)
        d = s.read(50)
        s.close()
        sw = sum(1 for b in d if b in CMD)
        noise = len(d) - sw
        if noise == 0:
            PORT = p
            print(f'找到开关: {p} ({len(d)} 字节)', flush=True)
            break
        elif sw > 0 and noise < 3:
            PORT = p
            print(f'找到开关: {p} ({sw}条指令)', flush=True)
            break
    except Exception as e:
        pass

if not PORT:
    print('未找到开关端口!')
    print('请检查: 1)ST-LINK供电 2)拔插ST-LINK复位 3)USB-TTL接线')
    sys.exit(1)

print(f'监听 {PORT} — 踩开关! Ctrl+C 退出\n', flush=True)
ser = serial.Serial(PORT, 9600, timeout=0.5)
try:
    while True:
        b = ser.read(1)
        if b and b[0] in CMD:
            print(f'0x{b[0]:02X}  {CMD[b[0]]}', flush=True)
except KeyboardInterrupt:
    print('停止')
    ser.close()

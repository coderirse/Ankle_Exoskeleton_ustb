#!/usr/bin/env python3
"""电机遥控器 — 键盘控制正反转、调速、使能/停止"""
import can, time, struct, sys, tty, termios, select

N = 0x53; STX = 0x653
speed = 0; enabled = False
bus = None

def w(idx, sub, data):
    sz = len(data); cmd = {1: 0x2F, 2: 0x2B, 4: 0x23}[sz]
    bus.send(can.Message(arbitration_id=STX,
        data=[cmd, idx & 0xFF, idx >> 8, sub] + list(data) + [0] * (4 - sz),
        is_extended_id=False))

def enable():
    global enabled
    bus.send(can.Message(arbitration_id=0x000, data=[0x80, N], is_extended_id=False))
    time.sleep(0.2)
    w(0x6060, 0, bytes([3]))
    w(0x6083, 0, struct.pack('<I', 50000))
    w(0x6084, 0, struct.pack('<I', 50000))
    w(0x60FF, 0, struct.pack('<i', 0))
    w(0x6040, 0, bytes([6, 0])); time.sleep(0.08)
    w(0x6040, 0, bytes([7, 0])); time.sleep(0.08)
    w(0x6040, 0, bytes([0xF, 0])); time.sleep(0.08)
    enabled = True
    print("✅ 电机已使能")

def disable():
    global enabled
    w(0x60FF, 0, struct.pack('<i', 0))
    time.sleep(0.1)
    w(0x6040, 0, bytes([6, 0]))
    enabled = False
    print("⏹ 电机已停止")

def set_speed(s):
    uu = int(s * 600)
    w(0x60FF, 0, struct.pack('<i', uu))

def getch():
    """非阻塞读单字符"""
    if select.select([sys.stdin], [], [], 0.1)[0]:
        return sys.stdin.read(1)
    return None

# ── 主程序 ──
bus = can.interface.Bus(interface='canalystii', channel=0, bitrate=1000000)
fd = sys.stdin.fileno()
old = termios.tcgetattr(fd)
tty.setcbreak(fd)

print("""
╔══════════════════════════════════╗
║        🎮 电机遥控器           ║
╠══════════════════════════════════╣
║  W / ↑     加速正转            ║
║  S / ↓     减速 / 反转         ║
║  A / ←     减速                ║
║  D / →     加速                ║
║  SPACE     紧急停转 (速度归零) ║
║  E         使能电机            ║
║  Q         停止(disable)       ║
║  R         反转方向            ║
║  1-5       预设速度档位        ║
║  X         退出                ║
╚══════════════════════════════════╝
""")

try:
    while True:
        c = getch()
        if not c:
            continue

        if c.lower() == 'e':
            enable()
        elif c.lower() == 'q':
            disable()
            speed = 0
        elif c.lower() == 'x':
            break
        elif c.lower() == 'r':
            speed = -speed
            if enabled:
                set_speed(speed)
                print(f"🔄 反转: {speed:+d} deg/s")
        elif c in ('w', 'W', '\x1b[A'):   # W or ↑
            speed += 10
            if enabled: set_speed(speed)
            print(f"↑ 速度: {speed:+d} deg/s")
        elif c in ('s', 'S', '\x1b[B'):   # S or ↓
            speed -= 10
            if enabled: set_speed(speed)
            print(f"↓ 速度: {speed:+d} deg/s")
        elif c in ('a', 'A', '\x1b[D'):   # A or ←
            if speed > 0: speed = max(0, speed - 5)
            else: speed = min(0, speed + 5)
            if enabled: set_speed(speed)
            print(f"← 速度: {speed:+d} deg/s")
        elif c in ('d', 'D', '\x1b[C'):   # D or →
            if speed >= 0: speed += 5
            else: speed = min(0, speed + 5)
            if enabled: set_speed(speed)
            print(f"→ 速度: {speed:+d} deg/s")
        elif c == ' ':
            speed = 0
            if enabled: set_speed(0)
            print("⏸ 急停!")
        elif c in '12345':
            presets = {1: 10, 2: 20, 3: 30, 4: 40, 5: 50}
            speed = presets[int(c)]
            if enabled: set_speed(speed)
            print(f"🎯 档位 {c}: {speed} deg/s")

        # 持续发送速度（防止电机超时）
        if enabled and c:
            pass  # 速度已在按键时发送

except KeyboardInterrupt:
    pass
finally:
    if enabled:
        disable()
    bus.shutdown()
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
    print("Bye!")

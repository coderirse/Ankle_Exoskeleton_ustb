#!/usr/bin/env python3
"""电机遥控器 — 键盘控制正反转、调速、使能/停止"""
import can, time, struct, sys, tty, termios, select, subprocess, os, re

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

def _find_can_device():
    """找到 CANalyst-II 的 USB 设备路径"""
    try:
        import usb.core
        dev = usb.core.find(idVendor=0x04d8, idProduct=0x0053)
        if dev:
            return f"/dev/bus/usb/{dev.bus:03d}/{dev.address:03d}"
    except Exception:
        pass
    # 回退：用已知模式扫描
    for d in os.listdir("/dev/bus/usb") if os.path.exists("/dev/bus/usb") else []:
        bus_dir = f"/dev/bus/usb/{d}"
        if os.path.isdir(bus_dir):
            for dev_file in os.listdir(bus_dir):
                path = f"{bus_dir}/{dev_file}"
                try:
                    out = subprocess.check_output(
                        ["udevadm", "info", "-q", "property", "-n", path],
                        timeout=2, stderr=subprocess.DEVNULL
                    ).decode()
                    if "04d8" in out and "0053" in out:
                        return path
                except Exception:
                    pass
    return None


def _kill_can_occupants():
    """找到并杀掉占用 CANalyst-II 的进程"""
    dev_path = _find_can_device()
    if not dev_path:
        return False
    try:
        out = subprocess.check_output(
            ["fuser", dev_path], timeout=3, stderr=subprocess.STDOUT
        ).decode().strip()
    except subprocess.CalledProcessError as e:
        out = e.output.decode() if e.output else ""
    except Exception:
        return False

    pids = re.findall(r'\d+', out)
    if not pids:
        return False

    killed = []
    for pid in pids:
        try:
            cmdline = open(f"/proc/{pid}/cmdline").read().replace("\0", " ").strip()
        except Exception:
            cmdline = "?"
        print(f"🔪 杀死占用进程 PID={pid} ({cmdline})")
        try:
            os.kill(int(pid), 9)
            killed.append(pid)
        except Exception:
            pass

    return len(killed) > 0


def _open_can_bus(retries=3):
    """打开 CAN 总线，遇到占用自动杀进程重试"""
    for attempt in range(retries):
        try:
            return can.interface.Bus(interface='canalystii', channel=0, bitrate=1000000)
        except can.exceptions.CanInterfaceNotImplementedError:
            raise  # 驱动没装，重试没用
        except Exception as e:
            err_msg = str(e)
            if "Access denied" in err_msg or "insufficient permissions" in err_msg:
                print("❌ 权限不足！请用 sudo 运行此脚本")
                print("   sudo /home/ler/.local/bin/python3.11 .../motor_remote.py")
                sys.exit(1)
            if "Resource busy" in err_msg or "busy" in err_msg.lower():
                if attempt < retries - 1:
                    print(f"⚠️  CAN 设备被占用，尝试清理... ({attempt+1}/{retries})")
                    if _kill_can_occupants():
                        import usb.core
                        try:
                            dev = usb.core.find(idVendor=0x04d8, idProduct=0x0053)
                            if dev:
                                usb.util.dispose_resources(dev)
                        except Exception:
                            pass
                        time.sleep(0.5)
                        continue
                    print("❌ 无法释放设备，请手动检查")
                raise
            raise
    return None


# ── 主程序 ──
bus = _open_can_bus()

if not sys.stdin.isatty():
    print("❌ 此程序需要在真实终端中运行（不能通过管道重定向 stdin）")
    print("   请使用: sudo /home/ler/.local/bin/python3.11 .../motor_remote.py")
    bus.shutdown()
    sys.exit(1)

fd = sys.stdin.fileno()
old = termios.tcgetattr(fd)
tty.setcbreak(fd)

print("""
╔══════════════════════════════════╗
║        🎮 电机遥控器           ║
╠══════════════════════════════════╣
║  W / ↑     加速正转            ║
║  S / ↓     减速 / 反转         ║
║  SPACE     急停 (速度归零)     ║
║  E         使能电机            ║
║  Q         停止(disable)       ║
║  R         反转方向            ║
║  1-5       预设速度档位        ║
║                              ║
║  J         点动正转 (按住转)   ║
║  K         点动反转 (按住转)   ║
║  松开=停                       ║
║                              ║
║  X         退出                ║
╚══════════════════════════════════╝
""")

jog_speed = 0  # 点动状态: 0=停, 1=正转, -1=反转
jog_timer = 0

try:
    while True:
        c = getch()
        now = time.time()

        # ── 点动模式：按住J/K转，松开200ms后自动停 ──
        if c and c.lower() == 'j' and enabled:
            jog_speed = 30  # 点动正转 30 deg/s
            jog_timer = now
            set_speed(jog_speed)
            continue
        elif c and c.lower() == 'k' and enabled:
            jog_speed = -30  # 点动反转 30 deg/s
            jog_timer = now
            set_speed(jog_speed)
            continue

        # 200ms 没按点动键 → 自动停
        if jog_speed != 0 and now - jog_timer > 0.2:
            jog_speed = 0
            if enabled:
                set_speed(0)
                print("⏸ 松开停止")

        if not c:
            continue

        if c.lower() == 'e':
            enable()
        elif c.lower() == 'q':
            disable()
            speed = 0; jog_speed = 0
        elif c.lower() == 'x':
            break
        elif c.lower() == 'r':
            speed = -speed
            if enabled: set_speed(speed)
            print(f"🔄 反转: {speed:+d} deg/s")
        elif c in ('w', 'W', '\x1b[A'):
            speed += 10
            if enabled: set_speed(speed)
            print(f"↑ {speed:+d} deg/s")
        elif c in ('s', 'S', '\x1b[B'):
            speed -= 10
            if enabled: set_speed(speed)
            print(f"↓ {speed:+d} deg/s")
        elif c == ' ':
            speed = 0; jog_speed = 0
            if enabled: set_speed(0)
            print("⏸ 急停!")
        elif c in '12345':
            presets = {1: 10, 2: 20, 3: 30, 4: 40, 5: 50}
            speed = presets[int(c)]
            if enabled: set_speed(speed)
            print(f"🎯 档位 {c}: {speed} deg/s")

        # 有按键但非点动键 → 取消点动
        if jog_speed != 0 and c.lower() not in ('j', 'k'):
            jog_speed = 0

except KeyboardInterrupt:
    pass
finally:
    if enabled:
        disable()
    if bus:
        bus.shutdown()
    # 释放 USB 设备，防止残留占用
    try:
        import usb.core
        dev = usb.core.find(idVendor=0x04d8, idProduct=0x0053)
        if dev:
            usb.util.dispose_resources(dev)
            print("🔌 USB 设备已释放")
    except Exception:
        pass
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
    print("Bye!")

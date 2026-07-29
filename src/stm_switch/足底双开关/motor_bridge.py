#!/usr/bin/env python3
"""
STM32 串口 ↔ USB-CAN 电机桥接

支持 Windows (ControlCAN.dll) 和 Linux/WSL (libcontrolcan.so)
自动检测 STM32 串口，无需手动配置

用法: python motor_bridge.py
"""

import serial
import time
import threading
import ctypes
import sys
import os
from ctypes import c_ubyte, c_uint, c_ulong

# ============================================================
# CAN 适配器配置 (卓誉 U-CAN 串口型)
# ============================================================
SEND_ID = 0x00000003         # 修正为 ID 3 (对应反馈 ID 1D3)
CAN_ADAPTER_PORT = 'COM15'   # 卓誉 U-CAN 所在的串口号
CAN_BAUD_RATE = 1000000       # 卓誉适配器串口波特率 (1000k)

# ============================================================
# 串口自动检测
# ============================================================
def find_stm32_port():
    """扫描可用串口，找 STM32 (115200bps VOFA 数据)"""
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    
    for p in ports:
        if p.device == CAN_ADAPTER_PORT: continue # 跳过 CAN 适配器
        try:
            s = serial.Serial(p.device, 115200, timeout=0.3)
            line = s.readline().decode(errors='ignore')
            s.close()
            if line.count(',') >= 3:
                print(f"  [FOUND] STM32 → {p.device}")
                return p.device
        except:
            continue
    return None

# ============================================================
# 卓誉 U-CAN 协议封装
# ============================================================
class CanDevice:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.ser = None

    def open(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            print(f"[CAN] 卓誉适配器 {self.port} 已打开")
            time.sleep(0.5)
            self.enable_motor()
        except Exception as e:
            raise RuntimeError(f"无法打开 CAN 适配器串口 {self.port}: {e}")

    def enable_motor(self):
        """发送使能报文: 00 00 00 00 00 00 00 01"""
        print(f"[CAN] 正在使能电机 (ID:{SEND_ID})...")
        enable_data = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01])
        self.send(enable_data)

    def disable_motor(self):
        """发送停机报文: 00 00 00 00 00 00 00 00"""
        print(f"[CAN] 正在停止电机 (ID:{SEND_ID})...")
        disable_data = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        self.send(disable_data)

    def send(self, data_bytes):
        """发送 CAN 帧 (自动封装协议)"""
        if not self.ser: return False
        
        # 卓誉 13 字节协议: AA 55 [ID_L] [ID_H] [DLC] [D0-D7] [SUM]
        can_id = SEND_ID & 0xFFFF
        data = list(data_bytes)
        if len(data) < 8: data += [0] * (8 - len(data))
        
        pkt = [0xAA, 0x55, (can_id & 0xFF), (can_id >> 8 & 0xFF), 0x08] + data
        checksum = sum(pkt) & 0xFF
        pkt.append(checksum)
        
        # print(f"  [CAN TX] {bytes(pkt).hex().upper()}") # 调试用
        self.ser.write(bytes(pkt))
        return True

    def recv(self):
        """接收逻辑暂时保持简化，优先保证发送"""
        if not self.ser or self.ser.in_waiting < 5:
            return []
        # 卓誉反馈解析...
        return []

    def close(self):
        if self.ser:
            self.ser.close()
            print("[CAN] 已关闭")

# ============================================================
def can_recv_thread(ser, can_dev):
    """CAN → STM32 反馈回传"""
    print("[RX] CAN 接收启动")
    last = 0
    while True:
        frames = can_dev.recv() # 修改为无参数调用
        now = time.time()
        if now - last < 0.05:
            continue
        for f in frames:
            if f.ID == 0x000001D3 and f.DataLen >= 8:
                raw_t = f.Data[2] | (f.Data[3] << 8)
                torque = raw_t * 40.0 / 4096.0
                ser.write(f"T{torque:.2f}\n".encode())
                last = now
            elif f.ID == 0x000003D3 and f.DataLen >= 5:
                raw_v = f.Data[1] | (f.Data[2] << 8) | \
                        (f.Data[3] << 16) | (f.Data[4] << 24)
                if raw_v & 0x80000000:
                    raw_v -= 0x100000000
                rpm = (0.6 / 0x00040000) * raw_v
                ser.write(f"S{rpm * 6.0:.2f}\n".encode())
                last = now

# ============================================================
def main():
    print("=" * 50)
    print("  STM32 串口 ↔ USB-CAN 电机桥接")
    print("=" * 50)

    # 1. 找 STM32
    print("[SCAN] 扫描 STM32 串口...")
    port = find_stm32_port()
    if not port:
        port = input("未检测到 STM32, 请输入串口号 (如 COM15 或 /dev/ttyUSB2): ").strip()
    ser = serial.Serial(port, 115200, timeout=0.1)
    print(f"[SERIAL] {port} 已连接")

    # 2. 打开 CAN
    can_dev = CanDevice(CAN_ADAPTER_PORT, CAN_BAUD_RATE)
    can_dev.open()

    # 3. 接收线程
    threading.Thread(target=can_recv_thread, args=(ser, can_dev), daemon=True).start()

    print("\n[OK] 桥接就绪. 按下开关触发步态...")
    print(f"     {port} → STM32 → CAN → 电机")
    print("     按 Ctrl+C 退出\n")

    try:
        while True:
            line = ser.readline().decode(errors='ignore').strip()
            if line:
                if not line.startswith('CAN_T:'):
                    print(f"[DEBUG] STM32 Data: {line}") # 打印波形数据或状态
                else:
                    print(f"[CONTROL] Recv CMD: {line}") # 打印控制指令
                    hex_str = line[6:].replace(' ', '')
                    try:
                        data = bytes.fromhex(hex_str)
                        can_dev.send(data)
                    except ValueError:
                        pass
    except KeyboardInterrupt:
        print("\n[EXIT]")
    finally:
        can_dev.close()
        ser.close()

if __name__ == '__main__':
    main()

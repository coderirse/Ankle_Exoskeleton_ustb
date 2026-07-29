#!/usr/bin/env python3
import serial
import time
import sys

# ============================================================
# 配置 (根据实际情况修改)
# ============================================================
CAN_ADAPTER_PORT = 'COM15'   # 卓誉适配器串口
MOTOR_ID = 3                 # 电机 CAN ID
BAUD_RATE = 1000000           # 卓誉适配器串口波特率 (1000k)

class RoboCTMotorControl:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        print(f"[INIT] 已连接适配器 {port}")

    def _send_can_frame(self, can_id, data):
        """卓誉 U-CAN 13字节协议封装: AA 55 ID_L ID_H DLC D0-D7 SUM"""
        if len(data) < 8:
            data = list(data) + [0] * (8 - len(data))
        
        pkt = [0xAA, 0x55, (can_id & 0xFF), (can_id >> 8 & 0xFF), 0x08] + list(data)
        checksum = sum(pkt) & 0xFF
        pkt.append(checksum)
        
        self.ser.write(bytes(pkt))
        # print(f"[TX] {bytes(pkt).hex().upper()}")

    def enable(self):
        """使能电机"""
        print(f"[CMD] 使能电机 ID:{MOTOR_ID}")
        # RoboCT 常用使能命令 (取决于具体固件，通常是最后一位为 1)
        self._send_can_frame(MOTOR_ID, [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01])

    def disable(self):
        """关断电机"""
        print(f"[CMD] 关断电机 ID:{MOTOR_ID}")
        self._send_can_frame(MOTOR_ID, [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    def set_speed(self, rpm):
        """设置电机速度 (RPM)"""
        # 简化版速度指令封装 (RoboCT 常用格式，具体需参考手册)
        # 这里假设使用的是一种简单的速度控制协议
        print(f"[CMD] 设置速度: {rpm} RPM")
        
        # 将速度转为 32 位整型 (小端)
        speed_raw = int(rpm * 100) # 假设单位是 0.01rpm
        s0 = speed_raw & 0xFF
        s1 = (speed_raw >> 8) & 0xFF
        s2 = (speed_raw >> 16) & 0xFF
        s3 = (speed_raw >> 24) & 0xFF
        
        # 示例指令：0x02 为速度模式标识 (需根据实际协议调整)
        self._send_can_frame(MOTOR_ID, [0x02, s0, s1, s2, s3, 0x00, 0x00, 0x00])

def main():
    try:
        motor = RoboCTMotorControl(CAN_ADAPTER_PORT, BAUD_RATE)
        
        print("\n" + "="*40)
        print("  卓誉电机直接控制测试工具")
        print("="*40)
        print(" [E] 使能电机")
        print(" [D] 关断电机")
        print(" [1] 慢速转动 (100 RPM)")
        print(" [2] 中速转动 (500 RPM)")
        print(" [0] 停止转动 (速度设为 0)")
        print(" [Q] 退出程序")
        print("="*40)

        while True:
            key = input("\n请输入指令: ").upper()
            
            if key == 'E':
                motor.enable()
            elif key == 'D':
                motor.disable()
            elif key == '1':
                motor.set_speed(100)
            elif key == '2':
                motor.set_speed(500)
            elif key == '0':
                motor.set_speed(0)
            elif key == 'Q':
                motor.disable()
                break
            else:
                print("无效指令")
                
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        print("[EXIT] 程序结束")

if __name__ == "__main__":
    main()

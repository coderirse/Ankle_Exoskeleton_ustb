import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Float64
import serial, time

class SwitchNode(Node):
    def __init__(self):
        super().__init__('switch_node')
        self.ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=0.1)  # 2026-07-29 实测: 开关在ttyUSB1
        self.ser.reset_input_buffer()  # 丢弃启动前缓冲区残留指令, 防止误触发电机
        self.cmd_pub = self.create_publisher(UInt8, '/command_topic', 10)
        self.one_pub = self.create_publisher(Float64, '/one_support_time', 10)
        self.two_pub = self.create_publisher(Float64, '/two_support_time', 10)
        self.three_pub = self.create_publisher(Float64, '/three_support_time', 10)
        self.swing_pub = self.create_publisher(Float64, '/swing_time', 10)
        self.last_cmd = 0; self.phase_t0 = time.time(); self.support_t0 = time.time()
        self.timer = self.create_timer(0.005, self.check)
        self.get_logger().info('Switch READY')
    def check(self):
        try:
            b = self.ser.read(1)
            if not b: return
            v = b[0]
            if v not in (0x41,0x42,0x43,0x44,0x45): return
            now = time.time(); n = {0x41:'HEEL',0x42:'FULL',0x43:'DRIVE',0x44:'SWING',0x45:'STOP'}[v]
            if v == 0x41: self.support_t0 = now; self.last_cmd = 0x41
            elif v == 0x42 and self.last_cmd == 0x41: self.one_pub.publish(Float64(data=now - self.phase_t0)); self.last_cmd = 0x42
            elif v == 0x43 and self.last_cmd == 0x42: self.two_pub.publish(Float64(data=now - self.phase_t0)); self.last_cmd = 0x43
            elif v == 0x44 and self.last_cmd == 0x43: self.three_pub.publish(Float64(data=now - self.phase_t0)); self.swing_pub.publish(Float64(data=now - self.support_t0)); self.last_cmd = 0x44
            else: return
            self.phase_t0 = now; self.cmd_pub.publish(UInt8(data=v))
            self.get_logger().info(f'[{v:02X}] {n}')
        except Exception as e: self.get_logger().error(str(e))
def main():
    rclpy.init(); node = SwitchNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()

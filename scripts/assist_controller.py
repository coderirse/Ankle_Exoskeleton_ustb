import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Float32, Float64
import socket, time, math, struct, threading

class AssistNode(Node):
    def __init__(self):
        super().__init__('assist')
        # 可调参数
        self.declare_parameter('force_limit', 2.0)     # 力限值(/Force尺度, 手绷紧≈0.3), 超限立即停电机
        self.declare_parameter('motor_dir', 1.0)       # 电机方向: 1收线拉紧 -1放线 (2026-07-29带线实测)
        self.declare_parameter('speed_gain', 20.0)     # DRIVE段 扭矩→速度增益 (3.0/10.0都太慢)
        self.declare_parameter('max_speed', 400.0)     # DRIVE段速度限幅 deg/s
        self.declare_parameter('swing_gain', 3.0)      # SWING归位 位置误差增益
        self.declare_parameter('swing_max', 60.0)      # SWING归位速度限幅 deg/s
        self.declare_parameter('preload_speed', 10.0)  # 预紧爬行速度 deg/s (30太快, 张力过冲到2.1)
        self.declare_parameter('preload_force', 0.2)   # 预紧目标力(鲍登线绷直判据, 松弛噪声±0.1)
        self.declare_parameter('preload_timeout', 4.0) # 预紧超时 s
        self.force_limit = self.get_parameter('force_limit').value
        self.motor_dir = self.get_parameter('motor_dir').value
        self.speed_gain = self.get_parameter('speed_gain').value
        self.max_speed = self.get_parameter('max_speed').value
        self.swing_gain = self.get_parameter('swing_gain').value
        self.swing_max = self.get_parameter('swing_max').value
        self.preload_speed = self.get_parameter('preload_speed').value
        self.preload_force = self.get_parameter('preload_force').value
        self.preload_timeout = self.get_parameter('preload_timeout').value
        self.force_alarm = False
        self.sock = socket.socket(); self.sock.connect(('127.0.0.1', 9876))
        self.lock = threading.Lock()
        def sdo(idx, sub, data):
            sz=len(data); cmd={1:0x2F,2:0x2B,4:0x23}[sz]
            self._tx(f"TX 00000653 8 {cmd:02X} {idx&0xFF:02X} {idx>>8:02X} {sub:02X} " + ' '.join(f'{b:02X}' for b in data) + ' 00'*(4-sz))
        self._tx('TX 00000000 2 80 53'); time.sleep(0.2)
        sdo(0x6060,0,bytes([3])); time.sleep(0.05)
        sdo(0x6040,0,bytes([6,0])); time.sleep(0.08)
        sdo(0x6040,0,bytes([7,0])); time.sleep(0.08)
        sdo(0x6040,0,bytes([0xF,0])); time.sleep(0.08)
        self.get_logger().info('Motor enabled')
        self.mode = 'PRELOAD'; self.encoder = 0.0; self.force = 0.0  # 开机先做一次预紧标定
        self.t0 = time.time(); self.last_cmd = 0; self.initial_encoder = 0.0
        self.peak = 18.0; self.rise = 0.3; self.fall = 0.3; self.duration = 0.6
        self.cmd_sub = self.create_subscription(UInt8, '/command_topic', self.on_switch, 10)
        self.enc_sub = self.create_subscription(Float64, '/angle', self.on_enc, 10)
        self.force_sub = self.create_subscription(Float32, '/Force', self.on_force, 10)
        self.torque_pub = self.create_publisher(Float64, '/target_torque', 10)
        self.speed_pub = self.create_publisher(Float64, '/motor_speed', 10)
        self.force_pub = self.create_publisher(Float64, '/force_scaled', 10)
        self.mode_pub = self.create_publisher(UInt8, '/gait_mode', 10)
        self.timer = self.create_timer(0.005, self.control)
        self.get_logger().info('READY')
    def _tx(self, s):
        with self.lock: self.sock.sendall((s+'\n').encode()); return self.sock.recv(256).decode().strip()
    def _set_speed(self, deg_s):
        uu = int(deg_s * self.motor_dir * 600.0)
        d = struct.pack('<i', uu)
        self._tx(f"TX 00000653 8 23 FF 60 00 {d[0]:02X} {d[1]:02X} {d[2]:02X} {d[3]:02X}")
    def on_switch(self, msg):
        cmd = msg.data
        if cmd == 0x41: self.mode = 'TORQUE'; self.t0 = time.time(); self.initial_encoder = self.encoder
        elif cmd == 0x42 and self.last_cmd == 0x41: self.mode = 'PRE_TORQUE'; self.t0 = time.time()
        elif cmd == 0x43 and self.last_cmd == 0x42: self.mode = 'DRIVE'; self.t0 = time.time(); self.get_logger().info(f'DRIVE peak={self.peak}Nm')
        elif cmd == 0x44 and self.last_cmd == 0x43: self.mode = 'SWING'; self.t0 = time.time()
        elif cmd == 0x45: self.mode = 'STAND'
        self.last_cmd = cmd if cmd in (0x41,0x42,0x43,0x44) else self.last_cmd
    def on_enc(self, msg): self.encoder = msg.data
    def on_force(self, msg): self.force = msg.data
    def control(self):
        # 力保护: 鲍登线拉力超限 → 立即停电机回STAND
        if abs(self.force) >= self.force_limit:
            if not self.force_alarm:
                self.force_alarm = True
                self.get_logger().warn(f'力超限! {self.force:.1f} >= {self.force_limit}, 电机停止')
            self._set_speed(0); self.mode = 'STAND'
            self.torque_pub.publish(Float64(data=0.0))
            self.force_pub.publish(Float64(data=float(self.force * 1000.0)))
            self.mode_pub.publish(UInt8(data=0))
            return
        elif self.force_alarm:
            self.force_alarm = False
            self.get_logger().info('力恢复正常')
        t = time.time() - self.t0
        if self.mode == 'PRELOAD':
            # 预紧: 慢速收线直到力传感器出现微小读数(鲍登线绷直)
            if abs(self.force) >= self.preload_force:
                self._set_speed(0); self.mode = 'TORQUE'
                self.get_logger().info(f'预紧完成 force={self.force:.2f}')
            elif t > self.preload_timeout:
                self._set_speed(0); self.mode = 'TORQUE'
                self.get_logger().warn(f'预紧超时 force={self.force:.2f}, 检查鲍登线')
            else:
                self._set_speed(self.preload_speed)
        elif self.mode == 'DRIVE':
            if t < self.rise: torque = (1 - math.cos(math.pi * t / self.rise)) / 2 * self.peak
            elif t < self.duration: torque = (1 + math.cos(math.pi * (t - self.rise) / self.fall)) / 2 * self.peak
            else: torque = 0; self.mode = 'STAND'; self.get_logger().info('DRIVE done')
            speed = max(-self.max_speed, min(self.max_speed, torque * self.speed_gain))
            self._set_speed(speed)
            self.torque_pub.publish(Float64(data=float(torque)))
            self.speed_pub.publish(Float64(data=float(speed)))
        elif self.mode == 'SWING':
            err = self.initial_encoder - self.encoder
            spd = max(-self.swing_max, min(self.swing_max, err * self.swing_gain))
            self._set_speed(spd)
            if abs(err) < 1.0: self.mode = 'STAND'; self._set_speed(0)
        else:
            self._set_speed(0); self.torque_pub.publish(Float64(data=0.0))
        self.force_pub.publish(Float64(data=float(self.force * 1000.0)))
        m = {'STAND':0,'TORQUE':1,'PRE_TORQUE':2,'DRIVE':3,'SWING':4,'PRELOAD':5}.get(self.mode,0)
        self.mode_pub.publish(UInt8(data=m))
def main():
    rclpy.init(); node = AssistNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node._set_speed(0); node._tx('TX 00000653 8 2B 40 60 00 06 00 00 00')
        node._tx('QUIT'); node.sock.close(); node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()

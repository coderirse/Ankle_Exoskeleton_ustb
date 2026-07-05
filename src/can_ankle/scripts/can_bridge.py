#!/usr/bin/env python3
"""
CAN-to-ROS2 bridge for CANalyst-II adapter.
Uses python-can to talk to hardware, bridges to ROS2 topics.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import can
import threading
import time

class CanBridge(Node):
    def __init__(self):
        super().__init__('can_bridge')
        self.declare_parameter('channel', 0)
        self.declare_parameter('bitrate', 1000000)

        channel = self.get_parameter('channel').value
        bitrate = self.get_parameter('bitrate').value

        # Open CAN bus
        self.bus = can.interface.Bus(
            interface='canalystii', channel=channel, bitrate=bitrate)
        self.get_logger().info(f'CAN opened: canalystii ch{channel} @ {bitrate}bps')

        # Publishers and subscribers
        self.rx_pub = self.create_publisher(UInt8MultiArray, '/can_rx', 100)
        self.tx_sub = self.create_subscription(
            UInt8MultiArray, '/can_tx', self.tx_callback, 100)

        # RX thread
        self.running = True
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()
        self.get_logger().info('CAN bridge ready')

    def rx_loop(self):
        """Read CAN frames and publish to /can_rx"""
        while self.running and rclpy.ok():
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg is None:
                    continue
                # Format: [id_lo, id_hi, dlc, data0..data7]
                frame = UInt8MultiArray()
                frame.data = [
                    msg.arbitration_id & 0xFF,
                    (msg.arbitration_id >> 8) & 0xFF,
                    (msg.arbitration_id >> 16) & 0xFF,
                    (msg.arbitration_id >> 24) & 0xFF,
                    msg.dlc,
                    *list(msg.data[:8]),
                    0, 0, 0, 0, 0, 0, 0, 0  # pad to 8 data bytes
                ]
                # Trim to 4+1+8 = 13 bytes
                frame.data = frame.data[:13]
                self.rx_pub.publish(frame)
            except Exception as e:
                self.get_logger().error(f'RX error: {e}')

    def tx_callback(self, msg):
        """Send CAN frame from /can_tx topic"""
        try:
            data = msg.data
            if len(data) < 5:
                return
            arb_id = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24)
            dlc = data[4]
            payload = list(data[5:5+dlc]) if len(data) > 5 else []
            frame = can.Message(
                arbitration_id=arb_id,
                data=payload,
                dlc=dlc,
                is_extended_id=False)
            self.bus.send(frame)
        except Exception as e:
            self.get_logger().error(f'TX error: {e}')

    def destroy(self):
        self.running = False
        if self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1)
        self.bus.shutdown()
        super().destroy_node()

def main():
    rclpy.init()
    node = CanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

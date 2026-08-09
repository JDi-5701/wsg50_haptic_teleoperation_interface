#!/usr/bin/env python3
"""UDP -> ROS2 bridge for the ESP32 finger force sensor.

Publishes:
    gripper_finger_force_raw  (Int32)    raw FSR value from the ESP32
    gripper_finger_force      (Float32)  filtered force in N, zeroed at startup
"""

import socket
import struct
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32

PACKET_FMT = '<iif4xQ'          # id(i) FSR(i) force_filtered(f) pad(4x) timestamp(Q)
PACKET_SIZE = struct.calcsize(PACKET_FMT)   # 24 bytes


class UdpForceSensorBridge(Node):
    def __init__(self):
        super().__init__('esp32_force_sensor_node')

        self.declare_parameter('local_port', 5000)
        self.declare_parameter('esp32_address', '10.200.2.149')
        self.declare_parameter('esp32_port', 5000)
        self.declare_parameter('filter_window_size', 5)
        self.declare_parameter('publish_rate', 100.0)       # Hz, upper bound on gripper_finger_force
        self.declare_parameter('calibration_samples', 100)  # ~1 s at 100 Hz

        self.esp32_ip = self.get_parameter('esp32_address').value
        self.filter_window_size = self.get_parameter('filter_window_size').value
        self.min_publish_interval = 1.0 / self.get_parameter('publish_rate').value
        self.calibration_samples = self.get_parameter('calibration_samples').value
        local_port = self.get_parameter('local_port').value

        # --- socket -------------------------------------------------------
        # Non-blocking: check_udp_messages() drains the whole kernel buffer on
        # every tick. Reading only one packet per tick caps the consume rate at
        # the timer rate; if that equals the ESP32 send rate any backlog (e.g.
        # the packets that pile up between bind() and the first callback) can
        # never be worked off and shows up as a constant latency.
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', local_port))
        self.sock.setblocking(False)
        self.get_logger().info(f'Listening on 0.0.0.0:{local_port}, ESP32 at {self.esp32_ip}')

        # --- publishers ---------------------------------------------------
        self.raw_pub = self.create_publisher(Int32, 'gripper_finger_force_raw', 10)
        self.force_pub = self.create_publisher(Float32, 'gripper_finger_force', 10)
        self.raw_msg = Int32()
        self.force_msg = Float32()

        # --- filtering ----------------------------------------------------
        self.force_history = deque(maxlen=self.filter_window_size)
        self.weights = np.exp(np.linspace(0, 1, self.filter_window_size))
        self.weights /= np.sum(self.weights)

        # --- zero calibration (tare) ---------------------------------------
        # The first `calibration_samples` packets are averaged into an offset
        # that is subtracted from every later reading. Keep the sensor unloaded
        # until "Zero offset" is logged.
        self.zero_offset = None
        self.calib_buffer = []

        self.last_publish_time = 0.0
        self.timer = self.create_timer(0.01, self.check_udp_messages)   # 100 Hz

    def check_udp_messages(self):
        """Drain every packet that has already arrived."""
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
            except BlockingIOError:
                return
            except OSError as e:
                self.get_logger().error(f'recvfrom failed: {e}')
                return
            if addr[0] == self.esp32_ip:
                self.process_packet(data)

    def process_packet(self, data):
        if len(data) != PACKET_SIZE:
            self.get_logger().warning(
                f'Unexpected packet size: {len(data)} (expected {PACKET_SIZE})')
            return

        _, raw_force, force_filtered, _ = struct.unpack(PACKET_FMT, data)

        self.raw_msg.data = raw_force
        self.raw_pub.publish(self.raw_msg)

        # Collect the tare offset before publishing any force.
        if self.zero_offset is None:
            self.calib_buffer.append(force_filtered)
            if len(self.calib_buffer) >= self.calibration_samples:
                self.zero_offset = float(np.mean(self.calib_buffer))
                self.get_logger().info(
                    f'Zero offset = {self.zero_offset:.4f} '
                    f'(from {len(self.calib_buffer)} samples)')
                self.calib_buffer.clear()
            return

        force = self.filter_force(force_filtered - self.zero_offset)

        now = time.time()
        if now - self.last_publish_time >= self.min_publish_interval:
            self.force_msg.data = float(force)
            self.force_pub.publish(self.force_msg)
            self.last_publish_time = now

    def filter_force(self, force):
        """Exponentially weighted moving average with a small deadzone."""
        self.force_history.append(force)
        if len(self.force_history) < self.filter_window_size:
            return force
        filtered = np.average(self.force_history, weights=self.weights)
        return 0.0 if abs(filtered) < 0.1 else filtered

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UdpForceSensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""OmniHGI UDP driver. Carries bytes to and from the knob+coil ESP32.

One socket, both directions, no policy: all scaling, clamping and force-source
decisions live in omni_hgi_haptic_teleop.py. This node only translates between
ROS messages and the wire structs in wifi_task.h, so the protocol has exactly
one home.

    ESP32 -> PC   MotorMsg    36 B  '<ii f Q Q f i'   packed
                  id, fsr_value, force_filtered, force_ts, motor_ts,
                  motor_torque, knob_state

    PC -> ESP32   ForceMsg3D  12 B  '<fff'
                  force_x, force_y  -> 2D coils
                  force_z           -> knob feedback motor

The board demultiplexes on packet size, so the 12 B command cannot be confused
with the 24 B FSRMsg the FSR board sends. Both sizes are asserted at import.

Only knob_state and motor_torque are republished. MotorMsg still carries
fsr_value and force_filtered from the FSR-relay era, but the fingertip force
now comes from the Paxini sensor over its own ROS topic, so those fields are
unpacked and dropped.

Topics
    subscribes  /coil_command   (Vector3)  x,y -> coils / z -> knob motor
    publishes   /knob_state     (Int32)    encoder count, absolute
                /knob_torque    (Float32)  feedback motor torque

The command is resent every tx period whether or not it changed, because the
link is UDP and the board holds the last value it received. If commands stop
arriving for `command_timeout`, zeros go out instead - a dead publisher must
not leave current parked in a coil.

Standalone:
    ros2 run wsg50_haptic_teleoperation_interface omni_hgi_udp_driver.py
"""

import socket
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Float32, Int32

# --- Wire formats, mirroring the firmware structs they must match ---
MOTOR_MSG_FORMAT = '<ii f Q Q f i'          # MotorMsg, #pragma pack(1)
MOTOR_MSG_SIZE = struct.calcsize(MOTOR_MSG_FORMAT)
FORCE_MSG_3D_FORMAT = '<fff'                # ForceMsg3D
FORCE_MSG_3D_SIZE = struct.calcsize(FORCE_MSG_3D_FORMAT)

assert MOTOR_MSG_SIZE == 36, MOTOR_MSG_SIZE
assert FORCE_MSG_3D_SIZE == 12, FORCE_MSG_3D_SIZE


class OmniHgiUdpDriver(Node):
    def __init__(self):
        super().__init__('omni_hgi_udp_driver')

        self.declare_parameter('local_port', 5001)
        self.declare_parameter('esp32_address', '10.200.2.148')
        self.declare_parameter('esp32_port', 5000)

        # The firmware drains one packet per WifiTask cycle and that cycle is
        # vTaskDelay(10 ms), so anything above 100 Hz just backs up in the
        # board's UDP buffer and shows up as growing latency.
        self.declare_parameter('tx_rate', 100.0)
        self.declare_parameter('publish_rate', 100.0)

        self.declare_parameter('command_timeout', 0.5)

        self.esp32_address = (
            self.get_parameter('esp32_address').value,
            self.get_parameter('esp32_port').value,
        )
        self.command_timeout = self.get_parameter('command_timeout').value
        self.min_publish_interval = 1.0 / self.get_parameter('publish_rate').value

        local_port = self.get_parameter('local_port').value
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', local_port))
        self.get_logger().info(f'Bound to local port {local_port}')

        self.knob_state_pub = self.create_publisher(Int32, 'knob_state', 10)
        self.knob_torque_pub = self.create_publisher(Float32, 'knob_torque', 10)

        self.create_subscription(Vector3, 'coil_command', self.command_callback, 10)

        # Written by the subscription, read by the TX timer. Both run on the
        # executor thread, so the tuple swap needs no lock.
        self.cmd = (0.0, 0.0, 0.0)
        self.last_cmd_stamp = 0.0
        self.timed_out = True                  # nothing received yet

        self.last_publish_time = 0.0
        self.rx_count = 0
        self.last_rate_log = time.time()

        self.running = True
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        tx_rate = self.get_parameter('tx_rate').value
        self.create_timer(1.0 / tx_rate, self.tx_loop)
        self.get_logger().info(
            f'OmniHGI UDP driver up: TX {tx_rate:.0f} Hz -> '
            f'{self.esp32_address[0]}:{self.esp32_address[1]}')

    # ---------------- PC -> ESP32 ----------------
    def command_callback(self, msg):
        self.cmd = (msg.x, msg.y, msg.z)
        self.last_cmd_stamp = time.time()
        if self.timed_out:
            self.get_logger().info('Coil command source live.')
            self.timed_out = False

    def tx_loop(self):
        if time.time() - self.last_cmd_stamp > self.command_timeout:
            if not self.timed_out:
                self.get_logger().warn(
                    f'No coil command for {self.command_timeout:.1f} s - '
                    f'sending zeros.')
                self.timed_out = True
            self.cmd = (0.0, 0.0, 0.0)

        x, y, z = self.cmd
        try:
            self.sock.sendto(
                struct.pack(FORCE_MSG_3D_FORMAT, float(x), float(y), float(z)),
                self.esp32_address)
        except OSError as e:
            self.get_logger().error(f'UDP send failed: {e}')

    # ---------------- ESP32 -> PC ----------------
    def rx_loop(self):
        knob_state_msg = Int32()
        knob_torque_msg = Float32()

        while self.running:
            try:
                self.sock.settimeout(0.1)
                data, addr = self.sock.recvfrom(1024)
            except (socket.timeout, OSError):
                continue

            if addr[0] != self.esp32_address[0] or len(data) < MOTOR_MSG_SIZE:
                continue

            # fsr_value and force_filtered belong to the retired FSR relay path.
            (_msg_id, _fsr_value, _force_filtered, _force_ts,
             _motor_ts, motor_torque, knob_state) = struct.unpack(
                MOTOR_MSG_FORMAT, data[:MOTOR_MSG_SIZE])

            self.rx_count += 1
            now = time.time()
            if now - self.last_rate_log >= 1.0:
                self.get_logger().info(f'RX {self.rx_count} packets/s')
                self.rx_count = 0
                self.last_rate_log = now

            if now - self.last_publish_time < self.min_publish_interval:
                continue
            self.last_publish_time = now

            knob_state_msg.data = knob_state
            knob_torque_msg.data = motor_torque
            self.knob_state_pub.publish(knob_state_msg)
            self.knob_torque_pub.publish(knob_torque_msg)

    def destroy_node(self):
        self.running = False
        if self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1.0)
        # Park the coils before dropping the link.
        try:
            self.sock.sendto(
                struct.pack(FORCE_MSG_3D_FORMAT, 0.0, 0.0, 0.0), self.esp32_address)
        except OSError:
            pass
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OmniHgiUdpDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

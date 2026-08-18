#!/usr/bin/env python3
"""OmniHGI bridge: one socket, both directions, to the knob+coil ESP32.

Replaces the pair udp_ros_bridge.py + coil_udp_driver.py. Those two could not
run together: both bound UDP :5001 and both published /knob_state, while the
board they talk to is the same one (10.200.2.148). Now that the firmware
handles knob and coil in a single WifiTask, the PC side collapses too.

Wire protocol (matches wifi_task.h):

    ESP32 -> PC   MotorMsg    36 B  '<ii f Q Q f i'   packed
                  id, fsr_value, force_filtered, force_ts, motor_ts,
                  motor_torque, knob_state

    PC -> ESP32   ForceMsg3D  12 B  '<fff'
                  force_x, force_y  -> 2D coils
                  force_z           -> knob feedback motor

The board demultiplexes on packet size, so the 12 B command cannot be confused
with the 24 B FSRMsg the old FSR board sends. Sizes are asserted at import.

Force source is the Paxini fingertip (/tactile_resultant_wrench): its shear
components drive the coils, its normal component drives the knob motor. Note
the sensor reports Fz unsigned, so the knob only ever feels push, never pull.

Topics
    subscribes  /tactile_resultant_wrench   (WrenchStamped)
    publishes   /knob_state                 (Int32)   encoder count
                /knob_torque                (Float32) feedback motor torque
                /gripper_finger_force       (Float32) tared, relayed force
                /coil_command               (Vector3) what was actually sent

Standalone:
    ros2 run wsg50_haptic_teleoperation_interface omni_hgi_bridge
"""

import socket
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import Vector3, WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Float32, Int32

# --- Wire formats, kept next to the firmware structs they mirror ---
MOTOR_MSG_FORMAT = '<ii f Q Q f i'          # MotorMsg, #pragma pack(1)
MOTOR_MSG_SIZE = struct.calcsize(MOTOR_MSG_FORMAT)
FORCE_MSG_3D_FORMAT = '<fff'                # ForceMsg3D
FORCE_MSG_3D_SIZE = struct.calcsize(FORCE_MSG_3D_FORMAT)

assert MOTOR_MSG_SIZE == 36, MOTOR_MSG_SIZE
assert FORCE_MSG_3D_SIZE == 12, FORCE_MSG_3D_SIZE


class OmniHgiBridge(Node):
    def __init__(self):
        super().__init__('omni_hgi_bridge')

        self.declare_parameter('local_port', 5001)
        self.declare_parameter('esp32_address', '10.200.2.148')
        self.declare_parameter('esp32_port', 5000)

        # The firmware drains one packet per WifiTask cycle and that cycle is
        # vTaskDelay(10 ms), so anything above 100 Hz just backs up in the
        # board's UDP buffer and shows up as growing latency.
        self.declare_parameter('tx_rate', 100.0)
        self.declare_parameter('publish_rate', 100.0)

        # Shear force (N) -> coil duty cycle, then clamped.
        self.declare_parameter('coil_ratio', 0.08)
        self.declare_parameter('coil_max_duty', 0.8)

        # Normal force (N) -> knob feedback motor.
        self.declare_parameter('knob_force_ratio', 1.0)
        self.declare_parameter('knob_force_max', 5.0)

        # Zero the relayed force at startup; see udp_ros_bridge.py.
        self.declare_parameter('calibration_samples', 100)

        # Stop driving the coils if the force source goes quiet, so a dead
        # Paxini node cannot leave current parked in a coil.
        self.declare_parameter('force_timeout', 0.5)

        self.esp32_address = (
            self.get_parameter('esp32_address').value,
            self.get_parameter('esp32_port').value,
        )
        self.coil_ratio = self.get_parameter('coil_ratio').value
        self.coil_max_duty = self.get_parameter('coil_max_duty').value
        self.knob_force_ratio = self.get_parameter('knob_force_ratio').value
        self.knob_force_max = self.get_parameter('knob_force_max').value
        self.force_timeout = self.get_parameter('force_timeout').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.min_publish_interval = 1.0 / self.publish_rate

        local_port = self.get_parameter('local_port').value
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', local_port))
        self.get_logger().info(f'Bound to local port {local_port}')

        self.knob_state_pub = self.create_publisher(Int32, 'knob_state', 10)
        self.knob_torque_pub = self.create_publisher(Float32, 'knob_torque', 10)
        self.force_pub = self.create_publisher(Float32, 'gripper_finger_force', 10)
        self.coil_cmd_pub = self.create_publisher(Vector3, 'coil_command', 10)

        self.create_subscription(
            WrenchStamped, '/tactile_resultant_wrench', self.wrench_callback, 10)

        # Command state, written by the subscription and read by the TX timer.
        self.cmd = (0.0, 0.0, 0.0)
        self.last_force_stamp = 0.0
        self.timed_out = True          # nothing received yet

        self.calibration_samples = self.get_parameter('calibration_samples').value
        self.calib_buffer = []
        self.zero_offset = 0.0 if self.calibration_samples <= 0 else None

        self.last_publish_time = 0.0
        self.rx_count = 0
        self.last_rate_log = time.time()

        self.running = True
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        tx_rate = self.get_parameter('tx_rate').value
        self.create_timer(1.0 / tx_rate, self.tx_loop)
        self.get_logger().info(
            f'OmniHGI bridge up: TX {tx_rate:.0f} Hz -> '
            f'{self.esp32_address[0]}:{self.esp32_address[1]}')

    # ---------------- PC -> ESP32 ----------------
    def wrench_callback(self, msg):
        f = msg.wrench.force

        def clamp(v, lo, hi):
            return max(min(v, hi), lo)

        duty_x = clamp(f.x * self.coil_ratio, -self.coil_max_duty, self.coil_max_duty)
        duty_y = clamp(f.y * self.coil_ratio, -self.coil_max_duty, self.coil_max_duty)
        # Fz is unsigned on the sensor, but clamp both ends anyway: a future
        # signed firmware must not be able to command a runaway here.
        force_z = clamp(f.z * self.knob_force_ratio,
                        -self.knob_force_max, self.knob_force_max)

        self.cmd = (duty_x, duty_y, force_z)
        self.last_force_stamp = time.time()
        if self.timed_out:
            self.get_logger().info('Force source live.')
            self.timed_out = False

    def tx_loop(self):
        if time.time() - self.last_force_stamp > self.force_timeout:
            if not self.timed_out:
                self.get_logger().warn(
                    f'No wrench for {self.force_timeout:.1f} s - zeroing coils.')
                self.timed_out = True
            self.cmd = (0.0, 0.0, 0.0)

        x, y, z = self.cmd
        try:
            self.sock.sendto(
                struct.pack(FORCE_MSG_3D_FORMAT, float(x), float(y), float(z)),
                self.esp32_address)
        except OSError as e:
            self.get_logger().error(f'UDP send failed: {e}')
            return

        cmd_msg = Vector3()
        cmd_msg.x, cmd_msg.y, cmd_msg.z = float(x), float(y), float(z)
        self.coil_cmd_pub.publish(cmd_msg)

    # ---------------- ESP32 -> PC ----------------
    def rx_loop(self):
        knob_state_msg = Int32()
        knob_torque_msg = Float32()
        force_msg = Float32()

        while self.running:
            try:
                self.sock.settimeout(0.1)
                data, addr = self.sock.recvfrom(1024)
            except (socket.timeout, OSError):
                continue

            if addr[0] != self.esp32_address[0] or len(data) < MOTOR_MSG_SIZE:
                continue

            (_msg_id, _fsr_value, force_filtered, _force_ts,
             _motor_ts, motor_torque, knob_state) = struct.unpack(
                MOTOR_MSG_FORMAT, data[:MOTOR_MSG_SIZE])

            self.rx_count += 1
            now = time.time()
            if now - self.last_rate_log >= 1.0:
                self.get_logger().info(f'RX {self.rx_count} packets/s')
                self.rx_count = 0
                self.last_rate_log = now

            # Tare on every packet, independent of the publish throttle.
            if self.zero_offset is None:
                self.calib_buffer.append(force_filtered)
                if len(self.calib_buffer) >= self.calibration_samples:
                    self.zero_offset = sum(self.calib_buffer) / len(self.calib_buffer)
                    self.get_logger().info(
                        f'Zero offset = {self.zero_offset:.4f} N '
                        f'(from {len(self.calib_buffer)} samples)')
                    self.calib_buffer.clear()

            if now - self.last_publish_time < self.min_publish_interval:
                continue
            self.last_publish_time = now

            knob_state_msg.data = knob_state
            knob_torque_msg.data = motor_torque
            self.knob_state_pub.publish(knob_state_msg)
            self.knob_torque_pub.publish(knob_torque_msg)

            if self.zero_offset is not None:
                force_msg.data = float(force_filtered - self.zero_offset)
                self.force_pub.publish(force_msg)

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
    node = OmniHgiBridge()
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

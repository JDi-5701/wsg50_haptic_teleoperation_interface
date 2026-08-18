#!/usr/bin/env python3
"""OmniHGI UDP driver. Carries bytes to and from the knob+coil ESP32.

One socket, both directions, no policy: all scaling, clamping and force-source
decisions live in omni_hgi_haptic_teleop.py. This node only translates between
ROS messages and the wire structs in wifi_task.h, so the protocol has exactly
one home.

    ESP32 -> PC   MotorMsg    12 B  '<i f i'   packed
                  id            packet sequence counter
                  motor_torque  q-axis voltage, V
                  knob_state    knob position, MILLIRADIANS, absolute

    PC -> ESP32   ForceMsg3D  12 B  '<fff'
                  force_x, force_y  -> 2D coils, normalised duty in [-1, 1]
                  force_z           -> knob feedback motor, newtons

Both structs are 12 B, so size alone cannot tell them apart. That is harmless
here because each travels one way only - the board never receives MotorMsg and
this node never receives ForceMsg3D - but it does mean the source-address check
in rx_loop is load-bearing, not just tidy, and the receive path demands an
exact length rather than a minimum.

force_z is passed through in newtons. The board applies its own gain, offset,
log compression and clamp in computeForceFeedback(), so pre-scaling here would
be applied twice. force_x/y are already normalised: forceToCounts() clamps
|value| to 1.0 and maps it onto the 10-bit PWM range, with a +/-0.001 deadband.

Topics
    subscribes  /coil_command   (Vector3)  x,y -> coils / z -> knob motor
    publishes   /knob_state     (Int32)    knob position, milliradians
                /knob_torque    (Float32)  q-axis voltage, V

Every received packet is republished; see the publish_rate parameter for why
throttling is off by default.

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
MOTOR_MSG_FORMAT = '<i f i'                 # MotorMsg, #pragma pack(1)
MOTOR_MSG_SIZE = struct.calcsize(MOTOR_MSG_FORMAT)
FORCE_MSG_3D_FORMAT = '<fff'                # ForceMsg3D
FORCE_MSG_3D_SIZE = struct.calcsize(FORCE_MSG_3D_FORMAT)

assert MOTOR_MSG_SIZE == 12, MOTOR_MSG_SIZE
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

        # 0 disables throttling, which is the right default: the board is
        # already the slower end, and WiFi delivers its packets in bursts (an
        # observed minimum inter-arrival of 0 ms against a ~13 ms median). A
        # rate limiter keeps the FIRST packet of a burst and drops the rest,
        # so it would publish the oldest sample and discard the freshest.
        self.declare_parameter('publish_rate', 0.0)

        self.declare_parameter('command_timeout', 0.5)

        self.esp32_address = (
            self.get_parameter('esp32_address').value,
            self.get_parameter('esp32_port').value,
        )
        self.command_timeout = self.get_parameter('command_timeout').value
        publish_rate = self.get_parameter('publish_rate').value
        self.min_publish_interval = 1.0 / publish_rate if publish_rate > 0 else 0.0

        # Separate sockets per direction, deliberately. A single shared socket
        # carries one blocking mode for both: settimeout() for the receive
        # thread also applies to sendto(), so when the board drops off WiFi the
        # send blocks up to the timeout on the executor thread and stalls the
        # whole node - command intake and the coil watchdog included. The board
        # sends to a hardcoded address:port rather than replying to our source
        # port, so transmitting from a different socket is invisible to it.
        local_port = self.get_parameter('local_port').value
        self.rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock.bind(('0.0.0.0', local_port))
        self.rx_sock.settimeout(0.1)          # set once, not per iteration
        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx_sock.setblocking(False)       # a dead board must never stall us
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
        self.lost_count = 0
        self.last_msg_id = None
        self.last_rate_log = time.time()
        self.last_rx_time = time.time()
        self.rx_stalled = False
        self.send_errors = 0
        self.last_send_error_log = 0.0

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
            self.tx_sock.sendto(
                struct.pack(FORCE_MSG_3D_FORMAT, float(x), float(y), float(z)),
                self.esp32_address)
            self.send_errors = 0
        except OSError as e:
            # Throttled: an unreachable board fails every tx period, and at
            # 100 Hz the raw error drowns everything else in the log.
            self.send_errors += 1
            now = time.time()
            if now - self.last_send_error_log >= 2.0:
                self.get_logger().error(
                    f'UDP send failing ({self.send_errors} times): {e}')
                self.last_send_error_log = now

    # ---------------- ESP32 -> PC ----------------
    def rx_loop(self):
        knob_state_msg = Int32()
        knob_torque_msg = Float32()

        while self.running:
            try:
                data, addr = self.rx_sock.recvfrom(1024)
            except (socket.timeout, OSError):
                if not self.rx_stalled and time.time() - self.last_rx_time > 2.0:
                    self.rx_stalled = True
                    self.get_logger().warn(
                        'No packet from the board for 2 s - is it powered and '
                        'on WiFi?')
                continue
            if self.rx_stalled:
                self.get_logger().info('Board is back.')
                self.rx_stalled = False
            self.last_rx_time = time.time()

            # Exact length: MotorMsg and ForceMsg3D are both 12 B, so a stray
            # command echoed back would unpack cleanly into nonsense. Together
            # with the source check above this keeps the two apart.
            if addr[0] != self.esp32_address[0] or len(data) != MOTOR_MSG_SIZE:
                continue

            msg_id, motor_torque, knob_state = struct.unpack(MOTOR_MSG_FORMAT, data)

            # id is a sequence counter, so gaps in it are dropped packets.
            if self.last_msg_id is not None:
                gap = msg_id - self.last_msg_id
                if gap > 1:
                    self.lost_count += gap - 1
                elif gap <= 0:
                    self.get_logger().warn(
                        f'Sequence went backwards ({self.last_msg_id} -> '
                        f'{msg_id}); board restarted?')
                    self.lost_count = 0
            self.last_msg_id = msg_id

            self.rx_count += 1
            now = time.time()
            if now - self.last_rate_log >= 1.0:
                if self.lost_count:
                    self.get_logger().warn(
                        f'RX {self.rx_count} packets/s, {self.lost_count} lost')
                else:
                    self.get_logger().info(f'RX {self.rx_count} packets/s')
                self.rx_count = 0
                self.lost_count = 0
                self.last_rate_log = now

            if self.min_publish_interval:
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
            self.tx_sock.sendto(
                struct.pack(FORCE_MSG_3D_FORMAT, 0.0, 0.0, 0.0), self.esp32_address)
        except OSError:
            pass
        self.tx_sock.close()
        self.rx_sock.close()
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

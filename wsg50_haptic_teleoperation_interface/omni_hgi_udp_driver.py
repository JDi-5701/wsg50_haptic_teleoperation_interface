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
        # Seconds between link-quality reports. 0 turns them off.
        self.declare_parameter('report_period', 2.0)

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
        # Monotonic totals, never reset. The report timer runs on the executor
        # thread while these are written by the RX thread, so it takes
        # differences against its own previous snapshot rather than zeroing
        # them - a reset from the other thread would lose whatever arrived
        # between the read and the write.
        self.rx_total = 0
        self.lost_total = 0
        self.tx_total = 0
        self.late_total = 0        # inter-arrival gaps beyond LATE_FACTOR
        self.prev_snapshot = (0, 0, 0, 0)
        self.gap_max = 0.0         # worst inter-arrival in the window, seconds
        self.prev_rx_time = None
        self.last_msg_id = None
        self.last_rx_time = time.time()
        self.rx_stalled = False
        self.send_errors = 0
        self.last_send_error_log = 0.0

        # An arrival is "late" once it is this many periods behind the one
        # before it. The reference has to be the rate the board actually SENDS
        # at, which is its own and is not the TX rate the PC uses: measured on
        # hardware the board sent 72 Hz while the PC sent 100, and judging the
        # incoming stream against the outgoing 10 ms period called ordinary
        # 14 ms spacing late. So it is measured, and seeded from tx_rate only
        # until the first window has been observed.
        #
        # Set before the RX thread starts: that thread reads late_threshold on
        # its very first packet, which can arrive before this constructor
        # returns.
        tx_rate = self.get_parameter('tx_rate').value
        self.report_period = float(self.get_parameter('report_period').value)
        self.nominal_rx_period = 1.0 / tx_rate
        self.rx_period_measured = False
        self.late_threshold = 2.5 * self.nominal_rx_period
        self.last_report = time.time()

        self.running = True
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        self.create_timer(1.0 / tx_rate, self.tx_loop)
        if self.report_period > 0.0:
            self.create_timer(self.report_period, self.report_link)

        self.get_logger().info(
            f'OmniHGI UDP driver up: TX {tx_rate:.0f} Hz -> '
            f'{self.esp32_address[0]}:{self.esp32_address[1]}')

    def report_link(self):
        """How the ESP32 link behaved over the last window.

        Reported from a timer rather than from the RX handler, so that a link
        that has gone completely silent says so instead of simply ceasing to
        log - which is what the old rate line did, and it read as if nothing
        were wrong.
        """
        now = time.time()
        elapsed = now - self.last_report
        self.last_report = now
        if elapsed <= 0.0:
            return

        rx, lost, tx, late = (self.rx_total, self.lost_total,
                              self.tx_total, self.late_total)
        p_rx, p_lost, p_tx, p_late = self.prev_snapshot
        self.prev_snapshot = (rx, lost, tx, late)
        d_rx, d_lost, d_tx, d_late = rx - p_rx, lost - p_lost, tx - p_tx, late - p_late
        gap_max = self.gap_max
        self.gap_max = 0.0

        rx_hz, tx_hz = d_rx / elapsed, d_tx / elapsed
        expected = d_rx + d_lost
        loss_pct = (100.0 * d_lost / expected) if expected else 0.0

        # Re-reference the lateness test to what the board is actually doing.
        # Uses the packets the board sent, not the ones that arrived, so a
        # window that lost half of them does not halve the expected rate and
        # then call the surviving packets punctual.
        if expected > 1:
            self.nominal_rx_period = elapsed / expected
            self.late_threshold = 2.5 * self.nominal_rx_period
            self.rx_period_measured = True

        if d_rx == 0:
            self.get_logger().error(
                f'ESP32 link: RX 0.0 Hz - NOTHING RECEIVED in {elapsed:.1f} s '
                f'(TX {tx_hz:.0f} Hz still going out)')
            return

        # Steady means every packet arrived and none of them arrived late. Both
        # halves matter: a stream can lose nothing and still stutter, and it can
        # keep perfect spacing while dropping every other packet.
        if d_lost == 0 and d_late == 0:
            verdict = 'STEADY'
        elif d_lost == 0:
            verdict = f'JITTERY ({d_late} late)'
        else:
            verdict = f'LOSSY ({d_lost} lost, {loss_pct:.1f}%'
            verdict += f', {d_late} late)' if d_late else ')'

        line = (f'ESP32 link {verdict}: RX {rx_hz:5.1f} Hz  TX {tx_hz:5.1f} Hz  '
                f'worst gap {gap_max * 1000:.0f} ms '
                f'(board sends every {self.nominal_rx_period * 1000:.0f} ms'
                f'{"" if self.rx_period_measured else ", assumed"})')

        # One severity per call site: rclpy identifies a logger call by its
        # file and line, and raises "Logger severity cannot be changed between
        # calls" if the same line logs at INFO once and WARN the next time.
        if verdict == 'STEADY':
            self.get_logger().info(line)
        else:
            self.get_logger().warn(line)

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
            self.tx_total += 1
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
                    self.lost_total += gap - 1
                elif gap <= 0:
                    self.get_logger().warn(
                        f'Sequence went backwards ({self.last_msg_id} -> '
                        f'{msg_id}); board restarted?')
            self.last_msg_id = msg_id

            self.rx_total += 1
            now = time.time()

            # Rate alone cannot tell a steady stream from a bursty one: 100
            # packets in a second arriving as one clump every 10 ms and as a
            # 500 ms silence followed by a burst both read as 100 Hz. The
            # spacing between arrivals is what separates them.
            if self.prev_rx_time is not None:
                gap = now - self.prev_rx_time
                if gap > self.gap_max:
                    self.gap_max = gap
                if gap > self.late_threshold:
                    self.late_total += 1
            self.prev_rx_time = now

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

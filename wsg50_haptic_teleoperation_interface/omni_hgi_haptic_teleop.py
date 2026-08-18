#!/usr/bin/env python3
"""OmniHGI haptic teleoperation. Maps topics to topics, touches no hardware.

Two independent mappings, both pure ROS:

    /knob_state              -> /wsg50/command/move   (operator drives gripper)
    /tactile_resultant_wrench -> /coil_command        (gripper force felt back)

omni_hgi_udp_driver.py owns the socket and the wire format; this node owns the
gains and limits. Splitting them keeps the protocol in one file and the tuning
in another, and lets either be tested without the other.

The haptic loop is closed through the PC: the Paxini fingertip is the only
force source, and nothing reaches the coils or the knob motor except through
these topics. Latency is Paxini poll + ROS + UDP, and the Paxini node's 30 Hz
is the slowest link in it.

Coil mapping - the fingertip's shear becomes coil drive, its normal force
becomes knob resistance:

    /coil_command.x = clamp(Fx * coil_ratio, +/- coil_max_duty)
    /coil_command.y = clamp(Fy * coil_ratio, +/- coil_max_duty)
    /coil_command.z = clamp(Fz * knob_force_ratio, +/- knob_force_max)

x and y are normalised coil duty: the board clamps |value| to 1.0 in
forceToCounts() before mapping onto its 10-bit PWM, and ignores anything under
0.001. z stays in newtons - computeForceFeedback() on the board applies its own
ratio, offset, log compression and clamp, so scaling it here would apply a gain
twice. knob_force_ratio therefore defaults to 1.0, a pass-through.

knob_state arrives in MILLIRADIANS (the firmware sends
int32_t(1000 * angle_from_start)), not encoder counts, so position_factor is mm
of gripper width per milliradian. At the 0.013 default the gripper's 95 mm span
takes about 7.3 rad, a little over one turn of the knob.

Standalone:
    ros2 run wsg50_haptic_teleoperation_interface omni_hgi_haptic_teleop.py
"""

import rclpy
from geometry_msgs.msg import Vector3, WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Int32
from wsg50_ros_driver.msg import GripperCommand


def clamp(v, lo, hi):
    return max(min(v, hi), lo)


class OmniHgiHapticTeleop(Node):
    def __init__(self):
        super().__init__('omni_hgi_haptic_teleop')

        # --- knob -> gripper ---
        # Carried over verbatim from knob_force_feedback_controller_wsg50.py,
        # which was working on the hardware. position_factor is now mm of
        # gripper width per MILLIRADIAN, since that is what the reworked
        # firmware sends, but the integration itself is unchanged.
        self.declare_parameter('position_factor', 0.013)
        self.declare_parameter('position_jump_threshold', 1000)
        self.declare_parameter('gripper_min_width', 10.0)
        self.declare_parameter('gripper_max_width', 105.0)
        self.declare_parameter('gripper_velocity', 0.1)
        self.declare_parameter('gripper_start_width', 102.0)
        self.declare_parameter('control_rate', 100.0)
        # 0 keeps the original behaviour of commanding on every tick. Set it
        # positive to suppress unchanged targets, which quiets the WSG50
        # driver's per-command 52-waypoint requeue and its log.
        self.declare_parameter('command_deadband', 0.0)

        # --- fingertip -> coils ---
        self.declare_parameter('coil_ratio', 0.08)
        self.declare_parameter('coil_max_duty', 0.8)
        self.declare_parameter('knob_force_ratio', 1.0)
        self.declare_parameter('knob_force_max', 5.0)
        self.declare_parameter('coil_deadzone', 0.0)

        self.position_factor = self.get_parameter('position_factor').value
        self.position_jump_threshold = self.get_parameter('position_jump_threshold').value
        self.gripper_min_width = self.get_parameter('gripper_min_width').value
        self.gripper_max_width = self.get_parameter('gripper_max_width').value
        self.gripper_velocity = self.get_parameter('gripper_velocity').value

        self.coil_ratio = self.get_parameter('coil_ratio').value
        self.coil_max_duty = self.get_parameter('coil_max_duty').value
        self.knob_force_ratio = self.get_parameter('knob_force_ratio').value
        self.knob_force_max = self.get_parameter('knob_force_max').value
        self.coil_deadzone = self.get_parameter('coil_deadzone').value

        self.gripper_target = self.get_parameter('gripper_start_width').value
        self.command_deadband = self.get_parameter('command_deadband').value
        self.last_commanded = None             # None until the first command
        self.knob_position_last = 0
        self.position_change = 0.0

        self.gripper_cmd_pub = self.create_publisher(
            GripperCommand, '/wsg50/command/move', 10)
        self.coil_cmd_pub = self.create_publisher(Vector3, '/coil_command', 10)

        self.create_subscription(Int32, '/knob_state', self.knob_callback, 10)
        self.create_subscription(
            WrenchStamped, '/tactile_resultant_wrench', self.wrench_callback, 10)

        control_rate = self.get_parameter('control_rate').value
        self.create_timer(1.0 / control_rate, self.control_loop)

        self.get_logger().info(
            f'OmniHGI haptic teleop up at {control_rate:.0f} Hz. '
            f'Gripper {self.gripper_min_width:.0f}-{self.gripper_max_width:.0f} mm, '
            f'start {self.gripper_target:.1f} mm.')

    # ---------------- knob -> gripper ----------------
    def knob_callback(self, msg):
        knob_position_current = msg.data
        knob_position_delta = knob_position_current - self.knob_position_last

        if abs(knob_position_delta) > self.position_jump_threshold:
            self.get_logger().warn(
                f'Large position jump detected: {knob_position_delta}')
        else:
            self.position_change = -1 * self.position_factor * knob_position_delta

        self.knob_position_last = knob_position_current

    def control_loop(self):
        self.gripper_target -= self.position_change
        self.gripper_target = clamp(self.gripper_target,
                                    self.gripper_min_width,
                                    self.gripper_max_width)

        if (self.command_deadband and self.last_commanded is not None
                and abs(self.gripper_target - self.last_commanded)
                < self.command_deadband):
            return

        cmd = GripperCommand()
        cmd.position = float(self.gripper_target)
        cmd.velocity = float(self.gripper_velocity)
        self.gripper_cmd_pub.publish(cmd)
        self.last_commanded = self.gripper_target

        if abs(self.position_change) > 0:
            self.get_logger().info(
                f'Position change: {self.position_change:.2f} mm')
            self.get_logger().info(
                f'Gripper target: {self.gripper_target:.2f} mm')

    # ---------------- fingertip -> coils ----------------
    def wrench_callback(self, msg):
        f = msg.wrench.force

        def shear(v):
            if abs(v) < self.coil_deadzone:
                return 0.0
            return clamp(v * self.coil_ratio, -self.coil_max_duty, self.coil_max_duty)

        out = Vector3()
        out.x = shear(f.x)
        out.y = shear(f.y)
        # The sensor reports Fz unsigned, so this is push-only in practice;
        # clamp both ends anyway so a signed firmware cannot command a runaway.
        out.z = clamp(f.z * self.knob_force_ratio,
                      -self.knob_force_max, self.knob_force_max)
        self.coil_cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OmniHgiHapticTeleop()
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

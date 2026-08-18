#!/usr/bin/env python3
"""OmniHGI haptic teleoperation. Maps topics to topics, touches no hardware.

Two independent mappings, both pure ROS:

    /knob_state              -> /wsg50/command/move   (operator drives gripper)
    /tactile_resultant_wrench -> /coil_command        (gripper force felt back)

omni_hgi_udp_driver.py owns the socket and the wire format; this node owns the
gains and limits. Splitting them keeps the protocol in one file and the tuning
in another, and lets either be tested without the other.

The haptic loop is closed through the PC here, unlike the FSR/knob setup where
the two boards talked directly. Latency is therefore Paxini poll + ROS + UDP,
and the Paxini node's 30 Hz is the slowest link in it.

Coil mapping - the fingertip's shear becomes coil drive, its normal force
becomes knob resistance:

    /coil_command.x = clamp(Fx * coil_ratio, +/- coil_max_duty)
    /coil_command.y = clamp(Fy * coil_ratio, +/- coil_max_duty)
    /coil_command.z = clamp(Fz * knob_force_ratio, +/- knob_force_max)

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
        self.declare_parameter('position_factor', 0.013)
        self.declare_parameter('position_jump_threshold', 1000)
        self.declare_parameter('gripper_min_width', 10.0)
        self.declare_parameter('gripper_max_width', 105.0)
        self.declare_parameter('gripper_velocity', 0.1)
        self.declare_parameter('gripper_start_width', 102.0)
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('invert_knob', True)

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
        self.knob_sign = -1.0 if self.get_parameter('invert_knob').value else 1.0

        self.coil_ratio = self.get_parameter('coil_ratio').value
        self.coil_max_duty = self.get_parameter('coil_max_duty').value
        self.knob_force_ratio = self.get_parameter('knob_force_ratio').value
        self.knob_force_max = self.get_parameter('knob_force_max').value
        self.coil_deadzone = self.get_parameter('coil_deadzone').value

        self.gripper_target = self.get_parameter('gripper_start_width').value
        self.knob_last = None                  # None until the first packet
        self.pending_delta = 0.0               # consumed by the control loop

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
        # Absolute encoder count; only its change matters. The first packet
        # establishes the reference instead of being read as a huge delta.
        if self.knob_last is None:
            self.knob_last = msg.data
            self.get_logger().info(f'Knob reference set at {msg.data}.')
            return

        delta = msg.data - self.knob_last
        self.knob_last = msg.data

        if abs(delta) > self.position_jump_threshold:
            self.get_logger().warn(f'Ignoring knob jump of {delta} counts.')
            return

        # Accumulate: the control loop may tick more than once per knob packet,
        # and must not apply the same delta twice.
        self.pending_delta += self.knob_sign * self.position_factor * delta

    def control_loop(self):
        if self.pending_delta:
            self.gripper_target = clamp(self.gripper_target + self.pending_delta,
                                        self.gripper_min_width,
                                        self.gripper_max_width)
            self.get_logger().info(
                f'Knob {self.pending_delta:+.2f} mm -> '
                f'target {self.gripper_target:.2f} mm')
            self.pending_delta = 0.0

        cmd = GripperCommand()
        cmd.position = float(self.gripper_target)
        cmd.velocity = float(self.gripper_velocity)
        self.gripper_cmd_pub.publish(cmd)

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

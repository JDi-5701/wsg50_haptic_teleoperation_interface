#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from wsg50_ros_driver.msg import GripperCommand
import numpy as np
import math

class WSG50TeleoperationInterface(Node):
    def __init__(self):
        super().__init__('wsg50_controller')

        # Initialize parameters with default values
        self.declare_parameter('position_factor', 0.013)
        self.declare_parameter('force_feedback_ratio', 0.5)
        self.declare_parameter('clamp_force_threshold_max', 5.0)
        self.declare_parameter('clamp_force_threshold_min', 0.0)
        self.declare_parameter('force_offset', 0.0)
        self.declare_parameter('position_jump_threshold', 1000)
        self.declare_parameter('force_deadzone', 0.1)
        self.declare_parameter('gripper_min_width', 10.0)
        self.declare_parameter('gripper_max_width', 105.0)
        self.declare_parameter('gripper_velocity', 0.1)
        self.declare_parameter('control_rate', 100.0)  # Hz
        self.declare_parameter('gripper_compliance_parameter', 0.05)

        # Initialize variables
        self.gripper_position_current = 102.0
        self.gripper_position_nominal = 102.0  # user-commanded position
        self.gripper_position_command = 102.0  # output to hardware
        self.knob_position_current = 0
        self.knob_position_last = 0
        self.knob_position_delta = 0
        self.position_change = 0.0
        self.gripper_force = 0.0

        # Create publishers and subscribers
        self.gripper_command_pub = self.create_publisher(
            GripperCommand, 
            '/wsg50/command/move', 
            10
        )

        self.knob_force_pub = self.create_publisher(
            Float32,
            '/knob_force_command',
            10
        )

        self.knob_state_sub = self.create_subscription(
            Int32,
            '/knob_state',
            self.knob_state_callback,
            10
        )

        self.gripper_position_sub = self.create_subscription(
            Float32,
            '/wsg50/position',
            self.gripper_position_callback,
            10
        )

        self.force_sub = self.create_subscription(
            Float32,
            '/gripper_finger_force',
            self.force_callback,
            10
        )

        # Create timer for main control loop
        control_period = 1.0 / self.get_parameter('control_rate').value
        self.timer = self.create_timer(control_period, self.control_loop)

        self.get_logger().info('WSG50 Controller initialized')
        self.get_logger().info(f'Control rate: {self.get_parameter("control_rate").value} Hz')
        self.get_logger().info(f'Gripper width range: {self.get_parameter("gripper_min_width").value} - {self.get_parameter("gripper_max_width").value} mm')

    def knob_state_callback(self, msg):
        self.knob_position_current = msg.data
        self.knob_position_delta = self.knob_position_current - self.knob_position_last

        position_jump_threshold = self.get_parameter('position_jump_threshold').value
        if abs(self.knob_position_delta) > position_jump_threshold:
            self.get_logger().warn(f'Large position jump detected: {self.knob_position_delta}')
            self.knob_position_delta = 0
        else:
            position_factor = self.get_parameter('position_factor').value
            self.position_change = -1 * position_factor * self.knob_position_delta

        self.knob_position_last = self.knob_position_current

    def gripper_position_callback(self, msg):
        if msg.data is not None:
            self.gripper_position_current = msg.data

    def force_callback(self, msg):
        self.gripper_force = msg.data

    def human_feedback_force(self, f_gripper, a=2.085, b=1.0, max_output=5.0):
        f = max(f_gripper, 0.0)
        f_human = a * math.log(1 + b * f)
        return min(f_human, max_output)

    def force_feedback(self):
        force_feedback_ratio = self.get_parameter('force_feedback_ratio').value
        force_offset = self.get_parameter('force_offset').value
        force_threshold_max = self.get_parameter('clamp_force_threshold_max').value
        force_threshold_min = self.get_parameter('clamp_force_threshold_min').value
        force_deadzone = self.get_parameter('force_deadzone').value

        force_feedback = self.human_feedback_force(force_feedback_ratio * (self.gripper_force - force_offset))

        if abs(force_feedback) < force_deadzone:
            force_feedback = 0.0

        if force_feedback > 0:
            force_feedback = min(force_threshold_max, max(force_threshold_min, force_feedback))
        else:
            force_feedback = max(-force_threshold_max, min(-force_threshold_min, force_feedback))

        force_msg = Float32()
        force_msg.data = float(-1 * force_feedback)
        self.knob_force_pub.publish(force_msg)
        self.get_logger().debug(f'Force feedback: {force_feedback:.2f} N')

    def compliance_controller(self, x_nominal, f_ext):
        k = self.get_parameter('gripper_compliance_parameter').value
        f_thresh = 0.4
        offset = k * max(f_ext - f_thresh, 0.0)
        return x_nominal + offset

    def control_loop(self):
        self.force_feedback()

        # Update user-commanded nominal position
        self.gripper_position_nominal -= self.position_change

        # Compute compliant position (without modifying nominal)
        x_nominal = self.gripper_position_nominal
        x_compliant = self.compliance_controller(x_nominal, self.gripper_force)

        # Clamp compliant command within limits
        gripper_min = self.get_parameter('gripper_min_width').value
        gripper_max = self.get_parameter('gripper_max_width').value
        self.gripper_position_command = max(gripper_min, min(gripper_max, x_compliant))

        # Publish command
        gripper_cmd = GripperCommand()
        gripper_cmd.position = self.gripper_position_command
        gripper_cmd.velocity = self.get_parameter('gripper_velocity').value
        self.gripper_command_pub.publish(gripper_cmd)

        # Logging
        if abs(self.position_change) > 0 or self.gripper_force > 0.4:
            self.get_logger().info(f'Δpos: {self.position_change:.2f} mm | force: {self.gripper_force:.2f} N')
            self.get_logger().info(f'Nominal: {self.gripper_position_nominal:.2f} mm | Commanded: {self.gripper_position_command:.2f} mm')


def main(args=None):
    rclpy.init(args=args)
    try:
        controller = WSG50TeleoperationInterface()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

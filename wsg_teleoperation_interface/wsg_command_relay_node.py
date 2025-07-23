#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from wsg50_ros_driver.msg import GripperCommand

class GripperRelay(Node):
    def __init__(self):
        super().__init__('gripper_relay')
        self.create_subscription(Float32, '/wsg50/command/move_deploy', self.float_cb, 10)
        self.pub = self.create_publisher(GripperCommand, '/wsg50/command/move', 10)
        self.get_logger().info("Gripper relay started")
        self.default_speed = 20.0  # 可参数化

    def float_cb(self, msg):
        cmd = GripperCommand()
        cmd.position = msg.data
        cmd.velocity = self.default_speed
        self.pub.publish(cmd)
        self.get_logger().info(f"Relayed gripper width: {msg.data} (speed: {self.default_speed})")

def main(args=None):
    rclpy.init(args=args)
    node = GripperRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

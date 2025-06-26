#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import math

class ApriltagTeleoperationNode(Node):
    def __init__(self):
        super().__init__('apriltag_teleoperation_node')
        self.get_logger().info('Apriltag Teleoperation Node Started')

        self.current_tcp_pose = None
        self.last_target_pose = None
        self.alpha = 0.15  # 平滑系数
        self.max_delta_position = 0.10  # 最大允许poly跳变，单位：米
        self.max_delta_angle = 0.35     # 最大允许poly跳变，单位：弧度（约20°）

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            PoseStamped,
            '/tcp_pose_broadcaster/pose',
            self.tcp_pose_callback,
            10
        )
        self.target_publisher = self.create_publisher(
            PoseStamped,
            '/cartesian_compliance_controller/target_frame',
            10
        )
        self.timer = self.create_timer(0.02, self.timer_callback)

    def tcp_pose_callback(self, msg):
        self.current_tcp_pose = msg

    def pose_distance(self, p1, p2):
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        dz = p1.z - p2.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def quaternion_distance(self, q1, q2):
        dot = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]
        dot = max(-1.0, min(1.0, dot))
        angle = 2 * math.acos(abs(dot))
        return angle

    def timer_callback(self):
        if self.current_tcp_pose is None:
            return

        try:
            tf = self.tf_buffer.lookup_transform('base', 'poly', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'No tf from base to poly: {e}')
            return

        virtual_pose = PoseStamped()
        virtual_pose.header.frame_id = 'poly'
        virtual_pose.header.stamp = self.get_clock().now().to_msg()

        # 先构造单位姿态（identity）
        virtual_pose.pose.position.x = 0.0
        virtual_pose.pose.position.y = 0.0
        virtual_pose.pose.position.z = 0.0
        virtual_pose.pose.orientation.x = 0.0
        virtual_pose.pose.orientation.y = 0.0
        virtual_pose.pose.orientation.z = 0.0
        virtual_pose.pose.orientation.w = 1.0

        # 使用官方 tf2 方法进行变换
        try:
            poly_in_base = do_transform_pose(virtual_pose, tf)
        except Exception as e:
            self.get_logger().warn(f'do_transform_pose failed: {e}')
            return

        # == 跳变抑制 ==
        ignore_frame = False
        if self.last_target_pose is not None:
            dist = self.pose_distance(
                poly_in_base.pose.position,
                self.last_target_pose.pose.position
            )
            q1 = [
                poly_in_base.pose.orientation.x,
                poly_in_base.pose.orientation.y,
                poly_in_base.pose.orientation.z,
                poly_in_base.pose.orientation.w,
            ]
            q2 = [
                self.last_target_pose.pose.orientation.x,
                self.last_target_pose.pose.orientation.y,
                self.last_target_pose.pose.orientation.z,
                self.last_target_pose.pose.orientation.w,
            ]
            angle = self.quaternion_distance(q1, q2)
            if dist > self.max_delta_position or angle > self.max_delta_angle:
                ignore_frame = True
                self.get_logger().warn(
                    f'Poly pose jump detected: Δpos={dist:.3f}m, Δrot={math.degrees(angle):.1f}°; skip this frame.'
                )
        self.last_target_pose = poly_in_base

        if ignore_frame:
            return

        # == 增量控制 ==
        delta_pos = [
        poly_in_base.pose.position.x - self.last_poly_pose.pose.position.x,
        poly_in_base.pose.position.y - self.last_poly_pose.pose.position.y,
        poly_in_base.pose.position.z - self.last_poly_pose.pose.position.z,
    ]

        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.current_tcp_pose.header.frame_id
        target.pose.position.x = self.current_tcp_pose.pose.position.x + delta_pos[0]
        target.pose.position.y = self.current_tcp_pose.pose.position.y + delta_pos[1]
        target.pose.position.z = self.current_tcp_pose.pose.position.z + delta_pos[2]
        target.pose.orientation = poly_in_base.pose.orientation  # orientation 直接替换（若 frame 已对齐）

        self.target_publisher.publish(target)

    def align_to_apriltag(self):
        try:
            tf = self.tf_buffer.lookup_transform('base', 'apriltag', rclpy.time.Time())
            virtual_pose = PoseStamped()
            virtual_pose.header.frame_id = 'apriltag'
            virtual_pose.header.stamp = self.get_clock().now().to_msg()
            virtual_pose.pose.position.x = 0.0
            virtual_pose.pose.position.y = 0.0
            virtual_pose.pose.position.z = 0.0
            virtual_pose.pose.orientation.x = 0.0
            virtual_pose.pose.orientation.y = 0.0
            virtual_pose.pose.orientation.z = 0.0
            virtual_pose.pose.orientation.w = 1.0
            aligned_pose = do_transform_pose(virtual_pose, tf)
            aligned_pose.header.stamp = self.get_clock().now().to_msg()
            self.target_publisher.publish(aligned_pose)
            self.get_logger().info('Aligned to apriltag pose.')
        except Exception as e:
            self.get_logger().warn(f'Failed to align to apriltag: {e}')

def main():
    rclpy.init()
    node = ApriltagTeleoperationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
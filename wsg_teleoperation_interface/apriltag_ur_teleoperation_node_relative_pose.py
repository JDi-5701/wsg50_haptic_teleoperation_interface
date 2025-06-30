#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf_transformations  # 替换 tf2_geometry_msgs
# from tf2_geometry_msgs import do_transform_pose
import math

class ApriltagTeleoperationNode(Node):
    def __init__(self):
        super().__init__('apriltag_teleoperation_node')
        self.get_logger().info('Apriltag Teleoperation Node Started')

        self.current_tcp_pose = None
        self.last_target_pose = None
        self.last_poly_pose = None  # <-- [ADDED] For incremental delta computation
        self.alpha = 0.15
        self.max_delta_position = 0.2
        self.max_delta_angle = 0.35

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
            tf = self.tf_buffer.lookup_transform('workspace_middle', 'poly', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'No tf from workspace_middle to poly: {e}')
            return

        virtual_pose = PoseStamped()
        virtual_pose.header.frame_id = 'poly'
        virtual_pose.header.stamp = self.get_clock().now().to_msg()
        virtual_pose.pose.position.x = 0.0
        virtual_pose.pose.position.y = 0.0
        virtual_pose.pose.position.z = 0.0
        virtual_pose.pose.orientation.x = 0.0
        virtual_pose.pose.orientation.y = 0.0
        virtual_pose.pose.orientation.z = 0.0
        virtual_pose.pose.orientation.w = 1.0

        try:
            # === 类型检查 start ===
            #self.get_logger().info(f"[CHECK] virtual_pose: {type(virtual_pose)} | has .pose: {'pose' in dir(virtual_pose)} | .pose type: {type(getattr(virtual_pose, 'pose', None))}")
            #self.get_logger().info(f"[CHECK] tf: {type(tf)} | has .transform: {'transform' in dir(tf)}")
            #self.get_logger().info(f"[CHECK] virtual_pose as dict: {vars(virtual_pose) if hasattr(virtual_pose, '__dict__') else str(virtual_pose)}")
            # === 类型检查 end ===
            poly_in_base = manual_transform_pose(virtual_pose, tf)  # <--- 替换为自定义变换函数
        except Exception as e:
            self.get_logger().warn(f'do_transform_pose failed: {e}')
            return

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

        # <-- [ADDED] Prevent use of uninitialized last_poly_pose
        if self.last_poly_pose is None:
            self.last_poly_pose = poly_in_base
            return

        # ==== 增量位置 ====
        delta_pos = [
            poly_in_base.pose.position.x - self.last_poly_pose.pose.position.x,
            poly_in_base.pose.position.y - self.last_poly_pose.pose.position.y,
            poly_in_base.pose.position.z - self.last_poly_pose.pose.position.z,
        ]

        # ==== 增量姿态 ====
        def quat_list(msg):
            return [msg.x, msg.y, msg.z, msg.w]
        current_q = quat_list(self.current_tcp_pose.pose.orientation)
        poly_q = quat_list(poly_in_base.pose.orientation)
        last_poly_q = quat_list(self.last_poly_pose.pose.orientation)
        last_poly_q_inv = tf_transformations.quaternion_inverse(last_poly_q)
        delta_q = tf_transformations.quaternion_multiply(poly_q, last_poly_q_inv)
        target_q = tf_transformations.quaternion_multiply(delta_q, current_q)

        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.current_tcp_pose.header.frame_id
        target.pose.position.x = self.current_tcp_pose.pose.position.x + delta_pos[0]
        target.pose.position.y = self.current_tcp_pose.pose.position.y + delta_pos[1]
        target.pose.position.z = self.current_tcp_pose.pose.position.z + delta_pos[2]
        target.pose.orientation.x = target_q[0]
        target.pose.orientation.y = target_q[1]
        target.pose.orientation.z = target_q[2]
        target.pose.orientation.w = target_q[3]

        self.target_publisher.publish(target)
        self.last_poly_pose = poly_in_base  # <-- [ADDED] Update for next delta computation

        delta_norm = math.sqrt(sum([d*d for d in delta_pos]))
        delta_angle = 2 * math.acos(min(1.0, abs(delta_q[3])))
        print(f"Δpos: x={delta_pos[0]:.6f}, y={delta_pos[1]:.6f}, z={delta_pos[2]:.6f}, norm={delta_norm:.8f}")
        print(f"Δquat: x={delta_q[0]:.6f}, y={delta_q[1]:.6f}, z={delta_q[2]:.6f}, w={delta_q[3]:.6f}, angle={math.degrees(delta_angle):.8f}°")


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
            aligned_pose = manual_transform_pose(virtual_pose, tf)  # <--- 替换为自定义变换函数
            aligned_pose.header.stamp = self.get_clock().now().to_msg()
            self.target_publisher.publish(aligned_pose)
            self.get_logger().info('Aligned to apriltag pose.')
        except Exception as e:
            self.get_logger().warn(f'Failed to align to apriltag: {e}')

# ===== 自定义 pose 变换函数 start =====
def manual_transform_pose(pose_stamped, transform_stamped):
    t = transform_stamped.transform.translation
    q = transform_stamped.transform.rotation
    pose_q = [
        pose_stamped.pose.orientation.x,
        pose_stamped.pose.orientation.y,
        pose_stamped.pose.orientation.z,
        pose_stamped.pose.orientation.w,
    ]
    pose_t = [
        pose_stamped.pose.position.x,
        pose_stamped.pose.position.y,
        pose_stamped.pose.position.z
    ]
    pose_mat = tf_transformations.quaternion_matrix(pose_q)
    pose_mat[0:3, 3] = pose_t
    trans_q = [q.x, q.y, q.z, q.w]
    trans_t = [t.x, t.y, t.z]
    trans_mat = tf_transformations.quaternion_matrix(trans_q)
    trans_mat[0:3, 3] = trans_t
    result_mat = trans_mat @ pose_mat
    new_pos = result_mat[0:3, 3]
    new_quat = tf_transformations.quaternion_from_matrix(result_mat)
    res = PoseStamped()
    res.header = transform_stamped.header
    res.pose.position.x = new_pos[0]
    res.pose.position.y = new_pos[1]
    res.pose.position.z = new_pos[2]
    res.pose.orientation.x = new_quat[0]
    res.pose.orientation.y = new_quat[1]
    res.pose.orientation.z = new_quat[2]
    res.pose.orientation.w = new_quat[3]
    return res
# ===== 自定义 pose 变换函数 end =====

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

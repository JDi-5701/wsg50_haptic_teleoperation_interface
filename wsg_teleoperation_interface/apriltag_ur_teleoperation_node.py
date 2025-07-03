#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf_transformations  # 替换 tf2_geometry_msgs
import math

class ApriltagTeleoperationNode(Node):
    def __init__(self, use_absolute_pose=True):
        super().__init__('apriltag_teleoperation_node')
        self.get_logger().info('Apriltag Teleoperation Node Started')

        self.current_tcp_pose = None
        self.last_target_pose = None
        self.last_poly_pose = None
        self.initial_poly_pose = None  # <--- 新增
        self.alpha = 0.15
        self.max_delta_position = 0.40
        self.max_delta_angle = 0.35
        self.use_absolute_pose = use_absolute_pose  # <--- 新增，选择模式
        self.start_tcp_pose = None  # <--- 新增，绝对模式参考

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
            '/cartesian_compliance_controller/safe_interface_frame',
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

        if not hasattr(self, 'goal_pose') or self.goal_pose is None:
            self.goal_pose = PoseStamped()
            self.goal_pose.header = self.current_tcp_pose.header
            self.goal_pose.pose.position.x = self.current_tcp_pose.pose.position.x
            self.goal_pose.pose.position.y = self.current_tcp_pose.pose.position.y
            self.goal_pose.pose.position.z = self.current_tcp_pose.pose.position.z
            self.goal_pose.pose.orientation.x = self.current_tcp_pose.pose.orientation.x
            self.goal_pose.pose.orientation.y = self.current_tcp_pose.pose.orientation.y
            self.goal_pose.pose.orientation.z = self.current_tcp_pose.pose.orientation.z
            self.goal_pose.pose.orientation.w = self.current_tcp_pose.pose.orientation.w

        try:
            tf = self.tf_buffer.lookup_transform('usb_cam', 'poly', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'No tf from base to poly: {e}')
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
            poly_in_base = manual_transform_pose(virtual_pose, tf)
        except Exception as e:
            self.get_logger().warn(f'do_transform_pose failed: {e}')
            return

        # ========== 跳变安全检测 ========== #
        if self.last_poly_pose is not None:
            jump_pos = self.pose_distance(
                poly_in_base.pose.position,
                self.last_poly_pose.pose.position
            )
            jump_ori = self.quaternion_distance(
                [poly_in_base.pose.orientation.x, poly_in_base.pose.orientation.y, poly_in_base.pose.orientation.z, poly_in_base.pose.orientation.w],
                [self.last_poly_pose.pose.orientation.x, self.last_poly_pose.pose.orientation.y, self.last_poly_pose.pose.orientation.z, self.last_poly_pose.pose.orientation.w]
            )
            if jump_pos > self.max_delta_position or jump_ori > self.max_delta_angle:
                self.get_logger().warn(f'TF jump detected: Δpos={jump_pos:.3f}, Δori={jump_ori:.3f}, skipping frame')
                return

        if self.last_poly_pose is None:
            self.last_poly_pose = poly_in_base
            self.initial_poly_pose = poly_in_base  # 只在首次保存
            self.start_tcp_pose = self.current_tcp_pose  # 绝对模式参考
            return

        # =============== 新增 delta pose 计算 ===============
        poly_pose_current = get_delta_pose(self.initial_poly_pose, poly_in_base)
        poly_pose_last = get_delta_pose(self.initial_poly_pose, self.last_poly_pose)

        if self.use_absolute_pose:
            # 绝对模式：每帧 goal_pose 直接 = 初始tcp_pose 叠加 poly_pose_current
            self.goal_pose = apply_delta_to_pose(self.start_tcp_pose, poly_pose_current)
            self.goal_pose.header.stamp = self.get_clock().now().to_msg()
            self.goal_pose.header.frame_id = self.current_tcp_pose.header.frame_id
        else:
            # ==== 增量位置 ====
            delta_pos = [
                poly_pose_current.pose.position.x - poly_pose_last.pose.position.x,
                poly_pose_current.pose.position.y - poly_pose_last.pose.position.y,
                poly_pose_current.pose.position.z - poly_pose_last.pose.position.z,
            ]

            # ==== 增量姿态 ====
            def quat_list(msg):
                return [msg.x, msg.y, msg.z, msg.w]
            goal_q = quat_list(self.goal_pose.pose.orientation)
            poly_q = quat_list(poly_pose_current.pose.orientation)
            last_poly_q = quat_list(poly_pose_last.pose.orientation)
            last_poly_q_inv = tf_transformations.quaternion_inverse(last_poly_q)
            delta_q = tf_transformations.quaternion_multiply(poly_q, last_poly_q_inv)
            goal_q_new = tf_transformations.quaternion_multiply(delta_q, goal_q)

            # == 累加到 goal_pose（不依赖 current_tcp_pose，仅初始化用）==
            self.goal_pose.pose.position.x += delta_pos[0]
            self.goal_pose.pose.position.y += delta_pos[1]
            self.goal_pose.pose.position.z += delta_pos[2]
            self.goal_pose.pose.orientation.x = goal_q_new[0]
            self.goal_pose.pose.orientation.y = goal_q_new[1]
            self.goal_pose.pose.orientation.z = goal_q_new[2]
            self.goal_pose.pose.orientation.w = goal_q_new[3]
            self.goal_pose.header.stamp = self.get_clock().now().to_msg()
            self.goal_pose.header.frame_id = self.current_tcp_pose.header.frame_id

        self.target_publisher.publish(self.goal_pose)
        self.last_poly_pose = poly_in_base

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
            aligned_pose = manual_transform_pose(virtual_pose, tf)
            aligned_pose.header.stamp = self.get_clock().now().to_msg()
            self.target_publisher.publish(aligned_pose)
            self.get_logger().info('Aligned to apriltag pose.')
        except Exception as e:
            self.get_logger().warn(f'Failed to align to apriltag: {e}')

# ===== 新增 delta pose 计算函数 =====
def get_delta_pose(pose_A, pose_B):
    """
    返回 pose_A 到 pose_B 的相对变换（A^{-1} * B），frame_id 取 pose_A
    """
    # pose_A, pose_B 都是 PoseStamped
    def to_mat(pose):
        q = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
        t = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        m = tf_transformations.quaternion_matrix(q)
        m[0:3, 3] = t
        return m
    mat_A = to_mat(pose_A)
    mat_B = to_mat(pose_B)
    delta_mat = tf_transformations.inverse_matrix(mat_A) @ mat_B
    new_pos = delta_mat[0:3, 3]
    new_quat = tf_transformations.quaternion_from_matrix(delta_mat)
    res = PoseStamped()
    res.header = pose_A.header
    res.pose.position.x = new_pos[0]
    res.pose.position.y = new_pos[1]
    res.pose.position.z = new_pos[2]
    res.pose.orientation.x = new_quat[0]
    res.pose.orientation.y = new_quat[1]
    res.pose.orientation.z = new_quat[2]
    res.pose.orientation.w = new_quat[3]
    return res
# ===== end =====

# ===== 新增 base pose 叠加 delta 的函数 =====
def apply_delta_to_pose(pose_base, delta_pose):
    """
    输入: pose_base (PoseStamped), delta_pose (PoseStamped)
    返回: pose_base 在 SE(3) 上右乘 delta_pose 后的新 PoseStamped
    """
    q_base = [pose_base.pose.orientation.x, pose_base.pose.orientation.y, pose_base.pose.orientation.z, pose_base.pose.orientation.w]
    t_base = [pose_base.pose.position.x, pose_base.pose.position.y, pose_base.pose.position.z]
    m_base = tf_transformations.quaternion_matrix(q_base)
    m_base[0:3, 3] = t_base

    q_delta = [delta_pose.pose.orientation.x, delta_pose.pose.orientation.y, delta_pose.pose.orientation.z, delta_pose.pose.orientation.w]
    t_delta = [delta_pose.pose.position.x, delta_pose.pose.position.y, delta_pose.pose.position.z]
    m_delta = tf_transformations.quaternion_matrix(q_delta)
    m_delta[0:3, 3] = t_delta

    m_new = m_base @ m_delta
    new_pos = m_new[0:3, 3]
    new_quat = tf_transformations.quaternion_from_matrix(m_new)

    res = PoseStamped()
    res.header = pose_base.header
    res.pose.position.x = new_pos[0]
    res.pose.position.y = new_pos[1]
    res.pose.position.z = new_pos[2]
    res.pose.orientation.x = new_quat[0]
    res.pose.orientation.y = new_quat[1]
    res.pose.orientation.z = new_quat[2]
    res.pose.orientation.w = new_quat[3]
    return res
# ===== end =====

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
    # 可根据需要更改参数，True 为绝对模式，False 为增量累加模式
    node = ApriltagTeleoperationNode(use_absolute_pose=False)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

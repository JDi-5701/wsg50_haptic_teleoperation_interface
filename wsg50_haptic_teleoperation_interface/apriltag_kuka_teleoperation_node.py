#!/usr/bin/env python3
"""AprilTag -> KUKA iiwa 6D TCP teleoperation.

The UR version (apriltag_ur_teleoperation_node.py) works, so the control law is
carried over unchanged and only the interface is repointed. What it does, and
notably does not do:

    first frame   remember the marker pose in the camera frame, and the TCP pose
    every frame   marker_delta = initial_marker^-1 * marker_now
                  goal         = start_tcp * marker_delta

The marker's displacement from its own starting point becomes the TCP's
displacement from its own starting point. There is NO hand-eye calibration: the
camera frame and the world frame are simply taken as aligned, and the operator
adapts to whatever rotation remains between them. That is why this works with
the coarse intrinsics we have - only the marker's motion relative to itself
matters, not where the camera actually is.

Changed from the UR node:
    /tcp_pose_broadcaster/pose                      -> /world/tcp_pose
    /cartesian_compliance_controller/safe_interface -> /world/tcp_command
    tf usb_cam -> poly                              -> camera_optical_frame -> tag_0
    frame_id on the command                         -> world_frame, which the
        KUKA relay requires; it drops anything else
    motion is scaled by position_scale / orientation_scale, 0.2 by default,
        where the UR ran 1:1. The jump thresholds below are checked on the
        MARKER's motion, before scaling, so they bound the input not the output.
    jump thresholds 0.40 m / 0.35 rad               -> 0.12 m / 0.30 rad, inside
        the relay's own 0.15 m / 0.50 rad limit so a jump is stopped here rather
        than rejected downstream
    tf_transformations                              -> .tf_compat, since that
        package is not installed (see tf_compat.py)

Unchanged: the delta maths, the incremental default, the skip-the-frame
response to a jump.

Standalone:
    ros2 run wsg50_haptic_teleoperation_interface apriltag_kuka_teleoperation_node.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from wsg50_haptic_teleoperation_interface import tf_compat as tf_transformations
import math

class ApriltagTeleoperationNode(Node):
    def __init__(self, use_absolute_pose=True):
        super().__init__('apriltag_kuka_teleoperation_node')
        self.get_logger().info('AprilTag -> KUKA 6D teleoperation started')

        self.current_tcp_pose = None
        self.last_target_pose = None
        self.last_poly_pose = None
        self.initial_poly_pose = None  # <--- 新增
        # Inside the KUKA relay's own 0.15 m / 0.50 rad limit, so a marker jump
        # is stopped here instead of being rejected downstream as a bad command.
        self.declare_parameter('max_delta_position', 0.12)
        self.declare_parameter('max_delta_angle', 0.30)
        # How much of the marker's motion the robot takes. 1.0 is the UR
        # behaviour, motion for motion; smaller trades range for steadiness,
        # and also scales down the hand tremor the marker picks up.
        self.declare_parameter('position_scale', 0.2)
        self.declare_parameter('orientation_scale', 0.2)
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('raw_tag_frame', 'tag_0')
        self.declare_parameter('tag_frame', 'tag_aligned')
        # Which robot axis a marker motion drives is decided by the marker
        # frame's orientation at the zero instant, since the displacement is
        # rotated into it. This node publishes raw_tag_frame -> tag_frame
        # itself, so it needs no external static_transform_publisher.
        #
        # The default is the mapping defined against this hardware:
        #     tag +z -> world +y,  tag +y -> world +z,  tag +x -> world -x
        # a 180 deg rotation about (0,1,1)/sqrt2. As a matrix that is
        # [[-1,0,0],[0,0,1],[0,1,0]]: orthogonal, determinant +1, so a real
        # rotation rather than a mirror. Flipping a single sign would make the
        # determinant -1, which no rigid rotation can produce.
        self.declare_parameter('tag_align_qx', 0.0)
        self.declare_parameter('tag_align_qy', 0.7071068)
        self.declare_parameter('tag_align_qz', 0.7071068)
        self.declare_parameter('tag_align_qw', 0.0)
        self.declare_parameter('command_frame', 'world_frame')
        self.max_delta_position = self.get_parameter('max_delta_position').value
        self.max_delta_angle = self.get_parameter('max_delta_angle').value
        self.position_scale = self.get_parameter('position_scale').value
        self.orientation_scale = self.get_parameter('orientation_scale').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.raw_tag_frame = self.get_parameter('raw_tag_frame').value
        self.tag_frame = self.get_parameter('tag_frame').value
        self.command_frame = self.get_parameter('command_frame').value
        self.use_absolute_pose = use_absolute_pose  # <--- 新增，选择模式
        self.start_tcp_pose = None  # <--- 新增，绝对模式参考

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._publish_alignment()

        self.create_subscription(
            PoseStamped,
            '/world/tcp_pose',
            self.tcp_pose_callback,
            10
        )

        self.target_publisher = self.create_publisher(
            PoseStamped,
            '/world/tcp_command',
            10
        )

        self.timer = self.create_timer(0.02, self.timer_callback)

    def _publish_alignment(self):
        """Publish raw_tag_frame -> tag_frame, a pure rotation."""
        if self.tag_frame == self.raw_tag_frame:
            self.get_logger().info(
                f'No axis remap: reading {self.tag_frame} directly.')
            return
        q = [self.get_parameter(f'tag_align_q{a}').value for a in 'xyzw']
        norm = math.sqrt(sum(v * v for v in q))
        if norm < 1e-9:
            raise RuntimeError('tag_align_q* is a zero quaternion')
        q = [v / norm for v in q]

        self._static_tf = StaticTransformBroadcaster(self)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.raw_tag_frame
        tf_msg.child_frame_id = self.tag_frame
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]
        self._static_tf.sendTransform(tf_msg)
        self.get_logger().info(
            f'Axis remap {self.raw_tag_frame} -> {self.tag_frame}, '
            f'q=({q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}, {q[3]:.4f})')

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
            self.goal_pose.header.frame_id = self.command_frame
            self.goal_pose.pose.position.x = self.current_tcp_pose.pose.position.x
            self.goal_pose.pose.position.y = self.current_tcp_pose.pose.position.y
            self.goal_pose.pose.position.z = self.current_tcp_pose.pose.position.z
            self.goal_pose.pose.orientation.x = self.current_tcp_pose.pose.orientation.x
            self.goal_pose.pose.orientation.y = self.current_tcp_pose.pose.orientation.y
            self.goal_pose.pose.orientation.z = self.current_tcp_pose.pose.orientation.z
            self.goal_pose.pose.orientation.w = self.current_tcp_pose.pose.orientation.w

        try:
            tf = self.tf_buffer.lookup_transform(
                self.camera_frame, self.tag_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(
                f'No tf {self.camera_frame} -> {self.tag_frame}: {e}',
                throttle_duration_sec=2.0)
            return

        virtual_pose = PoseStamped()
        virtual_pose.header.frame_id = self.tag_frame
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
            self.goal_pose = apply_delta_to_pose(
                self.start_tcp_pose,
                scale_delta_pose(poly_pose_current, self.position_scale,
                                 self.orientation_scale))
            self.goal_pose.header.stamp = self.get_clock().now().to_msg()
            self.goal_pose.header.frame_id = self.command_frame
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
            delta_q = scale_quaternion(delta_q, self.orientation_scale)
            goal_q_new = tf_transformations.quaternion_multiply(delta_q, goal_q)

            # == 累加到 goal_pose（不依赖 current_tcp_pose，仅初始化用）==
            self.goal_pose.pose.position.x += self.position_scale * delta_pos[0]
            self.goal_pose.pose.position.y += self.position_scale * delta_pos[1]
            self.goal_pose.pose.position.z += self.position_scale * delta_pos[2]
            self.goal_pose.pose.orientation.x = goal_q_new[0]
            self.goal_pose.pose.orientation.y = goal_q_new[1]
            self.goal_pose.pose.orientation.z = goal_q_new[2]
            self.goal_pose.pose.orientation.w = goal_q_new[3]
            self.goal_pose.header.stamp = self.get_clock().now().to_msg()
            self.goal_pose.header.frame_id = self.command_frame

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
def scale_quaternion(q, scale):
    """Take `scale` of the rotation q, via axis-angle.

    A quaternion cannot be scaled by multiplying it: that is not a smaller
    rotation, it is an unnormalised quaternion. Shrinking the angle about the
    same axis is what "20% of this rotation" actually means.
    """
    if scale == 1.0:
        return q
    x, y, z, w = q
    w = max(-1.0, min(1.0, w))
    angle = 2.0 * math.acos(w)
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-9 or abs(angle) < 1e-9:
        return [0.0, 0.0, 0.0, 1.0]          # no rotation to scale
    axis = [x / s, y / s, z / s]
    half = (angle * scale) / 2.0
    sh = math.sin(half)
    return [axis[0] * sh, axis[1] * sh, axis[2] * sh, math.cos(half)]


def scale_delta_pose(delta_pose, position_scale, orientation_scale):
    """Scale a delta transform: translation linearly, rotation by angle."""
    res = PoseStamped()
    res.header = delta_pose.header
    res.pose.position.x = delta_pose.pose.position.x * position_scale
    res.pose.position.y = delta_pose.pose.position.y * position_scale
    res.pose.position.z = delta_pose.pose.position.z * position_scale
    q = scale_quaternion(
        [delta_pose.pose.orientation.x, delta_pose.pose.orientation.y,
         delta_pose.pose.orientation.z, delta_pose.pose.orientation.w],
        orientation_scale)
    res.pose.orientation.x = q[0]
    res.pose.orientation.y = q[1]
    res.pose.orientation.z = q[2]
    res.pose.orientation.w = q[3]
    return res


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
    node = ApriltagTeleoperationNode(use_absolute_pose=False)   # as on the UR
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

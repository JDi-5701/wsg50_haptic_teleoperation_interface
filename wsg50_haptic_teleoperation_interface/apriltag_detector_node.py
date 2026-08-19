#!/usr/bin/env python3
"""AprilTag detection and 6D pose, on cv2.aruco.

OpenCV 4.2 ships the AprilTag families in cv2.aruco, so this needs no
apriltag_ros, no apriltag C library and no cv_bridge - none of which are
installed here. Image messages are unpacked by hand.

A tag backlit against a window underexposes badly. Detection runs on a
gamma-corrected retry when the first pass finds nothing, which recovered 100%
of frames where the raw image managed 14%.

Topics
    subscribes  image_raw/compressed  (CompressedImage)  or image_raw (Image)
                camera_info           (CameraInfo)       intrinsics
    publishes   tag_detections        (PoseArray)   every tag seen this frame
                tag_<id>/pose         (PoseStamped) one per configured tag
                tag_image             (CompressedImage) annotated, for eyeballing
                                                        (only with publish_debug_image)
    broadcasts  TF camera_frame -> tag_<id>

Pose comes from solvePnP over the four corners against the known tag size, so
it is only as good as the intrinsics: orientation holds up under a wrong fx,
but distance scales with it directly. camera_info carries whatever the camera
node had, calibrated or guessed - the camera node logs which.

The pose is of the tag in the CAMERA OPTICAL frame: Z forward out of the lens,
X right, Y down. Getting from there to the robot's world frame needs the
camera's own pose, which is a separate hand-eye problem this node does not
touch.

Standalone:
    ros2 run wsg50_haptic_teleoperation_interface apriltag_detector_node.py
    ros2 run ... apriltag_detector_node.py --ros-args -p tag_size:=0.038
"""

import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from tf2_ros import TransformBroadcaster

FAMILIES = {
    '36h11': cv2.aruco.DICT_APRILTAG_36h11,
    '36h10': cv2.aruco.DICT_APRILTAG_36h10,
    '25h9': cv2.aruco.DICT_APRILTAG_25h9,
    '16h5': cv2.aruco.DICT_APRILTAG_16h5,
}


def rotation_matrix_to_quaternion(r):
    """Shepperd's method: pick the largest diagonal term to stay conditioned.

    Hand-rolled because tf_transformations is not installed.
    """
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


class AprilTagDetectorNode(Node):
    def __init__(self):
        super().__init__('apriltag_detector_node')

        self.declare_parameter('tag_family', '36h11')
        self.declare_parameter('tag_size', 0.087)          # metres, outer black square
        self.declare_parameter('use_compressed', True)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        # Reject a pose whose corners do not reproject onto themselves. A tag
        # read at a steep angle or half out of frame can still decode while its
        # pose is nonsense; this is the cheapest way to catch that.
        self.declare_parameter('max_reprojection_error', 3.0)
        # Gamma applied before detection. <1 lifts shadows, which matters when
        # the tag is backlit: measured against a window, raw frames detected in
        # 14% of frames and gamma 0.45 in 100%. Manual exposure was no help -
        # this camera ignores CAP_PROP_EXPOSURE, its frame mean staying at 110
        # whatever it is set to. 1.0 disables.
        self.declare_parameter('gamma', 1.0)
        # Retried only when the first pass finds nothing, so a well-exposed
        # scene pays nothing for it. 0 disables the retry.
        self.declare_parameter('gamma_fallback', 0.45)

        family = self.get_parameter('tag_family').value
        if family not in FAMILIES:
            raise RuntimeError(
                f'Unknown family {family}; have {sorted(FAMILIES)}')
        self.dictionary = cv2.aruco.Dictionary_get(FAMILIES[family])
        self.params = cv2.aruco.DetectorParameters_create()
        # Corner accuracy is what the pose is made of, so refine with the
        # AprilTag refiner rather than the default contour corners.
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG

        self.tag_size = float(self.get_parameter('tag_size').value)
        self.camera_frame = self.get_parameter('camera_frame').value
        self.publish_debug = self.get_parameter('publish_debug_image').value
        self.max_reproj = float(self.get_parameter('max_reprojection_error').value)

        # Tag corners in the tag's own frame: origin at centre, Z out of the
        # face, ordered to match what detectMarkers returns.
        h = self.tag_size / 2.0
        self.object_points = np.array([
            [-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0],
        ], dtype=np.float32)

        self.camera_matrix = None
        self.dist_coeffs = None

        def build_lut(g):
            if g <= 0 or g == 1.0:
                return None
            return np.array([((i / 255.0) ** g) * 255 for i in range(256)],
                            np.uint8)

        self.lut = build_lut(float(self.get_parameter('gamma').value))
        self.lut_fallback = build_lut(
            float(self.get_parameter('gamma_fallback').value))
        self.fallback_hits = 0

        self.detections_pub = self.create_publisher(PoseArray, 'tag_detections', 10)
        self.debug_pub = self.create_publisher(CompressedImage, 'tag_image', 1)
        self.tag_pose_pubs = {}
        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.get_parameter('publish_tf').value
            else None)

        self.create_subscription(CameraInfo, 'camera_info', self.info_callback, 1)
        if self.get_parameter('use_compressed').value:
            self.create_subscription(
                CompressedImage, 'image_raw/compressed', self.compressed_callback, 1)
            source = 'image_raw/compressed'
        else:
            self.create_subscription(Image, 'image_raw', self.image_callback, 1)
            source = 'image_raw'

        self.frames = 0
        self.hits = 0
        self.rejected = 0
        self.create_timer(5.0, self.report)

        self.get_logger().info(
            f'AprilTag detector up: family {family}, tag {self.tag_size*1000:.0f} mm, '
            f'reading {source}. Waiting for camera_info.')

    # ------------------------------------------------------------------
    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info(
                f'Intrinsics: fx={self.camera_matrix[0,0]:.1f} '
                f'fy={self.camera_matrix[1,1]:.1f} '
                f'cx={self.camera_matrix[0,2]:.1f} cy={self.camera_matrix[1,2]:.1f}')

    def compressed_callback(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            self.process(frame, msg.header)

    def image_callback(self, msg):
        if msg.encoding not in ('bgr8', 'rgb8'):
            self.get_logger().warn(f'Unsupported encoding {msg.encoding}',
                                   throttle_duration_sec=5.0)
            return
        frame = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            frame = frame[:, :, ::-1]
        self.process(frame, msg.header)

    # ------------------------------------------------------------------
    def process(self, frame, header):
        self.frames += 1
        if self.camera_matrix is None:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.lut is not None:
            gray = cv2.LUT(gray, self.lut)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.params)

        if ids is None and self.lut_fallback is not None:
            corners, ids, _ = cv2.aruco.detectMarkers(
                cv2.LUT(gray, self.lut_fallback), self.dictionary,
                parameters=self.params)
            if ids is not None:
                self.fallback_hits += 1

        pose_array = PoseArray()
        pose_array.header.stamp = header.stamp
        pose_array.header.frame_id = self.camera_frame

        debug = frame.copy() if self.publish_debug else None

        if ids is not None:
            for marker_corners, tag_id in zip(corners, ids.flatten()):
                image_points = marker_corners.reshape(4, 2).astype(np.float32)
                ok, rvec, tvec = cv2.solvePnP(
                    self.object_points, image_points,
                    self.camera_matrix, self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok:
                    continue

                reproj, _ = cv2.projectPoints(
                    self.object_points, rvec, tvec,
                    self.camera_matrix, self.dist_coeffs)
                err = float(np.mean(np.linalg.norm(
                    reproj.reshape(4, 2) - image_points, axis=1)))
                if err > self.max_reproj:
                    self.rejected += 1
                    continue

                rot, _ = cv2.Rodrigues(rvec)
                qx, qy, qz, qw = rotation_matrix_to_quaternion(rot)
                t = tvec.flatten()

                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = (
                    float(t[0]), float(t[1]), float(t[2]))
                pose.orientation.x = float(qx)
                pose.orientation.y = float(qy)
                pose.orientation.z = float(qz)
                pose.orientation.w = float(qw)
                pose_array.poses.append(pose)
                self.hits += 1

                self._publish_tag(int(tag_id), pose, header.stamp)

                if debug is not None:
                    cv2.aruco.drawDetectedMarkers(debug, [marker_corners],
                                                  np.array([[tag_id]]))
                    cv2.aruco.drawAxis(debug, self.camera_matrix, self.dist_coeffs,
                                       rvec, tvec, self.tag_size * 0.75)
                    c = image_points.mean(axis=0).astype(int)
                    cv2.putText(
                        debug,
                        f'id{tag_id} {np.linalg.norm(t)*1000:.0f}mm e={err:.2f}px',
                        (c[0] - 70, c[1] - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 2)

        self.detections_pub.publish(pose_array)

        if debug is not None:
            ok, buf = cv2.imencode('.jpg', debug,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ok:
                msg = CompressedImage()
                msg.header.stamp = header.stamp
                msg.header.frame_id = self.camera_frame
                msg.format = 'jpeg'
                msg.data = buf.tobytes()
                self.debug_pub.publish(msg)

    def _publish_tag(self, tag_id, pose, stamp):
        if tag_id not in self.tag_pose_pubs:
            self.tag_pose_pubs[tag_id] = self.create_publisher(
                PoseStamped, f'tag_{tag_id}/pose', 10)
            self.get_logger().info(f'First sight of tag {tag_id}')

        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.camera_frame
        ps.pose = pose
        self.tag_pose_pubs[tag_id].publish(ps)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.camera_frame
            tf.child_frame_id = f'tag_{tag_id}'
            tf.transform.translation.x = pose.position.x
            tf.transform.translation.y = pose.position.y
            tf.transform.translation.z = pose.position.z
            tf.transform.rotation = pose.orientation
            self.tf_broadcaster.sendTransform(tf)

    def report(self):
        if self.camera_matrix is None:
            self.get_logger().warn('Still no camera_info - is the camera node up?')
            return
        note = f', {self.rejected} rejected on reprojection' if self.rejected else ''
        if self.fallback_hits:
            note += (f', {self.fallback_hits} only found after gamma - the '
                     f'scene is underexposing the tag')
        self.get_logger().info(
            f'{self.frames} frames, {self.hits} tag sightings{note} (last 5 s)')
        self.frames = self.hits = self.rejected = self.fallback_hits = 0


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AprilTagDetectorNode()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f'[apriltag_detector_node] {e}')
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

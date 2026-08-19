#!/usr/bin/env python3
"""USB camera -> ROS image topics, via OpenCV's V4L2 backend.

Written against plain cv2 rather than usb_cam/image_transport/cv_bridge,
none of which are installed here (this machine has ros-foxy-ros-base only).
Image messages are packed by hand, which is a few lines and costs nothing.

Topics
    publishes   image_raw               (Image)            bgr8
                image_raw/compressed    (CompressedImage)  jpeg
                camera_info             (CameraInfo)

Raw frames are large and DDS is the bottleneck, not the camera: capture, JPEG
encode and serialisation each sustain 30 Hz, but publishing 1280x720 bgr8
(2.76 MB a frame) measured 3.8 Hz end to end. So publish_raw defaults to false
and the default mode is 640x480 - the compressed topic carries the same pixels
and the detector reads that. Turn publish_raw on only for a tool that needs it,
and expect the rate to drop.

CALIBRATION: with no calibration file the intrinsics are guessed from
`vertical_fov_deg`, and the log says so. Vertical, because this sensor holds
the vertical field constant and crops horizontally for 4:3 modes - measured,
not assumed. That is fine for detecting a tag and
roughly right for its orientation, but the distance to the tag scales directly
with fx - an fx that is 10% off puts the tag 10% too near or too far. Run
calibrate_camera.py with a printed checkerboard and pass the result through
`camera_info_url` before trusting any translation.

Standalone:
    ros2 run wsg50_haptic_teleoperation_interface usb_camera_node.py
    ros2 run ... usb_camera_node.py --ros-args -p width:=1920 -p height:=1080
"""

import os

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class UsbCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')

        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        # MJPG is what lets this sensor reach 720p and above at 30 fps; YUYV
        # caps out much lower.
        self.declare_parameter('fourcc', 'MJPG')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        # Off by default. Capture, JPEG encode and serialisation all sustain
        # 30 Hz, but pushing raw frames through DDS does not: 1280x720 bgr8
        # is 2.76 MB a frame and measured 3.8 Hz end to end. The compressed
        # topic carries the same pixels for a fraction of the bandwidth.
        self.declare_parameter('publish_raw', False)
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('jpeg_quality', 85)
        self.declare_parameter('camera_info_url', '')
        # VERTICAL, not horizontal. Measured on this sensor: 1920x1080 and
        # 1280x720 see the same field (tag width/image width 0.0964 vs 0.0966),
        # but 640x480 sees 1.33x more per pixel (0.1282) - the 4:3 modes keep
        # the vertical field and crop horizontally. So fx guessed from a fixed
        # horizontal FOV is wrong by 4/3 between aspect ratios, while a fixed
        # vertical FOV predicts the measured ratio (240/360 = 0.667 against
        # 0.664 measured). 43 deg vertical is 70 deg horizontal at 16:9.
        self.declare_parameter('vertical_fov_deg', 43.0)

        device_id = self.get_parameter('device_id').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        fourcc = self.get_parameter('fourcc').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_raw = self.get_parameter('publish_raw').value
        self.publish_compressed = self.get_parameter('publish_compressed').value
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        self.cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f'Cannot open /dev/video{device_id}')
        if fourcc:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError(f'Opened /dev/video{device_id} but cannot read')
        self.height, self.width = frame.shape[:2]
        if (self.width, self.height) != (width, height):
            self.get_logger().warn(
                f'Asked for {width}x{height}, sensor gave '
                f'{self.width}x{self.height}')
        self.get_logger().info(
            f'/dev/video{device_id}: {self.width}x{self.height} @ '
            f'{self.cap.get(cv2.CAP_PROP_FPS):.0f} fps, {fourcc}')

        self.camera_info = self._build_camera_info()

        self.image_pub = self.create_publisher(Image, 'image_raw', 1)
        self.compressed_pub = self.create_publisher(
            CompressedImage, 'image_raw/compressed', 1)
        self.info_pub = self.create_publisher(CameraInfo, 'camera_info', 1)

        self.frame_count = 0
        self.create_timer(1.0 / fps, self.capture)

    # ------------------------------------------------------------------
    def _build_camera_info(self):
        info = CameraInfo()
        info.width = self.width
        info.height = self.height
        info.distortion_model = 'plumb_bob'

        url = self.get_parameter('camera_info_url').value
        if url:
            path = url[7:] if url.startswith('file://') else url
            path = os.path.expanduser(path)
            with open(path) as fh:
                calib = yaml.safe_load(fh)
            k = calib['camera_matrix']['data']
            d = calib['distortion_coefficients']['data']
            cw = calib.get('image_width', self.width)
            ch = calib.get('image_height', self.height)
            if (cw, ch) != (self.width, self.height):
                # Intrinsics are in pixels, so they only apply at the
                # resolution they were measured at. Rescaling is exact for a
                # pure resize, which is what changing the capture mode does.
                if abs((cw / ch) - (self.width / self.height)) > 0.01:
                    self.get_logger().error(
                        f'Calibration aspect {cw}:{ch} differs from capture '
                        f'{self.width}:{self.height}. This sensor crops rather '
                        f'than rescales across aspect ratios, so scaling the '
                        f'intrinsics is WRONG here - recalibrate at the '
                        f'capture resolution.')
                sx, sy = self.width / cw, self.height / ch
                k = [k[0]*sx, k[1]*sx, k[2]*sx,
                     k[3]*sy, k[4]*sy, k[5]*sy,
                     k[6],    k[7],    k[8]]
                self.get_logger().warn(
                    f'Calibration is for {cw}x{ch}, scaling to '
                    f'{self.width}x{self.height}')
            info.k = [float(v) for v in k]
            info.d = [float(v) for v in d]
            self.get_logger().info(f'Loaded calibration from {path}')
        else:
            fov = float(self.get_parameter('vertical_fov_deg').value)
            fy = (self.height / 2.0) / np.tan(np.radians(fov) / 2.0)
            fx = fy                       # square pixels
            cx, cy = self.width / 2.0, self.height / 2.0
            info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.get_logger().warn(
                f'NO CALIBRATION. Guessing fx=fy={fx:.1f} px from a {fov:.0f} '
                f'deg vertical FOV. Tag orientation will be usable, but the '
                f'distance to the tag scales with fx - calibrate before '
                f'trusting it.')

        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [info.k[0], 0.0, info.k[2], 0.0,
                  0.0, info.k[4], info.k[5], 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    # ------------------------------------------------------------------
    def capture(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('Frame grab failed', throttle_duration_sec=2.0)
            return

        stamp = self.get_clock().now().to_msg()
        self.camera_info.header.stamp = stamp
        self.camera_info.header.frame_id = self.frame_id
        self.info_pub.publish(self.camera_info)

        if self.publish_raw:
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.height, msg.width = frame.shape[0], frame.shape[1]
            msg.encoding = 'bgr8'
            msg.is_bigendian = 0
            msg.step = frame.shape[1] * 3
            msg.data = frame.tobytes()
            self.image_pub.publish(msg)

        if self.publish_compressed:
            ok, buf = cv2.imencode(
                '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if ok:
                cmsg = CompressedImage()
                cmsg.header.stamp = stamp
                cmsg.header.frame_id = self.frame_id
                cmsg.format = 'jpeg'
                cmsg.data = buf.tobytes()
                self.compressed_pub.publish(cmsg)

        self.frame_count += 1

    def destroy_node(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UsbCameraNode()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f'[usb_camera_node] {e}')
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

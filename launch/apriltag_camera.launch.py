"""USB camera + AprilTag 6D detection.

    usb_camera_node          /dev/videoN -> image_raw/compressed, camera_info
             |
    apriltag_detector_node   -> tag_detections, tag_<id>/pose, TF, tag_image

Defaults to 640x480 with the raw image topic off: capture and JPEG encode both
sustain 30 Hz, but pushing raw bgr8 frames through DDS does not (1280x720
measured 3.8 Hz end to end). The detector reads the compressed topic.

Poses are in the camera optical frame - Z out of the lens, X right, Y down.
Placing them in the robot's world frame needs the camera's own pose, which is a
hand-eye calibration this launch does not do.

Usage:
    ros2 launch wsg50_haptic_teleoperation_interface apriltag_camera.launch.py
    ros2 launch ... apriltag_camera.launch.py tag_size:=0.038 width:=1280 height:=720
    ros2 launch ... apriltag_camera.launch.py camera_info_url:=~/camera_640x480.yaml

Check it is working:
    ros2 topic echo /tag_0/pose
    ros2 topic hz /tag_detections
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device_id', default_value='0',
                              description='/dev/videoN'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='30.0'),
        DeclareLaunchArgument(
            'publish_raw', default_value='false',
            description='Raw bgr8 over DDS is the bottleneck; leave it off '
                        'unless a tool needs the uncompressed topic.'),
        DeclareLaunchArgument(
            'camera_info_url', default_value='',
            description='Calibration YAML from scripts/calibrate_camera.py. '
                        'Empty means intrinsics are guessed from '
                        'vertical_fov_deg and the distance to the tag is only '
                        'as good as that guess.'),
        DeclareLaunchArgument(
            'vertical_fov_deg', default_value='43.0',
            description='Vertical, not horizontal: this sensor keeps the '
                        'vertical field and crops horizontally for 4:3 modes.'),
        DeclareLaunchArgument('tag_family', default_value='36h11'),
        DeclareLaunchArgument('tag_size', default_value='0.038',
                              description='metres, outer edge of the black square'),
        DeclareLaunchArgument('camera_frame', default_value='camera_optical_frame'),
        DeclareLaunchArgument('publish_debug_image', default_value='true',
                              description='Annotated jpeg on tag_image'),

        Node(
            package='wsg50_haptic_teleoperation_interface',
            executable='usb_camera_node.py',
            name='usb_camera_node',
            output='screen',
            parameters=[{
                'device_id': LaunchConfiguration('device_id'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
                'publish_raw': LaunchConfiguration('publish_raw'),
                'publish_compressed': True,
                'camera_info_url': LaunchConfiguration('camera_info_url'),
                'vertical_fov_deg': LaunchConfiguration('vertical_fov_deg'),
                'frame_id': LaunchConfiguration('camera_frame'),
            }],
        ),

        Node(
            package='wsg50_haptic_teleoperation_interface',
            executable='apriltag_detector_node.py',
            name='apriltag_detector_node',
            output='screen',
            parameters=[{
                'tag_family': LaunchConfiguration('tag_family'),
                'tag_size': LaunchConfiguration('tag_size'),
                'use_compressed': True,
                'publish_debug_image': LaunchConfiguration('publish_debug_image'),
                'publish_tf': True,
                'camera_frame': LaunchConfiguration('camera_frame'),
            }],
        ),
    ])

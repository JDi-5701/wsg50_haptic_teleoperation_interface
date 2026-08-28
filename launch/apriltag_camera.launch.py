"""USB camera + AprilTag 6D detection.

    usb_camera_node          /dev/videoN -> image_raw/compressed, camera_info
             |
    apriltag_detector_node   -> tag_detections, tag_<id>/pose, TF, tag_image

Both run in the `namespace` argument's namespace, /webcam by default, so the
topics are /webcam/image_raw/compressed and so on. TF frames are not
namespaced.

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
    ros2 topic echo /webcam/tag_0/pose
    ros2 topic hz /webcam/tag_detections
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='webcam',
            description='Everything this launch publishes lands under it: '
                        '/webcam/image_raw, /webcam/camera_info, '
                        '/webcam/tag_image, /webcam/tag_0/pose. TF frame '
                        'names are NOT namespaced, so the teleop node, which '
                        'reads the marker over TF, is unaffected.'),
        DeclareLaunchArgument('device_id', default_value='0',
                              description='/dev/videoN'),
        DeclareLaunchArgument('width', default_value='800'),
        DeclareLaunchArgument('height', default_value='600'),
        DeclareLaunchArgument('fps', default_value='30.0'),
        DeclareLaunchArgument(
            'auto_exposure', default_value='false',
            description='Auto exposure lengthens the exposure past the frame '
                        'period to brighten the image and the rate collapses '
                        'to 20 fps. Off by default; turn it on only to compare.'),
        DeclareLaunchArgument(
            'exposure', default_value='312',
            description='Units of 100 us, so 312 = 31.2 ms. Capped at the frame '
                        'period (333 at 30 fps) - beyond that the sensor cannot '
                        'hold the rate. Raise it if the tag is underexposed, '
                        'and expect to lose frame rate above the cap.'),
        DeclareLaunchArgument(
            'publish_raw', default_value='false',
            description='Raw bgr8 over DDS is the bottleneck; leave it off '
                        'unless a tool needs the uncompressed topic.'),
        DeclareLaunchArgument(
            'camera_info_url', default_value='/home/fortiss/camera_800x600.yaml',
            description='Coarse intrinsics measured off the tag. Enough for '
                        'orientation and TF; tag distance is +/-10%. Set empty '
                        'to fall back to the vertical_fov_deg guess.'),
        DeclareLaunchArgument(
            'vertical_fov_deg', default_value='43.0',
            description='Vertical, not horizontal: this sensor keeps the '
                        'vertical field and crops horizontally for 4:3 modes.'),
        DeclareLaunchArgument('tag_family', default_value='36h11'),
        DeclareLaunchArgument('tag_size', default_value='0.032',
                              description='metres, outer edge of the black square'),
        DeclareLaunchArgument('camera_frame', default_value='camera_optical_frame'),
        DeclareLaunchArgument('publish_debug_image', default_value='true',
                              description='Annotated jpeg on tag_image'),
        DeclareLaunchArgument(
            'publish_debug_raw', default_value='false',
            description='Also publish the annotated frame uncompressed on '
                        'tag_image_raw. Needed on a machine without '
                        'compressed_image_transport, which cannot subscribe to '
                        'tag_image at all. Costs bandwidth - turn it off again '
                        'once you have looked.'),
        DeclareLaunchArgument(
            'debug_raw_rate', default_value='5.0',
            description='Hz cap on tag_image_raw. Publishing every frame '
                        'uncompressed drops detection from 30 Hz to 7; at '
                        '5 Hz detection measured a full 30 Hz with a viewer '
                        'attached.'),
        DeclareLaunchArgument(
            'debug_raw_scale', default_value='0.5',
            description='Shrink tag_image_raw by this factor. Cost is linear '
                        'in bytes and lives in rclpy, not the network: 126 ms '
                        'per publish at 1.0, 31 ms at 0.5, 8 ms at 0.25.'),
        DeclareLaunchArgument(
            'filter_min_cutoff', default_value='0.4',
            description="Hz. Cutoff of the pose filter while the marker is still - "
                        "lower is steadier and laggier. 0 publishes the raw pose, "
                        "which jitters about 1 mm and is what recordings before "
                        "2026-08-21 contain."),
        DeclareLaunchArgument(
            'filter_beta', default_value='8.0',
            description="Hz per m/s. How much the cutoff opens up with speed, which "
                        "is what stops a fast movement being smeared. 0 makes it a "
                        "plain fixed low-pass."),
        DeclareLaunchArgument(
            'corner_refinement', default_value='subpix',
            description="subpix | apriltag | none. apriltag refines better but "
                        "costs 24.5 ms a frame against subpix's 4.6, which does "
                        "not fit in a 33 ms frame period; the pose difference "
                        "is 0.7 mm."),

        Node(
            package='wsg50_haptic_teleoperation_interface',
            executable='usb_camera_node.py',
            name='usb_camera_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'device_id': LaunchConfiguration('device_id'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
                'auto_exposure': LaunchConfiguration('auto_exposure'),
                'exposure': LaunchConfiguration('exposure'),
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
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'tag_family': LaunchConfiguration('tag_family'),
                'tag_size': LaunchConfiguration('tag_size'),
                'use_compressed': True,
                'publish_debug_image': LaunchConfiguration('publish_debug_image'),
                'publish_debug_raw': LaunchConfiguration('publish_debug_raw'),
                'debug_raw_rate': LaunchConfiguration('debug_raw_rate'),
                'debug_raw_scale': LaunchConfiguration('debug_raw_scale'),
                'corner_refinement': LaunchConfiguration('corner_refinement'),
                'filter_min_cutoff': LaunchConfiguration('filter_min_cutoff'),
                'filter_beta': LaunchConfiguration('filter_beta'),
                'publish_tf': True,
                'camera_frame': LaunchConfiguration('camera_frame'),
            }],
        ),
    ])

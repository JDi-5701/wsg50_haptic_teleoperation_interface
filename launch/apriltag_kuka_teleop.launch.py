"""AprilTag -> KUKA iiwa 6D TCP teleoperation.

Starts the camera, the tag detector and the teleoperation node. The KUKA driver
is NOT started here - run it separately and make sure the arm is where you want
teleoperation to begin, because the first frame is what everything is measured
from:

    ros2 launch kuka_iiwa_ros2_driver driver.launch.py

The marker's displacement from its own starting pose becomes the TCP's
displacement from its own starting pose. No hand-eye calibration.

Which robot axis a given marker motion drives is set by the marker frame's
orientation at the zero instant, since the displacement is rotated into it. A
static transform tag_0 -> tag_aligned therefore remaps the axes, and that is
what tag_align_q* configures. The teleop node publishes that transform itself,
so ros2 run works standalone with no separate static_transform_publisher. The
default maps tag +z -> world +y, tag +y -> world +z, tag +x -> world -x. Check
it with small motions first.

Motion stops at the marker, not at the robot: a frame whose marker pose jumps
more than max_delta_position / max_delta_angle is skipped, and those defaults sit
inside the KUKA relay's own 0.15 m / 0.50 rad limit.

Usage:
    # everything at defaults
    ros2 launch wsg50_haptic_teleoperation_interface apriltag_kuka_teleop.launch.py

    # camera already running elsewhere
    ros2 launch ... apriltag_kuka_teleop.launch.py use_camera:=false

    # gentler translation, rotation left alone
    ros2 launch ... apriltag_kuka_teleop.launch.py position_scale:=0.1

    # translation and rotation ratios together
    ros2 launch ... apriltag_kuka_teleop.launch.py \
        position_scale:=0.15 orientation_scale:=0.3

    # every argument and its default
    ros2 launch ... apriltag_kuka_teleop.launch.py --show-args
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    haptic_launch = os.path.join(
        get_package_share_directory('wsg50_haptic_teleoperation_interface'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('use_camera', default_value='true',
                              description='Also start the camera and detector'),
        DeclareLaunchArgument(
            'tag_size', default_value='0.032',
            description='Marker size in metres, outer edge of the black square'),
        DeclareLaunchArgument(
            'tag_frame', default_value='tag_aligned',
            description='What the teleop node measures the marker in. Defaults '
                        'to the re-oriented frame below, not the raw tag_0.'),
        # The axis mapping is decided entirely by the marker frame's orientation
        # at the zero instant, because get_delta_pose rotates the displacement
        # into it: t_delta = R_initial^-1 * (t_now - t_initial). Inserting a
        # rotation R between tag_0 and the frame the node uses therefore remaps
        # the axes, since delta_aligned = R^-1 * delta_tag * R.
        #
        # Passed through to the node, which publishes the transform itself.
        #
        # The default is the mapping defined against the hardware:
        #
        #     tag +z  ->  world +y        (push the marker at the lens)
        #     tag +y  ->  world +z
        #     tag +x  ->  world -x
        #
        # As a matrix whose columns are the tag axes in world coordinates that
        # is [[-1,0,0],[0,0,1],[0,1,0]], which is orthogonal with determinant
        # +1 - a real rotation of 180 deg about (0,1,1)/sqrt2, not a mirror.
        # Worth checking if you change it: flip one sign and the determinant
        # goes to -1, which no rigid rotation can produce, and the arm would
        # move along an axis that cannot be reached by re-orienting anything.
        DeclareLaunchArgument(
            'tag_align_qx', default_value='0.0',
            description='Axis remap quaternion x. Default maps tag +z to world '
                        '+y, +y to +z, +x to -x.'),
        DeclareLaunchArgument('tag_align_qy', default_value='0.7071068',
                              description='Axis remap quaternion y'),
        DeclareLaunchArgument('tag_align_qz', default_value='0.7071068',
                              description='Axis remap quaternion z'),
        DeclareLaunchArgument('tag_align_qw', default_value='0.0',
                              description='Axis remap quaternion w'),
        DeclareLaunchArgument('camera_frame', default_value='camera_optical_frame',
                              description='Frame the tag pose is reported in'),
        DeclareLaunchArgument(
            'position_scale', default_value='0.8',
            description='Fraction of the marker motion the robot takes. 1.0 is '
                        'the UR behaviour.'),
        DeclareLaunchArgument(
            'orientation_scale', default_value='0.4',
            description='Fraction of the marker rotation the robot takes. '
                        'Scaled by angle about the same axis, not by '
                        'multiplying the quaternion.'),
        DeclareLaunchArgument(
            'max_delta_position', default_value='0.12',
            description='Skip a frame whose marker jumps further than this, in '
                        'metres. Stays inside the relay 0.15 m limit.'),
        DeclareLaunchArgument(
            'max_delta_angle', default_value='0.70',
            description='Same, in radians. Relay limit is 0.50.'),

        # Camera and detector settings. These are only re-declared so they can be
        # forwarded: every default lives in apriltag_camera.launch.py, and an
        # empty string here means "leave that launch file's default alone".
        # Before this existed the include passed only tag_size and camera_frame,
        # so exposure:=50 and friends were accepted on the command line and then
        # silently dropped.
        DeclareLaunchArgument('namespace', default_value='webcam',
                              description='Namespace for the camera and '
                                          'detector topics.'),
        DeclareLaunchArgument('device_id', default_value='0'),
        DeclareLaunchArgument('width', default_value='800'),
        DeclareLaunchArgument('height', default_value='600'),
        DeclareLaunchArgument('fps', default_value='30.0'),
        DeclareLaunchArgument('auto_exposure', default_value='false'),
        DeclareLaunchArgument('exposure', default_value='50',
                              description='Units of 100 us. See apriltag_camera.'),
        DeclareLaunchArgument('corner_refinement', default_value='subpix'),
        DeclareLaunchArgument('filter_min_cutoff', default_value='0.4'),
        DeclareLaunchArgument('filter_beta', default_value='8.0'),
        DeclareLaunchArgument('publish_debug_image', default_value='true'),
        DeclareLaunchArgument(
            'publish_debug_raw', default_value='false',
            description='Annotated frame uncompressed on tag_image_raw, for a '
                        'machine that cannot subscribe to the compressed one.'),
        DeclareLaunchArgument('debug_raw_rate', default_value='5.0',
                              description='Hz cap on tag_image_raw.'),
        DeclareLaunchArgument('debug_raw_scale', default_value='0.5',
                              description='Shrink tag_image_raw by this factor.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(haptic_launch, 'apriltag_camera.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_camera')),
            launch_arguments={
                'tag_size': LaunchConfiguration('tag_size'),
                'camera_frame': LaunchConfiguration('camera_frame'),
                'namespace': LaunchConfiguration('namespace'),
                'device_id': LaunchConfiguration('device_id'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
                'auto_exposure': LaunchConfiguration('auto_exposure'),
                'exposure': LaunchConfiguration('exposure'),
                'corner_refinement': LaunchConfiguration('corner_refinement'),
                'filter_min_cutoff': LaunchConfiguration('filter_min_cutoff'),
                'filter_beta': LaunchConfiguration('filter_beta'),
                'publish_debug_image': LaunchConfiguration('publish_debug_image'),
                'publish_debug_raw': LaunchConfiguration('publish_debug_raw'),
                'debug_raw_rate': LaunchConfiguration('debug_raw_rate'),
                'debug_raw_scale': LaunchConfiguration('debug_raw_scale'),
            }.items(),
        ),

        Node(
            package='wsg50_haptic_teleoperation_interface',
            executable='apriltag_kuka_teleoperation_node.py',
            name='apriltag_kuka_teleoperation_node',
            output='screen',
            parameters=[{
                'camera_frame': LaunchConfiguration('camera_frame'),
                'tag_frame': LaunchConfiguration('tag_frame'),
                'tag_align_qx': LaunchConfiguration('tag_align_qx'),
                'tag_align_qy': LaunchConfiguration('tag_align_qy'),
                'tag_align_qz': LaunchConfiguration('tag_align_qz'),
                'tag_align_qw': LaunchConfiguration('tag_align_qw'),
                'command_frame': 'world_frame',
                'position_scale': LaunchConfiguration('position_scale'),
                'orientation_scale': LaunchConfiguration('orientation_scale'),
                'max_delta_position': LaunchConfiguration('max_delta_position'),
                'max_delta_angle': LaunchConfiguration('max_delta_angle'),
            }],
        ),
    ])

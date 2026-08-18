"""Teleoperation controller: knob_state -> WSG50 gripper width.

The force feedback itself is closed on the ESP32 boards (FSR board -> knob
board -> its motor), so this node is one-way: it integrates the knob's encoder
delta into a target width and publishes /wsg50/command/move. The force topic is
subscribed for logging only.

The node subscribes to `gripper_finger_force_old`, a name nothing publishes;
it is remapped here onto the real topic.

Standalone:
    ros2 launch wsg50_haptic_teleoperation_interface teleoperation_interface.launch.py
    ros2 launch ... teleoperation_interface.launch.py position_factor:=0.02
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'position_factor', default_value='0.013',
            description='mm of gripper width per knob encoder count. Negative '
                        'flips the direction.'),
        DeclareLaunchArgument('gripper_min_width', default_value='10.0'),
        DeclareLaunchArgument('gripper_max_width', default_value='105.0'),
        DeclareLaunchArgument('force_topic', default_value='/gripper_finger_force'),

        Node(
            package='wsg50_haptic_teleoperation_interface',
            executable='knob_force_feedback_controller_wsg50.py',
            name='wsg50_controller',
            output='screen',
            parameters=[{
                'position_factor': LaunchConfiguration('position_factor'),
                'position_jump_threshold': 1000,
                'gripper_min_width': LaunchConfiguration('gripper_min_width'),
                'gripper_max_width': LaunchConfiguration('gripper_max_width'),
                'gripper_velocity': 0.1,
                'control_rate': 100.0,
            }],
            remappings=[
                ('/gripper_finger_force_old', LaunchConfiguration('force_topic')),
            ],
        ),
    ])

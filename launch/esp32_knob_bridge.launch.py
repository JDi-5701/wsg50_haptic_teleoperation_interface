"""Knob ESP32 -> ROS bridge.

The knob board relays the FSR board's reading and adds its own encoder/motor
state, so this one 36-byte packet carries everything the PC needs:

    /gripper_finger_force  (Float32)  force in N, relayed from the FSR board
    /knob_state            (Int32)    encoder count, absolute
    /knob_torque           (Float32)  force-feedback motor torque

Standalone:
    ros2 launch wsg50_haptic_teleoperation_interface esp32_knob_bridge.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('knob_ip', default_value='10.200.2.148',
                              description='IP of the knob ESP32'),
        DeclareLaunchArgument('knob_local_port', default_value='5001',
                              description='UDP port on this PC to listen on'),
        DeclareLaunchArgument(
            'calibration_samples', default_value='100',
            description='Packets averaged at startup into the force zero '
                        'offset. Keep the finger unloaded until "Zero offset" '
                        'is logged. 0 disables the tare.'),

        Node(
            package='wsg50_haptic_teleoperation_interface',
            executable='udp_ros_bridge.py',
            name='esp32_knob_node',
            output='screen',
            parameters=[{
                'local_port': LaunchConfiguration('knob_local_port'),
                'esp32_address': LaunchConfiguration('knob_ip'),
                'esp32_port': 5000,
                'publish_rate': 100.0,
                'calibration_samples': LaunchConfiguration('calibration_samples'),
            }],
        ),
    ])

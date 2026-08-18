"""FSR ESP32 -> ROS bridge (optional, for cross-checking).

The FSR board streams its own 24-byte packet straight to this PC in parallel
with feeding the knob board. That value is redundant with the one relayed
inside the knob packet, so it is published under `_direct` names and the two
can be compared to measure what the relay costs in latency.

Unlike the knob bridge, this node tares itself: keep the sensor unloaded until
it logs "Zero offset = ...".

Standalone:
    ros2 launch wsg50_haptic_teleoperation_interface esp32_force_sensor.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('force_sensor_ip', default_value='10.200.2.149',
                              description='IP of the FSR ESP32'),
        DeclareLaunchArgument('force_sensor_local_port', default_value='5000',
                              description='UDP port on this PC to listen on'),

        Node(
            package='wsg50_haptic_teleoperation_interface',
            executable='udp_ros_bridge_force_sensor.py',
            name='esp32_force_sensor_node',
            output='screen',
            parameters=[{
                'local_port': LaunchConfiguration('force_sensor_local_port'),
                'esp32_address': LaunchConfiguration('force_sensor_ip'),
                'esp32_port': 5000,
                'filter_window_size': 5,
                'publish_rate': 100.0,
                'calibration_samples': 100,
            }],
            remappings=[
                ('gripper_finger_force', 'gripper_finger_force_direct'),
                ('gripper_finger_force_raw', 'gripper_finger_force_direct_raw'),
            ],
        ),
    ])

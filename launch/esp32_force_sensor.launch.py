from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wsg_teleoperation_interface',
            executable='udp_ros_bridge_force_sensor.py',
            name='esp32_force_sensor_node',
            output='screen',
            parameters=[{
                'local_port': 5000,
                'esp32_address': '10.200.2.149',
                'esp32_port': 5000,
                'filter_window_size': 5
            }]
        )
    ]) 
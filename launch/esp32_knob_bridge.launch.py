from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wsg_teleoperation_interface',
            executable='udp_ros_bridge.py',
            name='esp32_knob_node',
            output='screen',
            parameters=[{
                'local_port': 5001,
                'esp32_address': '10.200.2.148',
                'esp32_port': 5000
            }]
        )
    ]) 
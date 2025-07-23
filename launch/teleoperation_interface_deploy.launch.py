from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wsg_teleoperation_interface',
            executable='wsg_command_relay_node.py',
            name='wsg_command_relay_node',
            output='screen',
            parameters=[
                {
                }
            ]
        )
    ]) 
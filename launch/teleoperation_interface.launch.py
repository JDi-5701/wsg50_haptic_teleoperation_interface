from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wsg_teleoperation_interface',
            executable='knob_force_feedback_controller_wsg50.py',
            name='knob_force_feedback_controller_wsg50',
            output='screen',
            parameters=[
                {
                }
            ]
        )
    ]) 
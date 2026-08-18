"""OmniHGI: Paxini fingertip -> coils + knob -> WSG50 gripper.

Full chain, PC side:

    paxini_2015_finger_node        /tactile_resultant_wrench  (30 Hz)
              |
    omni_hgi_bridge                Fx,Fy -> coils / Fz -> knob motor, 12 B UDP
              |  <-- 36 B UDP        /knob_state, /knob_torque,
              |                      /gripper_finger_force
    wsg50_controller               /knob_state -> /wsg50/command/move
              |
    wsg50_ros_driver               the gripper

Unlike the FSR/knob setup this replaces, the loop is closed through the PC:
the fingertip force reaches the coils only via omni_hgi_bridge. If this node
dies the bridge zeroes the coils after `force_timeout`.

Usage:
    # sensing + feedback only, gripper stays put
    ros2 launch wsg50_haptic_teleoperation_interface omni_hgi_teleop.launch.py \
        use_gripper:=false

    # everything (the gripper HOMES on startup - keep fingers clear)
    ros2 launch wsg50_haptic_teleoperation_interface omni_hgi_teleop.launch.py

    # tuning the shear-to-coil gain
    ros2 launch ... omni_hgi_teleop.launch.py coil_ratio:=0.12
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
    driver_launch = os.path.join(
        get_package_share_directory('wsg50_ros_driver'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0',
                              description='Paxini fingertip serial device'),
        DeclareLaunchArgument('esp32_ip', default_value='10.200.2.148',
                              description='IP of the knob+coil ESP32'),
        DeclareLaunchArgument('gripper_ip', default_value='10.200.2.152'),
        DeclareLaunchArgument(
            'sensor_rate', default_value='30.0',
            description='Paxini poll rate. Each cycle is two UART round trips; '
                        'the 52-point array is the slow one.'),
        DeclareLaunchArgument(
            'coil_ratio', default_value='0.08',
            description='Coil duty per newton of shear force (Fx/Fy).'),
        DeclareLaunchArgument(
            'coil_max_duty', default_value='0.8',
            description='Hard clamp on coil duty cycle.'),
        DeclareLaunchArgument(
            'knob_force_ratio', default_value='1.0',
            description='Knob feedback force per newton of normal force (Fz).'),
        DeclareLaunchArgument('position_factor', default_value='0.013',
                              description='mm of gripper width per knob count'),
        DeclareLaunchArgument(
            'use_gripper', default_value='true',
            description='Start the WSG50 driver. false = test the haptic loop '
                        'without moving the gripper.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Paxini tactile-field visualisation. Needs ros-foxy-rviz2, '
                        'which ros-foxy-ros-base does not install.'),

        # --- Fingertip ---
        Node(
            package='paxini_2015_finger_driver',
            executable='paxini_2015_finger_node',
            name='paxini_2015_finger_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': 921600,
                'frame_id': 'sensor_base',
                'publish_rate': LaunchConfiguration('sensor_rate'),
            }],
        ),

        # --- Bridge: force out to the coils, knob state back in ---
        Node(
            package='wsg50_haptic_teleoperation_interface',
            executable='omni_hgi_bridge.py',
            name='omni_hgi_bridge',
            output='screen',
            parameters=[{
                'local_port': 5001,
                'esp32_address': LaunchConfiguration('esp32_ip'),
                'esp32_port': 5000,
                'tx_rate': 100.0,
                'publish_rate': 100.0,
                'coil_ratio': LaunchConfiguration('coil_ratio'),
                'coil_max_duty': LaunchConfiguration('coil_max_duty'),
                'knob_force_ratio': LaunchConfiguration('knob_force_ratio'),
                'calibration_samples': 100,
                'force_timeout': 0.5,
            }],
        ),

        # --- Knob -> gripper width ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(haptic_launch, 'teleoperation_interface.launch.py')),
            launch_arguments={
                'position_factor': LaunchConfiguration('position_factor'),
            }.items(),
        ),

        # --- The gripper itself ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(driver_launch, 'wsg50_node.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_gripper')),
            launch_arguments={
                'gripper_ip': LaunchConfiguration('gripper_ip'),
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=['-d', os.path.join(
                get_package_share_directory('paxini_2015_finger_driver'),
                'rviz', 'paxini_2015_single_sensor.rviz')],
        ),
    ])

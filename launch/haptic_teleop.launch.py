"""Top-level entry point for WSG50 haptic teleoperation.

Composes the single-purpose launch files; it starts no node of its own:

    esp32_knob_bridge.launch.py    knob ESP32 -> /knob_state, /gripper_finger_force
    teleoperation_interface.launch.py  /knob_state -> /wsg50/command/move
    wsg50_node.launch.py           (wsg50_ros_driver) -> the gripper
    esp32_force_sensor.launch.py   optional, cross-check only

Topology - the haptic loop is closed between the two ESP32 boards, the PC is
not in it:

    FSR board (.149) --> knob board (.148) -- drives its own feedback motor
                              |
                              +-- UDP 36B --> PC : force + knob state, one packet

Usage:
    # sensing chain only, gripper stays put
    ros2 launch wsg50_haptic_teleoperation_interface haptic_teleop.launch.py use_gripper:=false

    # everything (the gripper HOMES on startup - keep fingers clear)
    ros2 launch wsg50_haptic_teleoperation_interface haptic_teleop.launch.py

    # calibrating the knob-to-width gain
    ros2 launch ... haptic_teleop.launch.py position_factor:=0.02
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    haptic_launch = os.path.join(
        get_package_share_directory('wsg50_haptic_teleoperation_interface'), 'launch')
    driver_launch = os.path.join(
        get_package_share_directory('wsg50_ros_driver'), 'launch')

    def include(path, condition=None, **launch_arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path),
            condition=condition,
            launch_arguments=launch_arguments.items(),
        )

    return LaunchDescription([
        DeclareLaunchArgument('knob_ip', default_value='10.200.2.148'),
        DeclareLaunchArgument('force_sensor_ip', default_value='10.200.2.149'),
        DeclareLaunchArgument('gripper_ip', default_value='10.200.2.152'),
        DeclareLaunchArgument('position_factor', default_value='0.013',
                              description='mm of gripper width per knob count'),
        DeclareLaunchArgument(
            'calibration_samples', default_value='100',
            description='Packets averaged at startup into the force zero '
                        'offset. Keep the finger unloaded until "Zero offset" '
                        'is logged. 0 disables the tare.'),
        DeclareLaunchArgument(
            'use_gripper', default_value='true',
            description='Start the WSG50 driver. false = test the knob/force '
                        'chain without moving the gripper.'),
        DeclareLaunchArgument(
            'use_direct_force_sensor', default_value='false',
            description='Also bridge the FSR board\'s own UDP stream, on '
                        '/gripper_finger_force_direct, to cross-check the '
                        'value relayed through the knob board.'),

        include(os.path.join(haptic_launch, 'esp32_knob_bridge.launch.py'),
                knob_ip=LaunchConfiguration('knob_ip'),
                calibration_samples=LaunchConfiguration('calibration_samples')),

        include(os.path.join(haptic_launch, 'teleoperation_interface.launch.py'),
                position_factor=LaunchConfiguration('position_factor')),

        include(os.path.join(driver_launch, 'wsg50_node.launch.py'),
                condition=IfCondition(LaunchConfiguration('use_gripper')),
                gripper_ip=LaunchConfiguration('gripper_ip')),

        include(os.path.join(haptic_launch, 'esp32_force_sensor.launch.py'),
                condition=IfCondition(LaunchConfiguration('use_direct_force_sensor')),
                force_sensor_ip=LaunchConfiguration('force_sensor_ip')),
    ])

"""OmniHGI: Paxini fingertip -> coils + knob -> WSG50 gripper.

Full chain, PC side:

    paxini_2015_finger_node        /tactile_resultant_wrench  (30 Hz)
              |
    omni_hgi_haptic_teleop         Fx,Fy -> coils / Fz -> knob  =>  /coil_command
              |                      /knob_state -> /wsg50/command/move
    omni_hgi_udp_driver            /coil_command -> ForceMsg3D 12 B
              |  <-- MotorMsg 12 B    -> /knob_state, /knob_torque
              |
    wsg50_ros_driver               the gripper

The Paxini fingertip is the only force source; the loop is closed through the
PC, so fingertip force reaches the coils only via these two nodes. If either
stops, omni_hgi_udp_driver zeroes the coils after `command_timeout`.

Usage:
    # sensing + feedback only, gripper stays put
    ros2 launch wsg50_haptic_teleoperation_interface omni_hgi.launch.py \
        use_gripper:=false

    # everything (the gripper HOMES on startup - keep fingers clear)
    ros2 launch wsg50_haptic_teleoperation_interface omni_hgi.launch.py

    # tuning the shear-to-coil gain, one axis at a time
    ros2 launch ... omni_hgi.launch.py coil_ratio_x:=0.12 coil_ratio_y:=0.20
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
            'gripper_kp', default_value='3.0',
            description='Stiffness of the PD running on the gripper, in 1/s. '
                        'Tracking lag is speed/kp, so this is the knob that '
                        'decides how tightly the fingers follow. See '
                        'wsg50_node.launch.py for the full note.'),
        DeclareLaunchArgument(
            'gripper_kd', default_value='0.05',
            description='Damping of that PD. It damps towards the commanded '
                        'velocity, not zero, so it fights fast tracking.'),
        DeclareLaunchArgument(
            'sensor_rate', default_value='30.0',
            description='Paxini poll rate. Each cycle is two UART round trips; '
                        'the 52-point array is the slow one.'),
        DeclareLaunchArgument(
            'coil_ratio_x', default_value='0.08',
            description='Coil X duty per newton of shear. Named for the coil it '
                        'drives, which is the fingertip Fy - the sensor frame '
                        'and the coil frame are 90 degrees apart. Tune the two '
                        'axes separately; they do not have the same geometry '
                        'and a shared gain feels lopsided.'),
        DeclareLaunchArgument(
            'coil_ratio_y', default_value='0.08',
            description='Coil Y duty per newton of shear, driven by fingertip Fx.'),
        DeclareLaunchArgument(
            'coil_deadzone', default_value='0.0',
            description='Newtons of shear below which the coils stay off. Raise '
                        'it if the coils buzz when the finger is at rest.'),
        DeclareLaunchArgument(
            'coil_max_duty', default_value='0.8',
            description='Hard clamp on coil duty cycle.'),
        DeclareLaunchArgument(
            'knob_force_ratio', default_value='1.0',
            description='Knob feedback force per newton of normal force (Fz). '
                        'The Z axis goes to the knob motor, not the coils, so '
                        'this is the counterpart of coil_ratio_x/y rather than '
                        'a third coil gain.'),
        DeclareLaunchArgument(
            'knob_force_max', default_value='5.0',
            description='Clamp on that knob force, both directions. The Z '
                        'counterpart of coil_max_duty: at ratio 1.0 the knob '
                        'saturates at 5 N of fingertip force, so raise this '
                        'as well if a higher ratio should stay useful over the '
                        'whole range rather than pinning early.'),
        DeclareLaunchArgument(
            'position_factor', default_value='0.04',
            description='mm of gripper width per knob count. Was 0.013 while the '
                        'control loop re-applied each knob delta until the next '
                        'packet arrived, which multiplied it by control_rate / '
                        'knob_rate and made the real ratio drift between 0.018 '
                        'and 0.026 with the WiFi. With that fixed the number '
                        'means what it says, and 0.022 reproduces the old feel.'),
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

        # --- Transport: UDP <-> topics, no policy ---
        Node(
            package='wsg50_haptic_teleoperation_interface',
            executable='omni_hgi_udp_driver.py',
            name='omni_hgi_udp_driver',
            output='screen',
            parameters=[{
                'local_port': 5001,
                'esp32_address': LaunchConfiguration('esp32_ip'),
                'esp32_port': 5000,
                'tx_rate': 100.0,
                'publish_rate': 0.0,
                'command_timeout': 0.5,
            }],
        ),

        # --- Policy: knob -> gripper, fingertip -> coils ---
        Node(
            package='wsg50_haptic_teleoperation_interface',
            executable='omni_hgi_haptic_teleop.py',
            name='omni_hgi_haptic_teleop',
            output='screen',
            parameters=[{
                'position_factor': LaunchConfiguration('position_factor'),
                'coil_ratio_x': LaunchConfiguration('coil_ratio_x'),
                'coil_ratio_y': LaunchConfiguration('coil_ratio_y'),
                'coil_deadzone': LaunchConfiguration('coil_deadzone'),
                'coil_max_duty': LaunchConfiguration('coil_max_duty'),
                'knob_force_ratio': LaunchConfiguration('knob_force_ratio'),
                'knob_force_max': LaunchConfiguration('knob_force_max'),
                'control_rate': 100.0,
            }],
        ),

        # --- The gripper itself ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(driver_launch, 'wsg50_node.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_gripper')),
            launch_arguments={
                'gripper_ip': LaunchConfiguration('gripper_ip'),
                'kp': LaunchConfiguration('gripper_kp'),
                'kd': LaunchConfiguration('gripper_kd'),
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

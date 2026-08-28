"""OmniHGI with a compliant gripper: Paxini fingertip -> coils + knob -> WSG50.

Same chain as omni_hgi.launch.py, with one difference: the WSG50 runs its
admittance controller. The knob no longer commands the finger width directly -
it commands the *rest width* of a virtual mass-spring-damper, and the fingertip
force pushes the fingers away from it:

    m*e'' + d*e' + k*e = f_ext,     width = knob_width + e

So the gripper yields when the fingertip is squeezed, instead of holding the
knob's width and crushing.

Note that the fingertip force now does double duty - it drives the knob
feedback the operator feels AND the compliance that makes the gripper escape
from it. Both are negative feedback and the loop is stable, but the operator
will feel the object 'give' as the fingers back off; that is the point, and it
is also why knob_force_ratio may want retuning against what worked in the stiff
version.

Everything is exposed as a launch argument, including the values that
omni_hgi.launch.py hardcodes, so the whole chain can be tuned from one command
line.

Usage:
    # everything (the gripper HOMES on startup - keep fingers clear)
    ros2 launch wsg50_haptic_teleoperation_interface omni_hgi_admittance.launch.py

    # haptics only, gripper stays put
    ros2 launch ... omni_hgi_admittance.launch.py use_gripper:=false

    # softer / stiffer fingers
    ros2 launch ... omni_hgi_admittance.launch.py virtual_stiffness:=50.0
    ros2 launch ... omni_hgi_admittance.launch.py virtual_stiffness:=200.0

    # back to the stiff behaviour of omni_hgi.launch.py, same command line
    ros2 launch ... omni_hgi_admittance.launch.py admittance:=false

The haptic gains default to the values in use on the bench: coil_ratio_x 0.1,
coil_ratio_y -0.1, knob_force_ratio 0.2, knob_force_max 15.0. Everything the
gripper side owns - gripper_kp, gripper_kd and the admittance gains - matches
wsg50_admittance.launch.py exactly, so the two files cannot drift apart. The admittance gains match the
driver's own defaults: k 200 N/m, d 30 N*s/m, m 0.05 kg, giving a damping ratio
of 4.7 and 5 mm of yield per newton.
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
    driver_launch = os.path.join(
        get_package_share_directory('wsg50_ros_driver'), 'launch')

    return LaunchDescription([
        # ================= Hardware addresses =================
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0',
                              description='Paxini fingertip serial device'),
        DeclareLaunchArgument('baud_rate', default_value='921600'),
        DeclareLaunchArgument('frame_id', default_value='sensor_base'),
        DeclareLaunchArgument('esp32_ip', default_value='10.200.2.148',
                              description='IP of the knob+coil ESP32'),
        DeclareLaunchArgument('gripper_ip', default_value='10.200.2.152'),
        DeclareLaunchArgument('gripper_port', default_value='1000'),
        DeclareLaunchArgument(
            'sensor_rate', default_value='30.0',
            description='Paxini poll rate. Each cycle is two UART round trips; '
                        'the 52-point array is the slow one, and it is polled '
                        'before the resultant the control loop actually needs.'),

        # ================= UDP transport =================
        DeclareLaunchArgument('local_port', default_value='5001'),
        DeclareLaunchArgument('esp32_port', default_value='5000'),
        DeclareLaunchArgument('tx_rate', default_value='100.0',
                              description='Coil command packets per second.'),
        DeclareLaunchArgument('udp_publish_rate', default_value='0.0',
                              description='0 = publish knob state as packets '
                                          'arrive rather than on a timer.'),
        DeclareLaunchArgument('command_timeout', default_value='0.5',
                              description='Coils are zeroed if no command '
                                          'arrives within this many seconds.'),
        DeclareLaunchArgument('report_period', default_value='2.0'),

        # ================= Teleop policy =================
        DeclareLaunchArgument(
            'position_factor', default_value='0.04',
            description='mm of gripper width per knob count. Under admittance '
                        'this scales the rest width, not the finger width.'),
        DeclareLaunchArgument('position_jump_threshold', default_value='1000'),
        DeclareLaunchArgument(
            'teleop_min_width', default_value='10.0',
            description="Clamp on the knob-driven width, inside the teleop node. "
                        "Separate from the driver's own min_width/max_width, "
                        "which bound the width after the admittance offset."),
        DeclareLaunchArgument('teleop_max_width', default_value='105.0'),
        DeclareLaunchArgument(
            'gripper_velocity', default_value='0.1',
            description='Velocity field of the GripperCommand. Ignored while '
                        'admittance runs with velocity_feedforward on - the '
                        'driver substitutes the virtual velocity.'),
        DeclareLaunchArgument('gripper_start_width', default_value='102.0'),
        DeclareLaunchArgument('control_rate', default_value='100.0',
                              description='Teleop node loop rate.'),
        DeclareLaunchArgument('command_deadband', default_value='0.0'),
        DeclareLaunchArgument(
            'coil_ratio_x', default_value='0.1',
            description='Coil X duty per newton of shear, driven by fingertip '
                        'Fy - the sensor and coil frames are 90 degrees apart.'),
        DeclareLaunchArgument(
            'coil_ratio_y', default_value='-0.1',
            description='Coil Y duty per newton of shear, driven by fingertip Fx.'),
        DeclareLaunchArgument('coil_deadzone', default_value='0.0'),
        DeclareLaunchArgument('coil_max_duty', default_value='0.8'),
        DeclareLaunchArgument(
            'knob_force_ratio', default_value='0.2',
            description='Knob feedback force per newton of fingertip Fz. The '
                        'compliant gripper backs away from that same Fz, so the '
                        'force the operator feels decays as the fingers yield; '
                        'expect to want a different value than in the stiff '
                        'version.'),
        DeclareLaunchArgument('knob_force_max', default_value='15.0'),

        # ================= Gripper servo loop =================
        DeclareLaunchArgument(
            'gripper_kp', default_value='15.0',
            description='Stiffness of the PD running on the gripper, in 1/s. '
                        'Tracking lag is speed/kp, so under admittance a 40 mm/s '
                        'yield trails by 40/kp mm. The script runs at ~30 Hz, so '
                        'kp = 30 is deadbeat and kp >= 60 oscillates.'),
        DeclareLaunchArgument(
            'gripper_kd', default_value='0.05',
            description='Damping of that PD. It damps towards the commanded '
                        'velocity, not zero.'),
        DeclareLaunchArgument('pub_rate', default_value='30.0',
                              description='Driver state/command rate. The '
                                          'gripper script tops out near 30 Hz.'),

        # ================= Admittance =================
        DeclareLaunchArgument(
            'admittance', default_value='true',
            description='false reproduces omni_hgi.launch.py exactly - the knob '
                        'width goes straight to the fingers.'),
        DeclareLaunchArgument(
            'virtual_stiffness', default_value='200.0',
            description='N/m. Steady-state yield is f/k, so 200 opens 5 mm per '
                        'newton. Raise it first if the fingers jitter: '
                        'softening is nearly free in noise terms, speeding up '
                        'is not.'),
        DeclareLaunchArgument(
            'virtual_damping', default_value='30.0',
            description='N*s/m. Lowering this is what makes the fingers quicker '
                        'off the mark - and noisier, since k/d is also the '
                        'corner sensor noise reaches them through.'),
        DeclareLaunchArgument(
            'virtual_mass', default_value='0.05',
            description='kg. Sets the initial reaction to a force step '
                        '(accel = f/m); the stiffness has no effect there.'),
        DeclareLaunchArgument(
            'force_source', default_value='tactile',
            description="'tactile' = the Paxini fingertip, 'motor' = the "
                        "gripper's own coarser estimate, 'none' = no force."),
        DeclareLaunchArgument(
            'force_topic', default_value='/tactile_resultant_wrench',
            description='Same topic the coil/knob feedback reads.'),
        DeclareLaunchArgument(
            'force_axis', default_value='z',
            description='Fz is the normal channel, and the one the knob already '
                        'uses. x and y are the shear that drives the coils.'),
        DeclareLaunchArgument(
            'force_scale', default_value='1.0',
            description='Raw reading to newtons. The Paxini driver only scales '
                        'its bytes by 0.1 and documents no unit, so check this '
                        'against a known weight - the stiffness assumes newtons.'),
        DeclareLaunchArgument('force_deadband', default_value='0.3',
                              description='N; subtracted, so the command stays '
                                          'continuous across the threshold.'),
        DeclareLaunchArgument('force_lowpass_hz', default_value='5.0'),
        DeclareLaunchArgument(
            'max_offset', default_value='30.0',
            description='mm; largest deviation the admittance may add. It '
                        'saturates at f = k*max_offset, i.e. 6 N at k=200.'),
        DeclareLaunchArgument('max_admittance_speed', default_value='100.0',
                              description='mm/s cap on the yielding motion.'),
        DeclareLaunchArgument(
            'driver_min_width', default_value='0.0',
            description='Stroke clamp inside the driver, applied after the '
                        'admittance offset.'),
        DeclareLaunchArgument('driver_max_width', default_value='110.0'),

        # ================= Switches =================
        DeclareLaunchArgument(
            'use_gripper', default_value='true',
            description='Start the WSG50 driver. false = test the haptic loop '
                        'without moving the gripper.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Paxini tactile-field visualisation. Needs '
                        'ros-foxy-rviz2, which ros-foxy-ros-base omits.'),

        # --- Fingertip ---
        Node(
            package='paxini_2015_finger_driver',
            executable='paxini_2015_finger_node',
            name='paxini_2015_finger_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'frame_id': LaunchConfiguration('frame_id'),
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
                'local_port': LaunchConfiguration('local_port'),
                'esp32_address': LaunchConfiguration('esp32_ip'),
                'esp32_port': LaunchConfiguration('esp32_port'),
                'tx_rate': LaunchConfiguration('tx_rate'),
                'publish_rate': LaunchConfiguration('udp_publish_rate'),
                'command_timeout': LaunchConfiguration('command_timeout'),
                'report_period': LaunchConfiguration('report_period'),
            }],
        ),

        # --- Policy: knob -> gripper rest width, fingertip -> coils ---
        Node(
            package='wsg50_haptic_teleoperation_interface',
            executable='omni_hgi_haptic_teleop.py',
            name='omni_hgi_haptic_teleop',
            output='screen',
            parameters=[{
                'position_factor': LaunchConfiguration('position_factor'),
                'position_jump_threshold': LaunchConfiguration('position_jump_threshold'),
                'gripper_min_width': LaunchConfiguration('teleop_min_width'),
                'gripper_max_width': LaunchConfiguration('teleop_max_width'),
                'gripper_velocity': LaunchConfiguration('gripper_velocity'),
                'gripper_start_width': LaunchConfiguration('gripper_start_width'),
                'control_rate': LaunchConfiguration('control_rate'),
                'command_deadband': LaunchConfiguration('command_deadband'),
                'coil_ratio_x': LaunchConfiguration('coil_ratio_x'),
                'coil_ratio_y': LaunchConfiguration('coil_ratio_y'),
                'coil_deadzone': LaunchConfiguration('coil_deadzone'),
                'coil_max_duty': LaunchConfiguration('coil_max_duty'),
                'knob_force_ratio': LaunchConfiguration('knob_force_ratio'),
                'knob_force_max': LaunchConfiguration('knob_force_max'),
            }],
        ),

        # --- The gripper, under admittance ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(driver_launch, 'wsg50_node.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_gripper')),
            launch_arguments={
                'gripper_ip': LaunchConfiguration('gripper_ip'),
                'gripper_port': LaunchConfiguration('gripper_port'),
                'pub_rate': LaunchConfiguration('pub_rate'),
                'kp': LaunchConfiguration('gripper_kp'),
                'kd': LaunchConfiguration('gripper_kd'),
                'admittance': LaunchConfiguration('admittance'),
                'force_source': LaunchConfiguration('force_source'),
                'force_topic': LaunchConfiguration('force_topic'),
                'force_axis': LaunchConfiguration('force_axis'),
                'force_scale': LaunchConfiguration('force_scale'),
                'force_deadband': LaunchConfiguration('force_deadband'),
                'force_lowpass_hz': LaunchConfiguration('force_lowpass_hz'),
                'virtual_mass': LaunchConfiguration('virtual_mass'),
                'virtual_damping': LaunchConfiguration('virtual_damping'),
                'virtual_stiffness': LaunchConfiguration('virtual_stiffness'),
                'max_offset': LaunchConfiguration('max_offset'),
                'max_admittance_speed': LaunchConfiguration('max_admittance_speed'),
                'min_width': LaunchConfiguration('driver_min_width'),
                'max_width': LaunchConfiguration('driver_max_width'),
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

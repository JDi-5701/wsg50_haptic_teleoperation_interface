from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    return LaunchDescription([

        # === 启动 RealSense 摄像头节点 ===
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         os.path.join(
        #             get_package_share_directory('realsense2_camera'),
        #             'launch',
        #             'rs_launch.py'
        #         )
        #     ]),
        #     # launch_arguments={
        #     #     'enable_color': 'true',
        #     #     'rgb_camera.color_profile': '640,480,30',
        #     #     'rgb_camera.color_format': 'RGB8',
        #     #     'enable_depth': 'false',
        #     #     # 'depth_module.depth_profile': '640,480,15',
        #     #     # 'depth_module.depth_format': 'Z16',
        #     #     # 'align_depth.enable': 'true',
        #     #     'enable_depth': 'false',
        #     #     'enable_infra1': 'false',
        #     #     'enable_infra2': 'false',
        #     #     'emitter_enabled': 'false',
        #     # }.items(),
        #     launch_arguments={
        #         'enable_color': 'true',
        #         'rgb_camera.color_profile': '640,480,30',
        #         'rgb_camera.color_format': 'RGB8',
        #         'enable_depth': 'false',
        #         'enable_infra1': 'false',
        #         'enable_infra2': 'false',
        #         'emitter_enabled': 'false',
        #     }.items(),
        # ),

        # === 启动你的 apriltag 节点 ===
        Node(
            package='wsg50_haptic_teleoperation_interface',
            executable='apriltag_ur_teleoperation_node.py',
            name='apriltag_ur_teleoperation_node',
            output='screen'
        ),

        # === 添加 world → camera_link 的静态变换 ===
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_world_to_base',
            arguments=[
                '0.0', '0.45', '0.0',     # translation: x y z
                '0.0', '0.0', '0.0', '1.0',  # rotation (quaternion): x y z w
                'world',
                'workspace_middle'
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_world_to_camera',
            arguments=[
                '0.582', '-0.265', '0.489',     # translation: x y z
                '-0.328', '0.154', '0.868', '0.340',  # rotation (quaternion): x y z w
                'workspace_middle',
                'camera_link'
            ]
        )
    ])

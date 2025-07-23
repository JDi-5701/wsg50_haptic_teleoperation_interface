from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('wsg_teleoperation_interface')
    wsg50_package_dir = get_package_share_directory('wsg50_ros_driver')
    
    # Include the force sensor bridge launch file
    # force_sensor_bridge = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(package_dir, 'launch', 'esp32_force_sensor.launch.py')
    #     )
    # )
    
    # Include the knob bridge launch file
    knob_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'esp32_knob_bridge.launch.py')
        )
    )
    
    # Include the teleoperation interface launch file
    teleoperation_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'teleoperation_interface_deploy.launch.py')
        )
    )

    # Include the WSG50 node launch file
    wsg50_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wsg50_package_dir, 'launch', 'wsg50_node.launch.py')
        )
    )

    return LaunchDescription([
        #force_sensor_bridge,
        knob_bridge,
        teleoperation_interface,
        wsg50_node,
    ])

from setuptools import setup
import os
from glob import glob

package_name = 'wsg_teleoperation_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ding',
    maintainer_email='ding@todo.todo',
    description='ROS2 teleoperation interface for WSG50 gripper',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'knob_force_feedback_controller_wsg50 = wsg_teleoperation_interface.knob_force_feedback_controller_wsg50:main',
            'udp_ros_bridge_force_sensor = wsg_teleoperation_interface.udp_ros_bridge_force_sensor:main',
            'udp_ros_bridge = wsg_teleoperation_interface.udp_ros_bridge:main',
            'udp_ros_bridge_imu = wsg_teleoperation_interface.udp_ros_bridge_imu:main',
            'apriltag_ur_teleoperation_node = wsg_teleoperation_interface.apriltag_ur_teleoperation_node:main',
        ],
    },
)

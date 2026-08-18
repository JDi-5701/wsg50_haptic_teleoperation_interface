cmake_minimum_required(VERSION 3.8)
project(wsg50_haptic_teleoperation_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  wsg50_haptic_teleoperation_interface/udp_ros_bridge_force_sensor.py
  wsg50_haptic_teleoperation_interface/knob_force_feedback_controller_wsg50.py
  wsg50_haptic_teleoperation_interface/udp_ros_bridge.py
  wsg50_haptic_teleoperation_interface/udp_ros_bridge_imu.py
  wsg50_haptic_teleoperation_interface/apriltag_ur_teleoperation_node.py
  wsg50_haptic_teleoperation_interface/wsg_command_relay_node.py
  wsg50_haptic_teleoperation_interface/omni_hgi_udp_driver.py
  wsg50_haptic_teleoperation_interface/omni_hgi_haptic_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


ament_export_dependencies(rosidl_default_runtime)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

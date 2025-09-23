cmake_minimum_required(VERSION 3.8)
project(rover_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ROS 2 dependencies
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ROS 2 Control dependencies
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(joint_state_broadcaster REQUIRED)

# Action and service dependencies
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# Navigation dependencies
find_package(nav2_msgs REQUIRED)
find_package(nav2_common REQUIRED)

# Custom interfaces
# find_package(urc_interfaces REQUIRED)  # Commented out - not actually used in code

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/DriveToPose.action"
  "action/FollowPath.action"
  "action/SetVelocity.action"
  "action/CancelMotion.action"
  "msg/RoverStatus.msg"
  "msg/DriveCommand.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    action_msgs
)

# Python modules and executables are handled by setup.py
# The setup.py file manages Python package installation and entry points

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

# Install resource files
install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource/
)

# Install test files
install(DIRECTORY test/
  DESTINATION share/${PROJECT_NAME}/test/
)

# Linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(rover_control_test test/)
endif()

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_package() 
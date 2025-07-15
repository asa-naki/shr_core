cmake_minimum_required(VERSION 3.8)
project(ddt_motor_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorStatus.msg"
  DEPENDENCIES std_msgs
)

# Install Python executables
install(PROGRAMS
  ddt_motor_control/ddt_motor_controller_node.py
  ddt_motor_control/ddt_motor_controller_test_node.py
  ddt_motor_control/ddt_motor_scan_control_node.py
  ddt_motor_control/laser_to_pointcloud_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

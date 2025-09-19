cmake_minimum_required(VERSION 3.8)
project(motor_control_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(joy REQUIRED)

# Include directories
include_directories(include)

# Create shared library with implementation
add_library(motor_control_lib SHARED
  src/ddt_motor_lib.cpp
  src/differential_drive.cpp
  src/motor_manager.cpp
  src/servo_control.cpp
)

target_include_directories(motor_control_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(motor_control_lib
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

# Create servo control node executable
add_executable(servo_control_node src/servo_control_node.cpp)
target_link_libraries(servo_control_node motor_control_lib)
ament_target_dependencies(servo_control_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  joy
)

# Create servo test node executable
add_executable(servo_test_node src/servo_test_node.cpp)
ament_target_dependencies(servo_test_node
  rclcpp
  std_msgs
  geometry_msgs
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)



# Export the library
install(TARGETS motor_control_lib
  EXPORT motor_control_libTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install executables separately
install(TARGETS servo_control_node servo_test_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(motor_control_libTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs geometry_msgs sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

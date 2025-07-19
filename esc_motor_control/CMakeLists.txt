cmake_minimum_required(VERSION 3.8)
project(esc_motor_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(include)

# ESC Motor Control Node
add_executable(esc_motor_control_node src/esc_motor_control_node.cpp src/esc_motor_control_component.cpp)
ament_target_dependencies(esc_motor_control_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

# Install executables
install(TARGETS
  esc_motor_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Unit tests for joy control
  ament_add_gtest(test_joy_control
    test/test_joy_control.cpp
    src/esc_motor_control_component.cpp
  )
  ament_target_dependencies(test_joy_control
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
  )
  target_include_directories(test_joy_control PRIVATE include)

endif()

ament_package()

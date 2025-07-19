cmake_minimum_required(VERSION 3.8)
project(joy_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(joy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

# Create the component library
add_library(joy_controller_component SHARED
  src/joy_controller_component.cpp
)

target_compile_definitions(joy_controller_component
  PRIVATE "JOY_CONTROLLER_BUILDING_DLL")

ament_target_dependencies(joy_controller_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
)

# Register the component
rclcpp_components_register_nodes(joy_controller_component
  "joy_controller::JoyControllerComponent")

# Create the executable
add_executable(joy_controller_node
  src/joy_controller_node.cpp
)

ament_target_dependencies(joy_controller_node
  rclcpp
  rclcpp_components
)

target_link_libraries(joy_controller_node joy_controller_component)

# Install targets
install(TARGETS
  joy_controller_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  joy_controller_node
  DESTINATION lib/joy_controller
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch/
  DESTINATION share/joy_controller/
)

install(DIRECTORY
  config/
  DESTINATION share/joy_controller/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(joy_controller_component)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(motor_control_app)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(motor_control_lib REQUIRED)
find_package(joy REQUIRED)

# Unified motor control node (コメントアウト)
# add_executable(unified_motor_control_node src/unified_motor_control_node.cpp)
# target_include_directories(unified_motor_control_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# ament_target_dependencies(unified_motor_control_node
#   rclcpp
#   std_msgs
#   geometry_msgs
#   sensor_msgs
#   motor_control_lib
# )

# Simple test node
add_executable(simple_test_node src/simple_test_node.cpp)
target_include_directories(simple_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(simple_test_node
  motor_control_lib
)

# Simple motor app
add_executable(simple_motor_app src/simple_motor_app.cpp)
target_include_directories(simple_motor_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(simple_motor_app
  rclcpp
  std_msgs
  geometry_msgs
)

# Drive Component (as a shared library for component loading)
add_library(drive_component SHARED
  src/drive_component.cpp
)
target_include_directories(drive_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(drive_component
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  motor_control_lib
)

# Register the component
rclcpp_components_register_nodes(drive_component "motor_control_app::DriveComponent")

# Drive Component Node (standalone executable)
add_executable(drive_component_node src/drive_component_node.cpp)
target_include_directories(drive_component_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(drive_component_node drive_component)
ament_target_dependencies(drive_component_node
  rclcpp
  motor_control_lib
)

# Individual Motor App (for testing individual motor control)
add_executable(individual_motor_app src/individual_motor_app.cpp)
target_include_directories(individual_motor_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(individual_motor_app
  rclcpp
  std_msgs
  motor_control_lib
)

# Unified Motor App (using MotorManager)
add_executable(unified_motor_app src/unified_motor_app.cpp)
target_include_directories(unified_motor_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(unified_motor_app
  rclcpp
  std_msgs
  geometry_msgs
  motor_control_lib
)

# Shot Component (as a shared library for component loading)
add_library(shot_component SHARED
  src/shot_component.cpp
)
target_include_directories(shot_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(shot_component
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  sensor_msgs
  motor_control_lib
  joy
)

# Register the shot component
rclcpp_components_register_nodes(shot_component "motor_control_app::ShotComponent")

# Shot Component Node (standalone executable)
add_executable(shot_component_node src/shot_component_node.cpp)
target_include_directories(shot_component_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(shot_component_node shot_component)
ament_target_dependencies(shot_component_node
  rclcpp
  motor_control_lib
)

# Install executables and libraries
install(TARGETS simple_test_node simple_motor_app drive_component_node individual_motor_app unified_motor_app shot_component_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS drive_component shot_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install config and launch files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

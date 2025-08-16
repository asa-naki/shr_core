cmake_minimum_required(VERSION 3.8)
project(ddt_motor_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Create component library
ament_auto_add_library(ddt_motor_controller_component SHARED
  src/ddt_motor_controller_component.cpp
)

# Register component
rclcpp_components_register_nodes(ddt_motor_controller_component "ddt_motor_control_cpp::DdtMotorControllerComponent")

# Create standalone executable
ament_auto_add_executable(ddt_motor_controller_node
  src/ddt_motor_controller_node.cpp
)

target_link_libraries(ddt_motor_controller_node
  ddt_motor_controller_component
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()

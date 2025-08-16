#include <rclcpp/rclcpp.hpp>

#include "ddt_motor_control_cpp/ddt_motor_controller_component.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node =
      std::make_shared<ddt_motor_control_cpp::DdtMotorControllerComponent>(rclcpp::NodeOptions());

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

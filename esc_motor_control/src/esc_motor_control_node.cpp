#include <rclcpp/rclcpp.hpp>
#include "esc_motor_control/esc_motor_control_component.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<esc_motor_control::ESCMotorControlComponent>(rclcpp::NodeOptions());

  RCLCPP_INFO(node->get_logger(), "ESC Motor Control Node started");

  try {
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Exception caught: %s", e.what());
  }

  RCLCPP_INFO(node->get_logger(), "ESC Motor Control Node shutting down");
  rclcpp::shutdown();
  return 0;
}

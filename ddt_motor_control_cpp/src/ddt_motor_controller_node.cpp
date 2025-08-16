#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>

#include "ddt_motor_control_cpp/ddt_motor_controller_component.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node =
      std::make_shared<ddt_motor_control_cpp::DdtMotorControllerComponent>(rclcpp::NodeOptions());

  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ddt_motor_controller_node"), "例外が発生しました: %s",
                 e.what());
  }

  rclcpp::shutdown();
  return 0;
}

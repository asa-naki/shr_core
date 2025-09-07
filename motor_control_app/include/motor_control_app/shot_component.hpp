#ifndef MOTOR_CONTROL_APP__SHOT_COMPONENT_HPP_
#define MOTOR_CONTROL_APP__SHOT_COMPONENT_HPP_

#include <chrono>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

#include "motor_control_lib/servo_control.hpp"

namespace motor_control_app {

class ShotComponent : public rclcpp::Node {
public:
  explicit ShotComponent(const rclcpp::NodeOptions& options);
  virtual ~ShotComponent();

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void aimCallback(const geometry_msgs::msg::Point::SharedPtr msg);
  void fireCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void homeCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void publishCurrentAim();
  void executeShotSequence();

  std::shared_ptr<motor_control_lib::FeetechServoController> servo_controller_;
  std::unique_ptr<motor_control_lib::ShotController> shot_controller_;

  int pan_servo_id_;
  int trigger_servo_id_;
  int fire_button_;
  int pan_up_axis_;
  int pan_down_axis_;
  int pan_step_;
  int fire_position_;
  int home_position_;
  int fire_duration_ms_;

  bool is_shooting_;
  bool last_button_state_;
  float last_pan_up_value_;
  float last_pan_down_value_;
  int current_pan_position_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr aim_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fire_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr home_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr current_aim_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace motor_control_app

#endif  // MOTOR_CONTROL_APP__SHOT_COMPONENT_HPP_

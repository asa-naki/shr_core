// Copyright (c) 2025 SHR Core Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <cmath>
#include <joy_controller/joy_controller_component.hpp>
#include <memory>
#include <vector>

namespace joy_controller {

JoyControllerComponent::JoyControllerComponent(const rclcpp::NodeOptions &options)
    : Node("joy_controller", options),
      message_count_(0) {  // servo position will be initialized after loading parameters
  // Set up parameter callback
  param_handler_ptr_ = this->add_on_set_parameters_callback(
      std::bind(&JoyControllerComponent::paramCallback, this, std::placeholders::_1));

  // Load parameters
  loadParameters();

  // Initialize servo pitch to center position and publish initial servo shot position
  current_servo_position_ = servo_center_position_;

  // Initialize ROS2 interfaces
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoyControllerComponent::joyCallback, this, std::placeholders::_1));

  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/target_twist", 1);
  servo_position_pub_ =
      this->create_publisher<std_msgs::msg::Int32>("/servo_pitch/position_command", 1);
  servo_shot_pub_ = this->create_publisher<std_msgs::msg::Int32>("/servo_shot/position_command", 1);

  // Publish initial positions
  auto servo_pitch_msg = std_msgs::msg::Int32();
  servo_pitch_msg.data = current_servo_position_;
  servo_position_pub_->publish(servo_pitch_msg);

  auto servo_shot_msg = std_msgs::msg::Int32();
  servo_shot_msg.data = servo_shot_position_2_;
  servo_shot_pub_->publish(servo_shot_msg);

  RCLCPP_INFO(this->get_logger(), "Joy Controller Component initialized");
  RCLCPP_INFO(this->get_logger(), "Servo pitch initialized to center position: %d",
              current_servo_position_);
  RCLCPP_INFO(this->get_logger(), "Servo shot initialized to position 2: %d",
              servo_shot_position_2_);
  if (debug_mode_) {
    RCLCPP_INFO(this->get_logger(), "Debug mode enabled");
  }
}

void JoyControllerComponent::loadParameters() {
  // Movement parameters
  declare_parameter("longitudinal_input_ratio", 1.0);
  get_parameter("longitudinal_input_ratio", longitudinal_input_ratio_);

  declare_parameter("lateral_input_ratio", 0.3);
  get_parameter("lateral_input_ratio", lateral_input_ratio_);

  declare_parameter("angular_input_ratio", 1.0);
  get_parameter("angular_input_ratio", angular_input_ratio_);

  // Controller mapping
  declare_parameter("linear_x_axis", 1);
  get_parameter("linear_x_axis", linear_x_axis_);

  declare_parameter("linear_y_axis", 0);
  get_parameter("linear_y_axis", linear_y_axis_);

  declare_parameter("angular_z_axis", 3);
  get_parameter("angular_z_axis", angular_z_axis_);

  // Right stick axis for servo control
  declare_parameter("right_stick_x_axis", 2);
  get_parameter("right_stick_x_axis", right_stick_x_axis_);

  // Button mapping for servo shot
  declare_parameter("servo_shot_button_1", 1);
  get_parameter("servo_shot_button_1", servo_shot_button_1_);

  declare_parameter("servo_shot_button_2", 0);
  get_parameter("servo_shot_button_2", servo_shot_button_2_);

  // Servo control parameters
  declare_parameter("servo_min_position", 0);
  get_parameter("servo_min_position", servo_min_position_);

  declare_parameter("servo_max_position", 4096);
  get_parameter("servo_max_position", servo_max_position_);

  declare_parameter("servo_center_position", 2048);
  get_parameter("servo_center_position", servo_center_position_);

  declare_parameter("servo_deadzone_threshold", 0.1);
  get_parameter("servo_deadzone_threshold", servo_deadzone_threshold_);

  declare_parameter("servo_increment_rate", 10.0);
  get_parameter("servo_increment_rate", servo_increment_rate_);

  // Servo shot positions
  declare_parameter("servo_shot_position_1", 1024);
  get_parameter("servo_shot_position_1", servo_shot_position_1_);

  declare_parameter("servo_shot_position_2", 3072);
  get_parameter("servo_shot_position_2", servo_shot_position_2_);

  // Debug mode
  declare_parameter("debug_mode", false);
  get_parameter("debug_mode", debug_mode_);
}

void JoyControllerComponent::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  if (!msg || msg->axes.empty() || msg->buttons.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received invalid joy message");
    return;
  }

  message_count_++;

  if (debug_mode_ && message_count_ % 50 == 0) {
    RCLCPP_INFO(this->get_logger(), "Received %d joy messages", message_count_);
  }

  // Update button states and process button actions
  updateButtonStates(msg);
  processButtonActions();

  publishTwist(msg);
  publishServoPosition(msg);
}

void JoyControllerComponent::updateButtonStates(const sensor_msgs::msg::Joy::SharedPtr msg) {
  auto now = this->now();

  for (size_t i = 0; i < msg->buttons.size(); ++i) {
    int button_index = static_cast<int>(i);

    // Initialize button info if not exists
    if (button_states_.find(button_index) == button_states_.end()) {
      button_states_[button_index] = ButtonInfo{false, false, ButtonState::RELEASED, now, now};
    }

    auto &button_info = button_states_[button_index];
    button_info.previous_state = button_info.current_state;
    button_info.current_state = (msg->buttons[i] == 1);

    // Update button state
    if (button_info.current_state && !button_info.previous_state) {
      button_info.button_state = ButtonState::PRESSED;
      button_info.last_press_time = now;
    } else if (!button_info.current_state && button_info.previous_state) {
      button_info.button_state = ButtonState::RELEASED;
      button_info.last_release_time = now;
    } else if (button_info.current_state && button_info.previous_state) {
      button_info.button_state = ButtonState::HELD;
    } else {
      button_info.button_state = ButtonState::RELEASED;
    }
  }
}

void JoyControllerComponent::processButtonActions() {
  // Servo shot button 1
  if (isButtonJustPressed(servo_shot_button_1_)) {
    publishServoShot(servo_shot_position_1_);
    if (debug_mode_) {
      RCLCPP_INFO(this->get_logger(), "Button %d pressed: servo shot position %d",
                  servo_shot_button_1_, servo_shot_position_1_);
    }
  }

  // Servo shot button 2
  if (isButtonJustPressed(servo_shot_button_2_)) {
    publishServoShot(servo_shot_position_2_);
    if (debug_mode_) {
      RCLCPP_INFO(this->get_logger(), "Button %d pressed: servo shot position %d",
                  servo_shot_button_2_, servo_shot_position_2_);
    }
  }
}

bool JoyControllerComponent::isButtonJustPressed(int button_index) {
  auto it = button_states_.find(button_index);
  return (it != button_states_.end()) && (it->second.button_state == ButtonState::PRESSED);
}

void JoyControllerComponent::publishServoShot(int position) {
  auto servo_msg = std_msgs::msg::Int32();
  servo_msg.data = position;
  servo_shot_pub_->publish(servo_msg);

  if (debug_mode_) {
    RCLCPP_INFO(this->get_logger(), "Published servo shot position: %d", position);
  }
}

void JoyControllerComponent::publishTwist(const sensor_msgs::msg::Joy::SharedPtr msg) {
  // joy_to_twist style implementation - simplified and direct mapping
  if (msg->axes.size() < 4) {
    return;
  }

  auto twist = geometry_msgs::msg::Twist();

  // Direct mapping like joy_to_twist: axes[1] -> linear.x, axes[3] -> angular.z
  twist.linear.x = msg->axes[linear_x_axis_] * longitudinal_input_ratio_;
  twist.angular.z = msg->axes[angular_z_axis_] * angular_input_ratio_;

  // Optional: Apply lateral movement if configured
  if (linear_y_axis_ >= 0 && static_cast<size_t>(linear_y_axis_) < msg->axes.size()) {
    twist.linear.y = msg->axes[linear_y_axis_] * lateral_input_ratio_;
  }

  twist_pub_->publish(twist);

  if (debug_mode_ && message_count_ % 50 == 0) {
    RCLCPP_INFO(this->get_logger(),
                "Published twist: linear=[%.3f, %.3f, %.3f], angular=[%.3f, %.3f, %.3f]",
                twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y,
                twist.angular.z);
  }
}

void JoyControllerComponent::publishServoPosition(const sensor_msgs::msg::Joy::SharedPtr msg) {
  // Check if right stick x axis is available
  if (right_stick_x_axis_ < 0 || static_cast<size_t>(right_stick_x_axis_) >= msg->axes.size()) {
    return;
  }

  // Get right stick x value (-1.0 to 1.0)
  double stick_value = msg->axes[right_stick_x_axis_];

  // Apply deadzone
  if (std::abs(stick_value) < servo_deadzone_threshold_) {
    stick_value = 0.0;
  }

  // Calculate increment based on stick value and increment rate
  if (stick_value != 0.0) {
    // Calculate increment (proportional to stick value and increment rate)
    // Invert the stick value so right decreases and left increases
    double increment = -stick_value * servo_increment_rate_;

    // Update current servo position
    current_servo_position_ += static_cast<int>(increment);

    // Clamp to valid range
    current_servo_position_ =
        std::max(servo_min_position_, std::min(servo_max_position_, current_servo_position_));
  }

  // Publish servo position command
  auto servo_msg = std_msgs::msg::Int32();
  servo_msg.data = current_servo_position_;
  servo_position_pub_->publish(servo_msg);

  if (debug_mode_ && message_count_ % 50 == 0) {
    RCLCPP_INFO(this->get_logger(), "Right stick: %.3f -> Servo position: %d", stick_value,
                current_servo_position_);
  }
}

rcl_interfaces::msg::SetParametersResult JoyControllerComponent::paramCallback(
    const std::vector<rclcpp::Parameter> &params) {
  auto result = std::make_shared<rcl_interfaces::msg::SetParametersResult>();

  try {
    for (const auto &param : params) {
      const std::string &name = param.get_name();

      if (name == "longitudinal_input_ratio") {
        longitudinal_input_ratio_ = param.get_value<double>();
      } else if (name == "lateral_input_ratio") {
        lateral_input_ratio_ = param.get_value<double>();
      } else if (name == "angular_input_ratio") {
        angular_input_ratio_ = param.get_value<double>();
      } else if (name == "debug_mode") {
        debug_mode_ = param.get_value<bool>();
      } else if (name == "servo_min_position") {
        servo_min_position_ = param.get_value<int>();
      } else if (name == "servo_max_position") {
        servo_max_position_ = param.get_value<int>();
      } else if (name == "servo_center_position") {
        servo_center_position_ = param.get_value<int>();
      } else if (name == "servo_deadzone_threshold") {
        servo_deadzone_threshold_ = param.get_value<double>();
      } else if (name == "servo_increment_rate") {
        servo_increment_rate_ = param.get_value<double>();
      } else if (name == "right_stick_x_axis") {
        right_stick_x_axis_ = param.get_value<int>();
      } else if (name == "servo_shot_button_1") {
        servo_shot_button_1_ = param.get_value<int>();
      } else if (name == "servo_shot_button_2") {
        servo_shot_button_2_ = param.get_value<int>();
      } else if (name == "servo_shot_position_1") {
        servo_shot_position_1_ = param.get_value<int>();
      } else if (name == "servo_shot_position_2") {
        servo_shot_position_2_ = param.get_value<int>();
      }
    }

    result->successful = true;
    result->reason = "Parameters updated successfully";
  } catch (const std::exception &e) {
    result->successful = false;
    result->reason = std::string("Failed to update parameters: ") + e.what();
    RCLCPP_ERROR(this->get_logger(), "%s", result->reason.c_str());
  }

  return *result;
}

}  // namespace joy_controller

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(joy_controller::JoyControllerComponent)

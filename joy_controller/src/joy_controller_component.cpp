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

#include <joy_controller/joy_controller_component.hpp>
#include <memory>
#include <vector>
#include <algorithm>
#include <cmath>

namespace joy_controller
{

    JoyControllerComponent::JoyControllerComponent(const rclcpp::NodeOptions &options)
        : Node("joy_controller", options),
          emergency_stop_active_(false),
          turbo_mode_active_(false),
          precision_mode_active_(false),
          current_speed_multiplier_(1.0),
          message_count_(0)
    {
        // Set up parameter callback
        param_handler_ptr_ = this->add_on_set_parameters_callback(
            std::bind(&JoyControllerComponent::paramCallback, this, std::placeholders::_1));

        // Load parameters
        loadParameters();

        // Initialize ROS2 interfaces
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyControllerComponent::joyCallback, this, std::placeholders::_1));

        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        // emergency_stop_pub_ = this->create_publisher<std_msgs::msg::Bool>("/emergency_stop", 10);
        // turbo_mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("/turbo_mode", 10);
        // precision_mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("/precision_mode", 10);
        // speed_multiplier_pub_ = this->create_publisher<std_msgs::msg::Float64>("/speed_multiplier", 10);

        // Initialize timer for periodic tasks (safety checks, etc.)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]()
            {
                // Check for joystick timeout
                if (enable_safety_checks_)
                {
                    auto now = this->now();
                    if ((now - last_joy_msg_time_).seconds() > joy_timeout_duration_)
                    {
                        publishEmergencyStop();
                    }
                }
            });

        // Initialize last message time
        last_joy_msg_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Joy Controller Component initialized");
        if (debug_mode_)
        {
            RCLCPP_INFO(this->get_logger(), "Debug mode enabled");
        }
    }

    void JoyControllerComponent::loadParameters()
    {
        // Movement parameters
        declare_parameter("longitudinal_input_ratio", 1.0);
        get_parameter("longitudinal_input_ratio", longitudinal_input_ratio_);

        declare_parameter("lateral_input_ratio", 0.3);
        get_parameter("lateral_input_ratio", lateral_input_ratio_);

        declare_parameter("angular_input_ratio", 1.0);
        get_parameter("angular_input_ratio", angular_input_ratio_);

        declare_parameter("turbo_multiplier", 2.0);
        get_parameter("turbo_multiplier", turbo_multiplier_);

        declare_parameter("precision_multiplier", 0.3);
        get_parameter("precision_multiplier", precision_multiplier_);

        declare_parameter("deadzone_threshold", 0.1);
        get_parameter("deadzone_threshold", deadzone_threshold_);

        // Controller mapping
        declare_parameter("linear_x_axis", 1);
        get_parameter("linear_x_axis", linear_x_axis_);

        declare_parameter("linear_y_axis", 0);
        get_parameter("linear_y_axis", linear_y_axis_);

        declare_parameter("angular_z_axis", 3);
        get_parameter("angular_z_axis", angular_z_axis_);

        declare_parameter("emergency_stop_button", 6);
        get_parameter("emergency_stop_button", emergency_stop_button_);

        declare_parameter("turbo_button", 4);
        get_parameter("turbo_button", turbo_button_);

        declare_parameter("precision_button", 5);
        get_parameter("precision_button", precision_button_);

        // Safety parameters
        declare_parameter("enable_safety_checks", true);
        get_parameter("enable_safety_checks", enable_safety_checks_);

        declare_parameter("max_linear_velocity", 2.0);
        get_parameter("max_linear_velocity", max_linear_velocity_);

        declare_parameter("max_angular_velocity", 2.0);
        get_parameter("max_angular_velocity", max_angular_velocity_);

        declare_parameter("joy_timeout_duration", 1.0);
        get_parameter("joy_timeout_duration", joy_timeout_duration_);

        // Debug mode
        declare_parameter("debug_mode", false);
        get_parameter("debug_mode", debug_mode_);
    }

    void JoyControllerComponent::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (!msg || msg->axes.empty() || msg->buttons.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received invalid joy message");
            return;
        }

        last_joy_msg_time_ = this->now();
        message_count_++;

        if (debug_mode_ && message_count_ % 50 == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Received %d joy messages", message_count_);
        }

        // Update button states
        updateButtonStates(msg);

        // Process button actions if not in emergency stop
        if (!emergency_stop_active_)
        {
            publishTwist(msg);
        }
        else
        {
            // Publish zero twist when emergency stop is active
            auto twist = geometry_msgs::msg::Twist();
            twist_pub_->publish(twist);
        }
    }

    void JoyControllerComponent::updateButtonStates(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto now = this->now();

        for (size_t i = 0; i < msg->buttons.size(); ++i)
        {
            int button_index = static_cast<int>(i);

            // Initialize button info if not exists
            if (button_states_.find(button_index) == button_states_.end())
            {
                button_states_[button_index] = ButtonInfo{false, false, ButtonState::RELEASED, now, now};
            }

            auto &button_info = button_states_[button_index];
            button_info.previous_state = button_info.current_state;
            button_info.current_state = (msg->buttons[i] == 1);

            // Update button state
            if (button_info.current_state && !button_info.previous_state)
            {
                button_info.button_state = ButtonState::PRESSED;
                button_info.last_press_time = now;
            }
            else if (!button_info.current_state && button_info.previous_state)
            {
                button_info.button_state = ButtonState::RELEASED;
                button_info.last_release_time = now;
            }
            else if (button_info.current_state && button_info.previous_state)
            {
                button_info.button_state = ButtonState::HELD;
            }
            else
            {
                button_info.button_state = ButtonState::RELEASED;
            }
        }
    }

    void JoyControllerComponent::processButtonActions()
    {
        // Emergency stop button
        /*
        if (isButtonJustPressed(emergency_stop_button_)) {
          emergency_stop_active_ = !emergency_stop_active_;
          auto msg = std_msgs::msg::Bool();
          msg.data = emergency_stop_active_;
          emergency_stop_pub_->publish(msg);

          if (emergency_stop_active_) {
            RCLCPP_WARN(this->get_logger(), "Emergency stop activated!");
          } else {
            RCLCPP_INFO(this->get_logger(), "Emergency stop deactivated");
          }
        }
        */

        // Turbo mode button
        /*
        if (isButtonJustPressed(turbo_button_)) {
          toggleTurboMode();
        }
        */

        // Precision mode button
        /*
        if (isButtonJustPressed(precision_button_)) {
          togglePrecisionMode();
        }
        */
    }

    void JoyControllerComponent::publishTwist(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto twist = geometry_msgs::msg::Twist();

        // Get axis values with deadzone filtering
        double linear_x = (std::abs(msg->axes[linear_x_axis_]) > deadzone_threshold_) ? msg->axes[linear_x_axis_] : 0.0;
        double linear_y = (std::abs(msg->axes[linear_y_axis_]) > deadzone_threshold_) ? msg->axes[linear_y_axis_] : 0.0;
        double angular_z = (std::abs(msg->axes[angular_z_axis_]) > deadzone_threshold_) ? msg->axes[angular_z_axis_] : 0.0;

        // Apply input ratios
        twist.linear.x = linear_x * longitudinal_input_ratio_;
        twist.linear.y = linear_y * lateral_input_ratio_;
        twist.angular.z = angular_z * angular_input_ratio_;

        // Apply mode multipliers (simplified - always use base multiplier)
        // twist.linear.x *= current_speed_multiplier_;
        // twist.linear.y *= current_speed_multiplier_;
        // twist.angular.z *= current_speed_multiplier_;

        // Apply safety limits
        if (enable_safety_checks_)
        {
            double linear_magnitude = std::sqrt(twist.linear.x * twist.linear.x +
                                                twist.linear.y * twist.linear.y);
            if (linear_magnitude > max_linear_velocity_)
            {
                double scale = max_linear_velocity_ / linear_magnitude;
                twist.linear.x *= scale;
                twist.linear.y *= scale;
            }

            if (std::abs(twist.angular.z) > max_angular_velocity_)
            {
                twist.angular.z = std::copysign(max_angular_velocity_, twist.angular.z);
            }
        }

        twist_pub_->publish(twist);

        if (debug_mode_ && message_count_ % 50 == 0)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Published twist: linear=[%.3f, %.3f, %.3f], angular=[%.3f, %.3f, %.3f]",
                        twist.linear.x, twist.linear.y, twist.linear.z,
                        twist.angular.x, twist.angular.y, twist.angular.z);
        }
    }

    void JoyControllerComponent::publishEmergencyStop()
    {
        /*
        if (!emergency_stop_active_) {
          emergency_stop_active_ = true;
          auto msg = std_msgs::msg::Bool();
          msg.data = true;
          emergency_stop_pub_->publish(msg);

          // Publish zero twist
          auto twist = geometry_msgs::msg::Twist();
          twist_pub_->publish(twist);

          RCLCPP_WARN(this->get_logger(), "Emergency stop activated due to joystick timeout!");
        }
        */
    }

    void JoyControllerComponent::toggleTurboMode()
    {
        /*
        turbo_mode_active_ = !turbo_mode_active_;

        if (turbo_mode_active_) {
          precision_mode_active_ = false;  // Disable precision mode
          current_speed_multiplier_ = turbo_multiplier_;
        } else {
          current_speed_multiplier_ = 1.0;
        }

        // Publish state messages
        auto turbo_msg = std_msgs::msg::Bool();
        turbo_msg.data = turbo_mode_active_;
        turbo_mode_pub_->publish(turbo_msg);

        auto precision_msg = std_msgs::msg::Bool();
        precision_msg.data = precision_mode_active_;
        precision_mode_pub_->publish(precision_msg);

        auto speed_msg = std_msgs::msg::Float64();
        speed_msg.data = current_speed_multiplier_;
        speed_multiplier_pub_->publish(speed_msg);

        RCLCPP_INFO(this->get_logger(), "Turbo mode: %s (multiplier: %.2f)",
                    turbo_mode_active_ ? "ON" : "OFF", current_speed_multiplier_);
        */
    }

    void JoyControllerComponent::togglePrecisionMode()
    {
        /*
        precision_mode_active_ = !precision_mode_active_;

        if (precision_mode_active_) {
          turbo_mode_active_ = false;  // Disable turbo mode
          current_speed_multiplier_ = precision_multiplier_;
        } else {
          current_speed_multiplier_ = 1.0;
        }

        // Publish state messages
        auto turbo_msg = std_msgs::msg::Bool();
        turbo_msg.data = turbo_mode_active_;
        turbo_mode_pub_->publish(turbo_msg);

        auto precision_msg = std_msgs::msg::Bool();
        precision_msg.data = precision_mode_active_;
        precision_mode_pub_->publish(precision_msg);

        auto speed_msg = std_msgs::msg::Float64();
        speed_msg.data = current_speed_multiplier_;
        speed_multiplier_pub_->publish(speed_msg);

        RCLCPP_INFO(this->get_logger(), "Precision mode: %s (multiplier: %.2f)",
                    precision_mode_active_ ? "ON" : "OFF", current_speed_multiplier_);
        */
    }

    bool JoyControllerComponent::isButtonPressed(int button_index)
    {
        auto it = button_states_.find(button_index);
        return (it != button_states_.end()) && it->second.current_state;
    }

    bool JoyControllerComponent::isButtonJustPressed(int button_index)
    {
        auto it = button_states_.find(button_index);
        return (it != button_states_.end()) && (it->second.button_state == ButtonState::PRESSED);
    }

    bool JoyControllerComponent::isButtonJustReleased(int button_index)
    {
        auto it = button_states_.find(button_index);
        return (it != button_states_.end()) && (it->second.button_state == ButtonState::RELEASED);
    }

    rcl_interfaces::msg::SetParametersResult JoyControllerComponent::paramCallback(
        const std::vector<rclcpp::Parameter> &params)
    {
        auto result = std::make_shared<rcl_interfaces::msg::SetParametersResult>();

        try
        {
            for (const auto &param : params)
            {
                const std::string &name = param.get_name();

                if (name == "longitudinal_input_ratio")
                {
                    longitudinal_input_ratio_ = param.get_value<double>();
                }
                else if (name == "lateral_input_ratio")
                {
                    lateral_input_ratio_ = param.get_value<double>();
                }
                else if (name == "angular_input_ratio")
                {
                    angular_input_ratio_ = param.get_value<double>();
                }
                else if (name == "turbo_multiplier")
                {
                    turbo_multiplier_ = param.get_value<double>();
                }
                else if (name == "precision_multiplier")
                {
                    precision_multiplier_ = param.get_value<double>();
                }
                else if (name == "deadzone_threshold")
                {
                    deadzone_threshold_ = param.get_value<double>();
                }
                else if (name == "max_linear_velocity")
                {
                    max_linear_velocity_ = param.get_value<double>();
                }
                else if (name == "max_angular_velocity")
                {
                    max_angular_velocity_ = param.get_value<double>();
                }
                else if (name == "enable_safety_checks")
                {
                    enable_safety_checks_ = param.get_value<bool>();
                }
                else if (name == "debug_mode")
                {
                    debug_mode_ = param.get_value<bool>();
                }
            }

            result->successful = true;
            result->reason = "Parameters updated successfully";
        }
        catch (const std::exception &e)
        {
            result->successful = false;
            result->reason = std::string("Failed to update parameters: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "%s", result->reason.c_str());
        }

        return *result;
    }

} // namespace joy_controller

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(joy_controller::JoyControllerComponent)

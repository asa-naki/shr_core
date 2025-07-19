#include "esc_motor_control/esc_motor_control_component.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace esc_motor_control
{

ESCMotorControlComponent::ESCMotorControlComponent(const rclcpp::NodeOptions & options)
: Node("esc_motor_control", options),
  current_speed_(0.0),
  emergency_stop_active_(false),
  full_speed_active_(false),
  pwm_initialized_(false),
  test_mode_(false),
  simulated_pwm_duty_(0.0)
{
  // Declare parameters
  this->declare_parameter("pwm_pin", 13);
  this->declare_parameter("max_speed", 1.0);
  this->declare_parameter("min_speed", -1.0);
  this->declare_parameter("pwm_frequency", 50);
  this->declare_parameter("enable_safety_stop", true);
  this->declare_parameter("safety_timeout", 1.0);
  this->declare_parameter("pwm_chip", 0);
  this->declare_parameter("pwm_channel", 0);
  this->declare_parameter("full_speed_button", 1);        // Button 1 (typically 'B' button)
  this->declare_parameter("full_speed_value", 1.0);       // Full forward speed
  this->declare_parameter("test_mode", false);            // Test mode for mocking PWM
  this->declare_parameter("joy_topic", "joy");            // Joy topic name

  // Get parameters
  pwm_pin_ = this->get_parameter("pwm_pin").as_int();
  max_speed_ = this->get_parameter("max_speed").as_double();
  min_speed_ = this->get_parameter("min_speed").as_double();
  pwm_frequency_ = this->get_parameter("pwm_frequency").as_int();
  enable_safety_stop_ = this->get_parameter("enable_safety_stop").as_bool();
  safety_timeout_ = this->get_parameter("safety_timeout").as_double();
  full_speed_button_ = this->get_parameter("full_speed_button").as_int();
  full_speed_value_ = this->get_parameter("full_speed_value").as_double();
  test_mode_ = this->get_parameter("test_mode").as_bool();
  std::string joy_topic = this->get_parameter("joy_topic").as_string();

  int pwm_chip = this->get_parameter("pwm_chip").as_int();
  int pwm_channel = this->get_parameter("pwm_channel").as_int();

  // Setup PWM paths
  pwm_chip_path_ = "/sys/class/pwm/pwmchip" + std::to_string(pwm_chip);
  pwm_channel_path_ = pwm_chip_path_ + "/pwm" + std::to_string(pwm_channel);

  RCLCPP_INFO(this->get_logger(), "ESC Motor Control Component initialized");
  RCLCPP_INFO(this->get_logger(), "PWM Pin: %d", pwm_pin_);
  RCLCPP_INFO(this->get_logger(), "Speed Range: [%.2f, %.2f]", min_speed_, max_speed_);
  RCLCPP_INFO(this->get_logger(), "PWM Frequency: %d Hz", pwm_frequency_);
  RCLCPP_INFO(
    this->get_logger(), "Full Speed Button: %d (Speed: %.2f)", full_speed_button_,
    full_speed_value_);

  // Initialize PWM
  initializePWM();

  // Create subscribers
  twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&ESCMotorControlComponent::twistCallback, this, std::placeholders::_1));

  joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
    joy_topic, 10,
    std::bind(&ESCMotorControlComponent::joyCallback, this, std::placeholders::_1));

  motor_speed_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
    "motor_speed", 10,
    std::bind(&ESCMotorControlComponent::motorSpeedCallback, this, std::placeholders::_1));

  // Create publisher
  motor_feedback_publisher_ = this->create_publisher<std_msgs::msg::Float32>("motor_feedback", 10);

  // Create timer for safety check
  if (enable_safety_stop_) {
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]()
      {
        auto now = this->get_clock()->now();
        auto time_diff = (now - last_command_time_).seconds();
        if (time_diff > safety_timeout_) {
          if (!emergency_stop_active_) {
            RCLCPP_WARN(this->get_logger(), "Safety timeout triggered - stopping motor");
            emergencyStop();
          }
        }
      });
  }

  last_command_time_ = this->get_clock()->now();
}

void ESCMotorControlComponent::initializePWM()
{
  if (test_mode_) {
    pwm_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "PWM initialized in test mode (no hardware)");
    return;
  }

  try {
    // Export PWM channel
    std::ofstream export_file(pwm_chip_path_ + "/export");
    if (export_file.is_open()) {
      export_file << "0";
      export_file.close();
    } else {
      RCLCPP_WARN(this->get_logger(), "PWM channel may already be exported");
    }

    // Wait a bit for the system to set up the channel
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Set PWM period (frequency)
    int period_ns = 1000000000 / pwm_frequency_;         // Convert to nanoseconds
    std::ofstream period_file(pwm_channel_path_ + "/period");
    if (period_file.is_open()) {
      period_file << period_ns;
      period_file.close();
      RCLCPP_INFO(this->get_logger(), "PWM period set to %d ns (%d Hz)", period_ns, pwm_frequency_);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to set PWM period");
      return;
    }

    // Set initial duty cycle to neutral (1.5ms for servo/ESC)
    int neutral_duty_ns = 1500000;         // 1.5ms in nanoseconds
    std::ofstream duty_file(pwm_channel_path_ + "/duty_cycle");
    if (duty_file.is_open()) {
      duty_file << neutral_duty_ns;
      duty_file.close();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to set initial duty cycle");
      return;
    }

    // Enable PWM
    std::ofstream enable_file(pwm_channel_path_ + "/enable");
    if (enable_file.is_open()) {
      enable_file << "1";
      enable_file.close();
      pwm_initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "PWM initialized successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to enable PWM");
      return;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "PWM initialization failed: %s", e.what());
  }
}

void ESCMotorControlComponent::setPWMDutyCycle(int /* pin */, float duty_cycle)
{
  if (!pwm_initialized_) {
    RCLCPP_WARN(this->get_logger(), "PWM not initialized");
    return;
  }

  if (test_mode_) {
    simulated_pwm_duty_ = duty_cycle;
    return;
  }

  try {
    // Convert duty cycle to nanoseconds
    // For ESC: 1ms = full reverse, 1.5ms = neutral, 2ms = full forward
    int base_pulse_ns = 1000000;          // 1ms base
    int pulse_range_ns = 1000000;         // 1ms range (1ms to 2ms)

    // Clamp duty cycle to [-1, 1]
    duty_cycle = std::max(-1.0f, std::min(1.0f, duty_cycle));

    // Calculate pulse width: 1ms + (duty_cycle + 1) * 0.5ms
    int pulse_width_ns = base_pulse_ns +
      static_cast<int>((duty_cycle + 1.0) * 0.5 * pulse_range_ns);

    std::ofstream duty_file(pwm_channel_path_ + "/duty_cycle");
    if (duty_file.is_open()) {
      duty_file << pulse_width_ns;
      duty_file.close();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to write PWM duty cycle");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "PWM duty cycle setting failed: %s", e.what());
  }
}

void ESCMotorControlComponent::cleanupPWM()
{
  if (pwm_initialized_) {
    try {
      // Set to neutral position
      setPWMDutyCycle(pwm_pin_, 0.0);

      // Disable PWM
      std::ofstream enable_file(pwm_channel_path_ + "/enable");
      if (enable_file.is_open()) {
        enable_file << "0";
        enable_file.close();
      }

      // Unexport PWM channel
      std::ofstream unexport_file(pwm_chip_path_ + "/unexport");
      if (unexport_file.is_open()) {
        unexport_file << "0";
        unexport_file.close();
      }

      RCLCPP_INFO(this->get_logger(), "PWM cleaned up");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "PWM cleanup failed: %s", e.what());
    }
  }
}

void ESCMotorControlComponent::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_command_time_ = this->get_clock()->now();
  // Don't automatically clear emergency stop from twist messages
  // emergency_stop_active_ = false;

  // Use linear.x for forward/backward motion
  float speed = static_cast<float>(msg->linear.x);
  setMotorSpeed(speed);
}

void ESCMotorControlComponent::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  last_command_time_ = this->get_clock()->now();

  // Check for emergency stop button (button 0, typically 'A' button)
  if (!msg->buttons.empty() && msg->buttons[0] == 1) {
    if (!emergency_stop_active_) {
      emergencyStop();
    }
    return;
  }

  // Emergency stop can only be cleared by explicitly releasing the emergency stop button
  // and then pressing it again, or by calling clearEmergencyStop()
  // Don't auto-clear just because the button is not currently pressed
  if (emergency_stop_active_) {
    RCLCPP_INFO(this->get_logger(), "Emergency stop is active - ignoring joy input");
    return;
  }

  // Check for full speed button (configurable button, typically 'B' button)
  bool full_speed_button_pressed = false;
  if (msg->buttons.size() > static_cast<size_t>(full_speed_button_) &&
    full_speed_button_ >= 0 &&
    msg->buttons[full_speed_button_] == 1)
  {
    full_speed_button_pressed = true;
    if (!full_speed_active_) {
      RCLCPP_INFO(
        this->get_logger(), "Full speed button pressed - setting speed to %.2f",
        full_speed_value_);
      full_speed_active_ = true;
      setMotorSpeed(full_speed_value_);
    }
  } else {
    // Button not pressed, check if we were in full speed mode
    if (full_speed_active_) {
      full_speed_active_ = false;
      setMotorSpeed(0.0);           // Stop motor when button is released
      RCLCPP_INFO(this->get_logger(), "Full speed button released - motor stopped");
    }
  }

  // Only process normal joystick input if full speed button is not pressed
  if (!full_speed_button_pressed && !full_speed_active_) {
    // Use right trigger (axis 5) for throttle control
    if (msg->axes.size() > 5) {
      // Right trigger goes from 1 (not pressed) to -1 (fully pressed)
      float trigger_value = static_cast<float>(msg->axes[5]);
      float speed = (1.0f - trigger_value) * 0.5f;           // Convert to 0-1 range

      // Use left trigger for reverse (axis 2)
      if (msg->axes.size() > 2) {
        float left_trigger = static_cast<float>(msg->axes[2]);
        float reverse_speed = (1.0f - left_trigger) * 0.5f;
        speed -= reverse_speed;
      }

      setMotorSpeed(speed);
    }
  }
}

void ESCMotorControlComponent::motorSpeedCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  last_command_time_ = this->get_clock()->now();
  // Don't automatically clear emergency stop from motor speed messages
  // emergency_stop_active_ = false;

  setMotorSpeed(msg->data);
}

void ESCMotorControlComponent::setMotorSpeed(float speed)
{
  if (emergency_stop_active_) {
    RCLCPP_WARN(this->get_logger(), "Emergency stop active - ignoring speed command");
    return;
  }

  // Constrain speed to allowed range
  speed = constrainSpeed(speed);
  current_speed_ = speed;

  // Set PWM duty cycle
  setPWMDutyCycle(pwm_pin_, speed);

  // Publish feedback
  auto feedback_msg = std_msgs::msg::Float32();
  feedback_msg.data = current_speed_;
  motor_feedback_publisher_->publish(feedback_msg);

  RCLCPP_DEBUG(this->get_logger(), "Motor speed set to: %.3f", speed);
}

void ESCMotorControlComponent::emergencyStop()
{
  emergency_stop_active_ = true;
  full_speed_active_ = false;       // Reset full speed state
  current_speed_ = 0.0;
  setPWMDutyCycle(pwm_pin_, 0.0);

  RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP ACTIVATED");

  // Publish feedback
  auto feedback_msg = std_msgs::msg::Float32();
  feedback_msg.data = 0.0;
  motor_feedback_publisher_->publish(feedback_msg);
}

float ESCMotorControlComponent::constrainSpeed(float speed)
{
  return std::max(min_speed_, std::min(max_speed_, speed));
}

int ESCMotorControlComponent::speedToPWM(float speed)
{
  // Convert speed (-1 to 1) to PWM value
  // This is a placeholder - actual conversion depends on your ESC
  speed = constrainSpeed(speed);

  // For a typical servo/ESC:
  // -1.0 speed -> 1ms pulse width
  //  0.0 speed -> 1.5ms pulse width
  //  1.0 speed -> 2ms pulse width

  float pulse_width_ms = 1.5 + (speed * 0.5);       // 1.0 to 2.0 ms
  int pulse_width_us = static_cast<int>(pulse_width_ms * 1000);

  return pulse_width_us;
}

ESCMotorControlComponent::~ESCMotorControlComponent()
{
  cleanupPWM();
}

} // namespace esc_motor_control

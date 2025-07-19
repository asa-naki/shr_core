#ifndef ESC_MOTOR_CONTROL__ESC_MOTOR_CONTROL_COMPONENT_HPP_
#define ESC_MOTOR_CONTROL__ESC_MOTOR_CONTROL_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <memory>
#include <fstream>
#include <string>

namespace esc_motor_control
{

class ESCMotorControlComponent : public rclcpp::Node
{
public:
  explicit ESCMotorControlComponent(const rclcpp::NodeOptions & options);
  virtual ~ESCMotorControlComponent();

  // Test-specific methods
  float getCurrentSpeed() const {return current_speed_;}
  bool isTestMode() const {return test_mode_;}
  bool isEmergencyStopActive() const {return emergency_stop_active_;}
  bool isFullSpeedActive() const {return full_speed_active_;}

  // Motor simulation methods for testing
  float getSimulatedPWMDuty() const {return simulated_pwm_duty_;}
  bool isSimulatedMotorRotating() const
  {
    return test_mode_ && std::abs(simulated_pwm_duty_) > 0.05;
  }
  float getSimulatedRotationSpeed() const
  {
    return test_mode_ ? simulated_pwm_duty_ * 1000.0 : 0.0;
  }

  // Emergency stop control for testing
  void clearEmergencyStop() {emergency_stop_active_ = false;}

private:
  // PWM control methods
  void initializePWM();
  void setPWMDutyCycle(int pin, float duty_cycle);
  void cleanupPWM();

  // Callback functions
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void motorSpeedCallback(const std_msgs::msg::Float32::SharedPtr msg);

  // Motor control methods
  void setMotorSpeed(float speed);
  void emergencyStop();

  // Utility methods
  float constrainSpeed(float speed);
  int speedToPWM(float speed);

  // ROS2 subscribers and publishers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr motor_speed_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor_feedback_publisher_;

  // Timer for periodic tasks
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  int pwm_pin_;
  float max_speed_;
  float min_speed_;
  int pwm_frequency_;
  bool enable_safety_stop_;
  float safety_timeout_;
  int full_speed_button_;
  float full_speed_value_;

  // State variables
  float current_speed_;
  bool emergency_stop_active_;
  bool full_speed_active_;
  rclcpp::Time last_command_time_;

  // PWM control files
  std::string pwm_chip_path_;
  std::string pwm_channel_path_;
  bool pwm_initialized_;
  bool test_mode_;       // For testing without actual PWM hardware

  // Test simulation variables
  float simulated_pwm_duty_;
};

} // namespace esc_motor_control

#endif // ESC_MOTOR_CONTROL__ESC_MOTOR_CONTROL_COMPONENT_HPP_

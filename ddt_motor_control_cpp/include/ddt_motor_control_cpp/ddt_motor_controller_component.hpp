#ifndef DDT_MOTOR_CONTROL_CPP__DDT_MOTOR_CONTROLLER_COMPONENT_HPP_
#define DDT_MOTOR_CONTROL_CPP__DDT_MOTOR_CONTROLLER_COMPONENT_HPP_

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <ddt_motor_control/msg/motor_status.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vector>

namespace ddt_motor_control_cpp {

class DdtMotorControllerComponent : public rclcpp::Node {
public:
  explicit DdtMotorControllerComponent(const rclcpp::NodeOptions& options);
  virtual ~DdtMotorControllerComponent();

private:
  // モーター設定（M15データシートより）
  int left_motor_id_;
  int right_motor_id_;
  int max_motor_rpm_;

  // 車輪パラメータ
  double wheel_radius_;
  double wheel_separation_;

  // シリアル通信
  int serial_fd_;
  std::string serial_port_;
  int baud_rate_;

  // ROS2通信
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Publisher<ddt_motor_control::msg::MotorStatus>::SharedPtr motor_status_pub_;

  // タイマー
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr status_publish_timer_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;

  // 状態管理
  int left_motor_velocity_;
  int right_motor_velocity_;
  double left_motor_rotations_;
  double right_motor_rotations_;
  std::chrono::steady_clock::time_point last_twist_time_;
  std::chrono::steady_clock::time_point last_status_time_;

  // M15モーター固有の状態
  struct MotorFeedback {
    uint8_t mode;
    uint16_t current;
    int16_t speed;
    uint8_t angle;
    uint8_t temperature;
    uint8_t fault_code;
  };

  MotorFeedback left_motor_feedback_;
  MotorFeedback right_motor_feedback_;

  // メソッド
  void initializeSerial();
  void closeSerial();
  void setModeVelocity(int motor_id);
  void setMotorVelocity(int motor_id, int velocity_rpm);
  bool requestMotorFeedback(int motor_id);
  void processFeedbackResponse(int motor_id, const std::vector<uint8_t>& response);
  std::pair<double, double> twistToMotorVelocities(double linear_x, double angular_z);
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void updateMotorRotations();
  void publishMotorStatus();
  void stopMotors();
  void watchdogCallback();
  void logStatus();
  void feedbackCallback();
  void checkMotorHealth();

  // ユーティリティ（M15データシート準拠）
  uint8_t crc8Maxim(const std::vector<uint8_t>& data);
  bool sendCommand(const std::vector<uint8_t>& command, int retry_count = 3);
  ssize_t writeSerial(const void* data, size_t size);
  ssize_t readSerial(void* data, size_t size);
};

}  // namespace ddt_motor_control_cpp

#endif  // DDT_MOTOR_CONTROL_CPP__DDT_MOTOR_CONTROLLER_COMPONENT_HPP_

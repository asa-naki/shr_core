#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

#include "motor_control_lib/ddt_motor_lib.hpp"

using namespace std::chrono_literals;

class IndividualMotorNode : public rclcpp::Node {
private:
  std::unique_ptr<motor_control_lib::DdtMotorLib> motor_lib_;

  // ROS 2 通信
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr motor1_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr motor2_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  // パラメータ
  std::string serial_port_;
  int baud_rate_;
  double wheel_radius_;
  double wheel_separation_;
  int motor1_id_;
  int motor2_id_;
  int max_motor_rpm_;

public:
  IndividualMotorNode() : Node("individual_motor_node") {
    RCLCPP_INFO(this->get_logger(), "個別モーター制御ノードを開始中...");

    // パラメータの宣言と取得
    initializeParameters();

    // DDTモータライブラリの初期化
    if (!initializeMotorLib()) {
      RCLCPP_ERROR(this->get_logger(), "モーターライブラリの初期化に失敗しました");
      return;
    }

    // ROS 2 通信の設定
    setupCommunication();

    RCLCPP_INFO(this->get_logger(), "個別モーター制御ノード初期化完了");
  }

  ~IndividualMotorNode() {
    if (motor_lib_) {
      RCLCPP_INFO(this->get_logger(), "モータシステムを停止中...");
      motor_lib_->emergencyStop();
      motor_lib_->shutdown();
    }
  }

private:
  void initializeParameters() {
    // パラメータを宣言
    this->declare_parameter("serial_port", "/dev/ttyACM0");
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("wheel_radius", 0.1);
    this->declare_parameter("wheel_separation", 0.5);
    this->declare_parameter("motor1_id", 1);
    this->declare_parameter("motor2_id", 2);
    this->declare_parameter("max_motor_rpm", 1000);

    // パラメータを取得
    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();
    motor1_id_ = this->get_parameter("motor1_id").as_int();
    motor2_id_ = this->get_parameter("motor2_id").as_int();
    max_motor_rpm_ = this->get_parameter("max_motor_rpm").as_int();

    RCLCPP_INFO(this->get_logger(), "パラメータ初期化完了:");
    RCLCPP_INFO(this->get_logger(), "  モーター1 ID: %d", motor1_id_);
    RCLCPP_INFO(this->get_logger(), "  モーター2 ID: %d", motor2_id_);
    RCLCPP_INFO(this->get_logger(), "  シリアルポート: %s", serial_port_.c_str());
    RCLCPP_INFO(this->get_logger(), "  最大RPM: %d", max_motor_rpm_);
  }

  bool initializeMotorLib() {
    try {
      // DDTモータライブラリのインスタンスを作成
      motor_lib_ = std::make_unique<motor_control_lib::DdtMotorLib>(serial_port_, baud_rate_);

      // 最大RPMを設定
      if (!motor_lib_->setMaxRpm(max_motor_rpm_)) {
        RCLCPP_ERROR(this->get_logger(), "最大RPM設定に失敗しました");
        return false;
      }

      // モータライブラリを初期化
      if (!motor_lib_->initialize()) {
        RCLCPP_ERROR(this->get_logger(), "モーターライブラリ初期化に失敗しました");
        return false;
      }

      // 個別モーター初期化
      if (!motor_lib_->initializeMotor(motor1_id_)) {
        RCLCPP_ERROR(this->get_logger(), "モーター1の初期化に失敗しました");
        return false;
      }

      if (!motor_lib_->initializeMotor(motor2_id_)) {
        RCLCPP_ERROR(this->get_logger(), "モーター2の初期化に失敗しました");
        return false;
      }

      RCLCPP_INFO(this->get_logger(), "すべてのモーターが初期化されました");
      return true;

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "モーター初期化中に例外が発生: %s", e.what());
      return false;
    }
  }

  void setupCommunication() {
    // サブスクライバー設定
    motor1_vel_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "motor1_velocity", 10,
        std::bind(&IndividualMotorNode::motor1VelCallback, this, std::placeholders::_1));

    motor2_vel_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "motor2_velocity", 10,
        std::bind(&IndividualMotorNode::motor2VelCallback, this, std::placeholders::_1));

    // パブリッシャー設定
    status_pub_ = this->create_publisher<std_msgs::msg::String>("individual_motor_status", 10);

    // ステータスタイマー
    status_timer_ =
        this->create_wall_timer(500ms, std::bind(&IndividualMotorNode::statusCallback, this));
  }

  void motor1VelCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    if (!motor_lib_) {
      RCLCPP_WARN(this->get_logger(), "モーターライブラリが未初期化です");
      return;
    }

    int velocity_rpm = std::clamp(msg->data, -motor_lib_->getMaxRpm(), motor_lib_->getMaxRpm());

    if (motor_lib_->setMotorVelocity(motor1_id_, velocity_rpm)) {
      RCLCPP_INFO(this->get_logger(), "モーター1速度設定: %d RPM", velocity_rpm);
    } else {
      RCLCPP_ERROR(this->get_logger(), "モーター1速度設定に失敗: %d RPM", velocity_rpm);
    }
  }

  void motor2VelCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    if (!motor_lib_) {
      RCLCPP_WARN(this->get_logger(), "モーターライブラリが未初期化です");
      return;
    }

    int velocity_rpm = std::clamp(msg->data, -motor_lib_->getMaxRpm(), motor_lib_->getMaxRpm());

    if (motor_lib_->setMotorVelocity(motor2_id_, velocity_rpm)) {
      RCLCPP_INFO(this->get_logger(), "モーター2速度設定: %d RPM", velocity_rpm);
    } else {
      RCLCPP_ERROR(this->get_logger(), "モーター2速度設定に失敗: %d RPM", velocity_rpm);
    }
  }

  void statusCallback() {
    if (!motor_lib_) {
      return;
    }

    try {
      // 個別モーターステータスを取得
      int motor1_rpm, motor2_rpm;
      uint8_t motor1_temp, motor2_temp;
      uint8_t motor1_fault, motor2_fault;

      bool motor1_ok =
          motor_lib_->getMotorStatus(motor1_id_, motor1_rpm, motor1_temp, motor1_fault);
      bool motor2_ok =
          motor_lib_->getMotorStatus(motor2_id_, motor2_rpm, motor2_temp, motor2_fault);

      // ステータス情報を作成
      std::string status_str =
          "{"
          "\"motor1_id\":" +
          std::to_string(motor1_id_) +
          ","
          "\"motor1_rpm\":" +
          std::to_string(motor1_rpm) +
          ","
          "\"motor1_temperature\":" +
          std::to_string(motor1_temp) +
          ","
          "\"motor1_fault_code\":" +
          std::to_string(motor1_fault) +
          ","
          "\"motor1_status\":\"" +
          (motor1_ok ? "OK" : "ERROR") +
          "\","
          "\"motor2_id\":" +
          std::to_string(motor2_id_) +
          ","
          "\"motor2_rpm\":" +
          std::to_string(motor2_rpm) +
          ","
          "\"motor2_temperature\":" +
          std::to_string(motor2_temp) +
          ","
          "\"motor2_fault_code\":" +
          std::to_string(motor2_fault) +
          ","
          "\"motor2_status\":\"" +
          (motor2_ok ? "OK" : "ERROR") +
          "\","
          "\"system_healthy\":" +
          (motor_lib_->isHealthy() ? "true" : "false") + "}";

      auto status_msg = std_msgs::msg::String();
      status_msg.data = status_str;
      status_pub_->publish(status_msg);

      RCLCPP_DEBUG(this->get_logger(), "モーター状態 - M%d: %dRPM, M%d: %dRPM", motor1_id_,
                   motor1_rpm, motor2_id_, motor2_rpm);

    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "ステータスコールバックで例外発生: %s", e.what());
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<IndividualMotorNode>();

  RCLCPP_INFO(node->get_logger(), "個別モーター制御アプリケーション開始");

  // テスト用のデモシーケンス（オプション）
  std::thread demo_thread([node]() {
    std::this_thread::sleep_for(3s);

    RCLCPP_INFO(node->get_logger(), "デモシーケンス開始...");

    // モーター1を100RPMで3秒間回転
    auto msg1 = std_msgs::msg::Int32();
    msg1.data = 100;
    // NOTE: 実際の使用では外部からパブリッシュします
    RCLCPP_INFO(node->get_logger(), "デモ: モーター1を100RPMで動作");
    std::this_thread::sleep_for(3s);

    // モーター2を-100RPMで3秒間回転
    auto msg2 = std_msgs::msg::Int32();
    msg2.data = -100;
    RCLCPP_INFO(node->get_logger(), "デモ: モーター2を-100RPMで動作");
    std::this_thread::sleep_for(3s);

    // 両方を停止
    msg1.data = 0;
    msg2.data = 0;
    RCLCPP_INFO(node->get_logger(), "デモ: 全モーター停止");
  });

  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "実行中に例外発生: %s", e.what());
  }

  if (demo_thread.joinable()) {
    demo_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}

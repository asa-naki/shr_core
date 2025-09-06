#include "motor_control_app/shot_component.hpp"

#include <chrono>
#include <thread>

namespace motor_control_app {

ShotComponent::ShotComponent(const rclcpp::NodeOptions& options)
    : Node("shot_component", options), is_shooting_(false), last_button_state_(false) {
  // パラメーター宣言
  this->declare_parameter("port", "/dev/ttyUSB0");
  this->declare_parameter("baudrate", 115200);
  this->declare_parameter("pan_servo_id", 1);
  this->declare_parameter("trigger_servo_id", 3);
  this->declare_parameter("fire_button", 0);         // 射撃ボタン（Aボタンなど）
  this->declare_parameter("fire_position", 1500);    // 射撃位置
  this->declare_parameter("home_position", 2048);    // ホーム位置
  this->declare_parameter("fire_duration_ms", 300);  // 射撃持続時間（ミリ秒）

  // パラメーター取得
  std::string port = this->get_parameter("port").as_string();
  int baudrate = this->get_parameter("baudrate").as_int();
  pan_servo_id_ = this->get_parameter("pan_servo_id").as_int();
  trigger_servo_id_ = this->get_parameter("trigger_servo_id").as_int();
  fire_button_ = this->get_parameter("fire_button").as_int();
  fire_position_ = this->get_parameter("fire_position").as_int();
  home_position_ = this->get_parameter("home_position").as_int();
  fire_duration_ms_ = this->get_parameter("fire_duration_ms").as_int();

  // サーボコントローラー初期化
  servo_controller_ = std::make_shared<motor_control_lib::FeetechServoController>(port, baudrate);
  if (!servo_controller_->connect()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to servo controller");
    return;
  }

  // 射撃コントローラー初期化
  shot_controller_ = std::make_unique<motor_control_lib::ShotController>(servo_controller_);

  // joyサブスクライバー作成
  joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 1, std::bind(&ShotComponent::joyCallback, this, std::placeholders::_1));

  // 既存のサブスクライバー（互換性のため）
  aim_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
      "aim_target", 1, std::bind(&ShotComponent::aimCallback, this, std::placeholders::_1));

  fire_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "fire_trigger", 1, std::bind(&ShotComponent::fireCallback, this, std::placeholders::_1));

  home_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "return_home", 1, std::bind(&ShotComponent::homeCallback, this, std::placeholders::_1));

  // パブリッシャー作成
  current_aim_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("current_aim", 1);

  // 現在位置を定期的に公開（一時的に無効化）
  // timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
  //                                  std::bind(&ShotComponent::publishCurrentAim, this));

  // 最初にホーム位置に移動
  std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 初期化待機
  if (!servo_controller_->setPosition(trigger_servo_id_, home_position_, false)) {
    RCLCPP_WARN(this->get_logger(), "Failed to move to initial home position");
  }

  RCLCPP_INFO(this->get_logger(), "Shot component started");
  RCLCPP_INFO(this->get_logger(), "Fire button: %d, Fire position: %d, Home position: %d",
              fire_button_, fire_position_, home_position_);
}

ShotComponent::~ShotComponent() {
  if (servo_controller_) {
    servo_controller_->disconnect();
  }
}

void ShotComponent::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  if (!msg || msg->buttons.empty() || is_shooting_) {
    return;
  }

  // 指定されたボタンの状態をチェック
  if (fire_button_ >= 0 && fire_button_ < static_cast<int>(msg->buttons.size())) {
    bool current_button_state = msg->buttons[fire_button_] == 1;

    // ボタンが押された瞬間を検出（立ち上がりエッジ）
    if (current_button_state && !last_button_state_) {
      executeShotSequence();
    }

    last_button_state_ = current_button_state;
  }
}

void ShotComponent::executeShotSequence() {
  if (is_shooting_) {
    return;  // 既に射撃中の場合は無視
  }

  is_shooting_ = true;
  RCLCPP_INFO(this->get_logger(), "Starting shot sequence...");

  // 1. 射撃位置に移動
  if (servo_controller_->setPosition(trigger_servo_id_, fire_position_, false)) {
    RCLCPP_INFO(this->get_logger(), "Moved to fire position (%d)", fire_position_);

    // 射撃持続時間待機
    std::this_thread::sleep_for(std::chrono::milliseconds(fire_duration_ms_));

    // 2. すべてのサーボをホーム位置に戻る
    if (servo_controller_->setPosition(trigger_servo_id_, home_position_, false)) {
      RCLCPP_INFO(this->get_logger(), "Returned to home position (%d)", home_position_);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to return to home position");
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to move to fire position");
  }

  is_shooting_ = false;
  RCLCPP_INFO(this->get_logger(), "Shot sequence completed");
}

void ShotComponent::aimCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
  // Point.x = pan位置 (0-4095)
  uint16_t pan_position = static_cast<uint16_t>(std::max(0.0, std::min(4095.0, msg->x)));

  if (servo_controller_->setPosition(pan_servo_id_, pan_position, false)) {
    RCLCPP_INFO(this->get_logger(), "Aiming at pan=%d", pan_position);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to aim at target position");
  }
}

void ShotComponent::fireCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data) {
    executeShotSequence();
  }
}

void ShotComponent::homeCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data) {
    // ホーム位置に戻る
    if (servo_controller_->setPosition(trigger_servo_id_, home_position_, false)) {
      RCLCPP_INFO(this->get_logger(), "Returned to home position");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to return home");
    }
  }
}

void ShotComponent::publishCurrentAim() {
  int32_t pan_position = servo_controller_->getCurrentPosition(pan_servo_id_);
  if (pan_position != -1) {
    auto msg = std::make_unique<geometry_msgs::msg::Point>();
    msg->x = static_cast<double>(pan_position);
    msg->y = 0.0;  // tilt機構なし
    msg->z = 0.0;
    current_aim_publisher_->publish(std::move(msg));
  }
}

}  // namespace motor_control_app

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(motor_control_app::ShotComponent)

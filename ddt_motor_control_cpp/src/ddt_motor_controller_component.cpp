#include "ddt_motor_control_cpp/ddt_motor_controller_component.hpp"

#include <chrono>
#include <rclcpp_components/register_node_macro.hpp>
#include <thread>

using namespace std::chrono_literals;

namespace ddt_motor_control_cpp {

DdtMotorControllerComponent::DdtMotorControllerComponent(const rclcpp::NodeOptions& options)
    : Node("ddt_motor_controller", options),
      left_motor_id_(1),
      right_motor_id_(2),
      max_motor_rpm_(330),     // M15データシートより：-330 ～ 330 rpm
      wheel_radius_(0.1),      // 車輪の半径 (m)
      wheel_separation_(0.5),  // 左右車輪間の距離 (m)
      serial_fd_(-1),
      serial_port_("/dev/ttyACM0"),
      baud_rate_(115200),  // M15データシートより：115200 bps
      left_motor_velocity_(0),
      right_motor_velocity_(0),
      left_motor_rotations_(0.0),
      right_motor_rotations_(0.0) {
  // パラメータの宣言と取得
  this->declare_parameter("serial_port", serial_port_);
  this->declare_parameter("baud_rate", baud_rate_);
  this->declare_parameter("wheel_radius", wheel_radius_);
  this->declare_parameter("wheel_separation", wheel_separation_);
  this->declare_parameter("left_motor_id", left_motor_id_);
  this->declare_parameter("right_motor_id", right_motor_id_);
  this->declare_parameter("max_motor_rpm", max_motor_rpm_);

  serial_port_ = this->get_parameter("serial_port").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  wheel_radius_ = this->get_parameter("wheel_radius").as_double();
  wheel_separation_ = this->get_parameter("wheel_separation").as_double();
  left_motor_id_ = this->get_parameter("left_motor_id").as_int();
  right_motor_id_ = this->get_parameter("right_motor_id").as_int();
  max_motor_rpm_ = this->get_parameter("max_motor_rpm").as_int();

  // シリアル通信の初期化
  initializeSerial();

  // 両方のモーターを速度モードに設定
  setModeVelocity(left_motor_id_);
  setModeVelocity(right_motor_id_);
  std::this_thread::sleep_for(200ms);

  // Twistメッセージのサブスクライバーを作成
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1,
      std::bind(&DdtMotorControllerComponent::twistCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Twistメッセージのサブスクライバーを作成しました");

  // MotorStatusメッセージのパブリッシャーを作成
  motor_status_pub_ =
      this->create_publisher<ddt_motor_control::msg::MotorStatus>("motor_status", 10);
  RCLCPP_INFO(this->get_logger(), "MotorStatusパブリッシャーを作成しました");

  // タイマーの初期化
  watchdog_timer_ = this->create_wall_timer(
      500ms, std::bind(&DdtMotorControllerComponent::watchdogCallback, this));

  status_timer_ =
      this->create_wall_timer(1s, std::bind(&DdtMotorControllerComponent::logStatus, this));

  status_publish_timer_ = this->create_wall_timer(
      100ms, std::bind(&DdtMotorControllerComponent::publishMotorStatus, this));

  // M15モーターフィードバック取得タイマー（20ms =
  // 50Hz、データシートの最大500Hzより余裕を持った設定）
  feedback_timer_ = this->create_wall_timer(
      20ms, std::bind(&DdtMotorControllerComponent::feedbackCallback, this));

  // 時刻の初期化
  last_twist_time_ = std::chrono::steady_clock::now();
  last_status_time_ = std::chrono::steady_clock::now();

  // フィードバック構造体の初期化
  left_motor_feedback_ = {};
  right_motor_feedback_ = {};
}

DdtMotorControllerComponent::~DdtMotorControllerComponent() {
  stopMotors();
  closeSerial();
}

void DdtMotorControllerComponent::initializeSerial() {
  try {
    // シリアルポートを開く
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
      throw std::runtime_error("シリアルポートが開けませんでした: " + serial_port_);
    }

    // シリアルポートの設定
    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
      throw std::runtime_error("tcgetattr エラー");
    }

    // ボーレート設定
    speed_t speed = B115200;
    switch (baud_rate_) {
      case 9600:
        speed = B9600;
        break;
      case 19200:
        speed = B19200;
        break;
      case 38400:
        speed = B38400;
        break;
      case 57600:
        speed = B57600;
        break;
      case 115200:
        speed = B115200;
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "未対応のボーレート %d、115200を使用", baud_rate_);
        speed = B115200;
        break;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8N1設定
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                      // disable break processing
    tty.c_lflag = 0;                             // no signaling chars, no echo,
    tty.c_oflag = 0;                             // no remapping, no delays
    tty.c_cc[VMIN] = 0;                          // read doesn't block
    tty.c_cc[VTIME] = 5;                         // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);         // ignore modem controls,
    tty.c_cflag &= ~(PARENB | PARODD);       // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      throw std::runtime_error("tcsetattr エラー");
    }

    RCLCPP_INFO(this->get_logger(), "シリアルポートが開きました: %s", serial_port_.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "シリアルポートの初期化に失敗しました: %s", e.what());
    throw;
  }
}

void DdtMotorControllerComponent::closeSerial() {
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

ssize_t DdtMotorControllerComponent::writeSerial(const void* data, size_t size) {
  if (serial_fd_ < 0) {
    return -1;
  }
  return write(serial_fd_, data, size);
}

ssize_t DdtMotorControllerComponent::readSerial(void* data, size_t size) {
  if (serial_fd_ < 0) {
    return -1;
  }
  return read(serial_fd_, data, size);
}

uint8_t DdtMotorControllerComponent::crc8Maxim(const std::vector<uint8_t>& data) {
  // M15データシート準拠：CRC8（Maximポリノミアル x⁸ + x⁵ + x⁴ + 1）
  uint8_t crc = 0x00;
  for (uint8_t byte : data) {
    crc ^= byte;
    for (int i = 0; i < 8; i++) {
      if (crc & 0x01) {
        crc = (crc >> 1) ^ 0x8C;  // 0x8C = x⁵ + x⁴ + 1を右シフト形式で表現
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

bool DdtMotorControllerComponent::sendCommand(const std::vector<uint8_t>& command,
                                              int retry_count) {
  for (int attempt = 0; attempt < retry_count; attempt++) {
    try {
      ssize_t written = writeSerial(command.data(), command.size());
      if (written == static_cast<ssize_t>(command.size())) {
        fsync(serial_fd_);  // バッファをフラッシュ
        std::this_thread::sleep_for(50ms);
        return true;
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "シリアル通信エラー (試行 %d): %s", attempt + 1, e.what());
      std::this_thread::sleep_for(100ms);
    }
  }
  return false;
}

void DdtMotorControllerComponent::setModeVelocity(int motor_id) {
  std::vector<uint8_t> data_fields = {static_cast<uint8_t>(motor_id), 0xA0, 0, 0, 0, 0, 0, 0, 0};

  uint8_t crc = crc8Maxim(data_fields);
  data_fields.push_back(crc);

  bool success = sendCommand(data_fields);
  if (success) {
    RCLCPP_INFO(this->get_logger(), "モーター %d を速度制御モードに設定しました", motor_id);
  } else {
    RCLCPP_ERROR(this->get_logger(), "モーター %d の速度制御モード設定に失敗しました", motor_id);
  }
}

void DdtMotorControllerComponent::setMotorVelocity(int motor_id, int velocity_rpm) {
  // 速度を制限
  int velocity_int = std::clamp(velocity_rpm, -max_motor_rpm_, max_motor_rpm_);

  // 速度バイトを作成（リトルエンディアン）
  uint8_t vel_low = static_cast<uint8_t>(velocity_int & 0xFF);
  uint8_t vel_high = static_cast<uint8_t>((velocity_int >> 8) & 0xFF);

  std::vector<uint8_t> data_fields = {
      static_cast<uint8_t>(motor_id), 0x64, vel_low, vel_high, 0, 0, 0, 0, 0};

  uint8_t crc = crc8Maxim(data_fields);
  data_fields.push_back(crc);

  bool success = sendCommand(data_fields);
  if (success) {
    // モーターIDに応じて現在速度を更新（送信成功時のみ）
    if (motor_id == left_motor_id_) {
      left_motor_velocity_ = velocity_int;
    } else if (motor_id == right_motor_id_) {
      right_motor_velocity_ = velocity_int;
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "モーター %d の速度設定に失敗しました (目標: %d RPM)",
                 motor_id, velocity_int);
  }
}

std::pair<double, double> DdtMotorControllerComponent::twistToMotorVelocities(double linear_x,
                                                                              double angular_z) {
  // 差動駆動の運動学方程式を使用
  double v_left = linear_x - (angular_z * wheel_separation_ / 2.0);
  double v_right = linear_x + (angular_z * wheel_separation_ / 2.0);

  // 並進速度からRPMに変換
  double rpm_left = (v_left / (2.0 * M_PI * wheel_radius_)) * 60.0;
  double rpm_right = -1 * (v_right / (2.0 * M_PI * wheel_radius_)) * 60.0;  // 右モーターは逆方向

  return std::make_pair(rpm_left, rpm_right);
}

void DdtMotorControllerComponent::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  double linear_x = msg->linear.x;
  double angular_z = msg->angular.z;

  auto [left_rpm, right_rpm] = twistToMotorVelocities(linear_x, angular_z);

  RCLCPP_DEBUG(this->get_logger(),
               "Twist受信 - 線形: %.3f m/s, 角速度: %.3f rad/s -> 左: %.1f RPM, 右: %.1f RPM",
               linear_x, angular_z, left_rpm, right_rpm);

  // 左右のモーターを同期して制御
  setMotorVelocity(left_motor_id_, static_cast<int>(left_rpm));
  std::this_thread::sleep_for(10ms);
  setMotorVelocity(right_motor_id_, static_cast<int>(right_rpm));

  // ウォッチドッグタイマーのリセット
  last_twist_time_ = std::chrono::steady_clock::now();
}

void DdtMotorControllerComponent::updateMotorRotations() {
  auto current_time = std::chrono::steady_clock::now();
  auto dt = std::chrono::duration<double>(current_time - last_status_time_).count();

  // RPMから1秒間の回転数を計算し、積算
  double left_rotations_per_sec = left_motor_velocity_ / 60.0;
  double right_rotations_per_sec = right_motor_velocity_ / 60.0;

  left_motor_rotations_ += left_rotations_per_sec * dt;
  right_motor_rotations_ += right_rotations_per_sec * dt;

  last_status_time_ = current_time;
}

void DdtMotorControllerComponent::publishMotorStatus() {
  updateMotorRotations();

  auto msg = ddt_motor_control::msg::MotorStatus();
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "base_link";

  msg.left_motor_rpm = left_motor_velocity_;
  msg.right_motor_rpm = right_motor_velocity_;
  msg.left_motor_rotations = left_motor_rotations_;
  msg.right_motor_rotations = right_motor_rotations_;

  motor_status_pub_->publish(msg);
}

void DdtMotorControllerComponent::stopMotors() {
  RCLCPP_INFO(this->get_logger(), "モーターを停止しています...");
  setMotorVelocity(left_motor_id_, 0);
  std::this_thread::sleep_for(10ms);
  setMotorVelocity(right_motor_id_, 0);
}

void DdtMotorControllerComponent::watchdogCallback() {
  auto current_time = std::chrono::steady_clock::now();
  auto time_diff = std::chrono::duration<double>(current_time - last_twist_time_).count();

  if (time_diff > 0.5) {
    stopMotors();
    RCLCPP_WARN(this->get_logger(), "Twistメッセージのタイムアウト: モーターを停止しました");
  }
}

bool DdtMotorControllerComponent::requestMotorFeedback(int motor_id) {
  // M15データシート準拠：フィードバック取得コマンド
  // ID、0x74、その他は0、CRC8
  std::vector<uint8_t> data_fields = {static_cast<uint8_t>(motor_id), 0x74, 0, 0, 0, 0, 0, 0, 0};

  uint8_t crc = crc8Maxim(data_fields);
  data_fields.push_back(crc);

  bool success = sendCommand(data_fields);
  if (!success) {
    RCLCPP_WARN(this->get_logger(), "モーター %d のフィードバック要求に失敗しました", motor_id);
    return false;
  }

  // 応答を読み取り（10バイト）
  std::vector<uint8_t> response(10);
  std::this_thread::sleep_for(5ms);  // 応答待ち

  ssize_t bytes_read = readSerial(response.data(), response.size());
  if (bytes_read == 10) {
    processFeedbackResponse(motor_id, response);
    return true;
  } else {
    RCLCPP_DEBUG(this->get_logger(), "モーター %d からの応答が不完全です (%ld bytes)", motor_id,
                 bytes_read);
    return false;
  }
}

void DdtMotorControllerComponent::processFeedbackResponse(int motor_id,
                                                          const std::vector<uint8_t>& response) {
  // CRC8チェック
  std::vector<uint8_t> data_for_crc(response.begin(), response.begin() + 9);
  uint8_t calculated_crc = crc8Maxim(data_for_crc);

  if (calculated_crc != response[9]) {
    RCLCPP_WARN(this->get_logger(), "モーター %d のCRCエラー", motor_id);
    return;
  }

  // フィードバックデータを解析
  MotorFeedback* feedback =
      (motor_id == left_motor_id_) ? &left_motor_feedback_ : &right_motor_feedback_;

  feedback->mode = response[1];
  feedback->current = (static_cast<uint16_t>(response[2]) << 8) | response[3];
  feedback->speed = static_cast<int16_t>((static_cast<uint16_t>(response[4]) << 8) | response[5]);
  feedback->temperature = response[6];
  feedback->angle = response[7];
  feedback->fault_code = response[8];

  // ログ出力
  // RCLCPP_INFO(this->get_logger(),
  //             "モーター %d フィードバック - モード: %d, 電流: %d, 速度: %d RPM, "
  //             "温度: %d℃, 角度: %d, 故障コード: 0x%02X",
  //             motor_id, feedback->mode, feedback->current, feedback->speed,
  //             feedback->temperature, feedback->angle, feedback->fault_code);

  // 故障コードチェック
  if (feedback->fault_code != 0) {
    RCLCPP_ERROR(this->get_logger(), "モーター %d で故障検出: 0x%02X", motor_id,
                 feedback->fault_code);
  }
}

void DdtMotorControllerComponent::feedbackCallback() {
  // 交互にモーターのフィードバックを取得（通信負荷分散）
  static bool request_left = true;

  if (request_left) {
    requestMotorFeedback(left_motor_id_);
  } else {
    requestMotorFeedback(right_motor_id_);
  }

  request_left = !request_left;

  // モーターの健康状態をチェック
  checkMotorHealth();
}

void DdtMotorControllerComponent::checkMotorHealth() {
  // 左モーターの健康状態チェック
  if (left_motor_feedback_.fault_code != 0) {
    if (left_motor_feedback_.fault_code & 0x01) {
      RCLCPP_ERROR(this->get_logger(), "左モーター: 位相過電流エラー");
    }
    if (left_motor_feedback_.fault_code & 0x02) {
      RCLCPP_ERROR(this->get_logger(), "左モーター: バス過電流エラー");
    }
    if (left_motor_feedback_.fault_code & 0x04) {
      RCLCPP_ERROR(this->get_logger(), "左モーター: センサー故障");
    }
    if (left_motor_feedback_.fault_code & 0x08) {
      RCLCPP_ERROR(this->get_logger(), "左モーター: スタリング（回転停止）");
    }
  }

  // 右モーターの健康状態チェック
  if (right_motor_feedback_.fault_code != 0) {
    if (right_motor_feedback_.fault_code & 0x01) {
      RCLCPP_ERROR(this->get_logger(), "右モーター: 位相過電流エラー");
    }
    if (right_motor_feedback_.fault_code & 0x02) {
      RCLCPP_ERROR(this->get_logger(), "右モーター: バス過電流エラー");
    }
    if (right_motor_feedback_.fault_code & 0x04) {
      RCLCPP_ERROR(this->get_logger(), "右モーター: センサー故障");
    }
    if (right_motor_feedback_.fault_code & 0x08) {
      RCLCPP_ERROR(this->get_logger(), "右モーター: スタリング（回転停止）");
    }
  }
}

void DdtMotorControllerComponent::logStatus() {
  RCLCPP_INFO(this->get_logger(),
              "モーター状態 - 左: %d RPM (%.2f 回転, %d℃), 右: %d RPM (%.2f 回転, %d℃)",
              left_motor_velocity_, left_motor_rotations_, left_motor_feedback_.temperature,
              right_motor_velocity_, right_motor_rotations_, right_motor_feedback_.temperature);
}

}  // namespace ddt_motor_control_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(ddt_motor_control_cpp::DdtMotorControllerComponent)

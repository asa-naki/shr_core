#ifndef MOTOR_CONTROL_LIB_SIMPLE__MOTOR_FACTORY_HPP_
#define MOTOR_CONTROL_LIB_SIMPLE__MOTOR_FACTORY_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>

namespace motor_control_lib_simple {

/**
 * @brief モータ設定構造体
 */
struct MotorConfig {
  std::string type;                               // "ddt", "esc", "servo"
  std::string name;                               // ユニークな名前
  std::string mode;                               // "drive", "shot", または "both"
  std::map<std::string, std::string> parameters;  // タイプ別設定
};

/**
 * @brief 基本モータコントローラインターフェース
 */
class BaseMotorController {
public:
  explicit BaseMotorController(const std::string& type) : type_(type), initialized_(false) {}
  virtual ~BaseMotorController() = default;

  virtual bool initialize() = 0;
  virtual void shutdown() = 0;
  virtual bool isHealthy() const = 0;
  virtual void emergencyStop() = 0;

  const std::string& getType() const { return type_; }
  bool isInitialized() const { return initialized_; }

protected:
  std::string type_;
  bool initialized_;
};

/**
 * @brief ドライブモータインターフェース
 */
class IDriveMotor {
public:
  virtual ~IDriveMotor() = default;
  virtual bool setVelocity(double linear_x, double angular_z) = 0;
  virtual void stop() = 0;
  virtual bool isMoving() const = 0;
};

/**
 * @brief シュートモータインターフェース
 */
class IShotMotor {
public:
  virtual ~IShotMotor() = default;
  virtual bool setShotSpeed(double speed) = 0;
  virtual bool setShotAngle(double angle) = 0;
  virtual bool executeFire() = 0;
  virtual bool isReadyToFire() const = 0;
  virtual void stopFiring() = 0;
};

/**
 * @brief モータファクトリクラス
 */
class MotorFactory {
public:
  // モータ作成関数のタイプ定義
  using MotorCreator = std::function<std::shared_ptr<BaseMotorController>(const MotorConfig&)>;

  // モータ作成
  static std::shared_ptr<BaseMotorController> createMotor(const MotorConfig& config);

  // インターフェース取得
  static std::shared_ptr<IDriveMotor> getDriveInterface(std::shared_ptr<BaseMotorController> motor);
  static std::shared_ptr<IShotMotor> getShotInterface(std::shared_ptr<BaseMotorController> motor);

  // 設定作成ヘルパー
  static MotorConfig createConfig(const std::string& type, const std::string& name,
                                  const std::string& mode);

  // サポートされているタイプの取得
  static std::vector<std::string> getSupportedTypes();

private:
  static const std::map<std::string, MotorCreator> motor_creators_;

  // 各モータタイプの作成関数
  static std::shared_ptr<BaseMotorController> createDdtMotor(const MotorConfig& config);
  static std::shared_ptr<BaseMotorController> createEscMotor(const MotorConfig& config);
  static std::shared_ptr<BaseMotorController> createServoMotor(const MotorConfig& config);
};

/**
 * @brief 統合モータコントローラ
 */
class UnifiedMotorController {
public:
  UnifiedMotorController();
  ~UnifiedMotorController();

  // モータ管理
  bool addMotor(const MotorConfig& config);
  bool initialize();
  void shutdown();

  // 制御メソッド
  bool setVelocity(double linear_x, double angular_z, const std::string& motor_name = "");
  bool setShotParameters(double speed, double angle, const std::string& motor_name = "");
  bool executeFire(const std::string& motor_name = "");
  void stopAll();
  void emergencyStopAll();

  // 状態確認
  bool isHealthy() const;
  std::vector<std::string> getMotorNames() const;
  std::vector<std::string> getDriveMotorNames() const;
  std::vector<std::string> getShotMotorNames() const;

private:
  struct MotorInfo {
    std::string name;
    std::shared_ptr<BaseMotorController> controller;
    std::shared_ptr<IDriveMotor> drive_interface;
    std::shared_ptr<IShotMotor> shot_interface;
  };

  std::vector<MotorInfo> motors_;
  bool initialized_;
  mutable std::mutex motors_mutex_;

  // ヘルパーメソッド
  std::vector<MotorInfo>::iterator findMotor(const std::string& name);
  std::vector<MotorInfo>::const_iterator findMotor(const std::string& name) const;
};

/**
 * @brief 標準ロボット設定のヘルパークラス
 */
class QuickSetup {
public:
  // 標準的なロボット設定
  static std::shared_ptr<UnifiedMotorController> setupStandardRobot();

  // カスタムロボット設定例
  static std::shared_ptr<UnifiedMotorController> setupCustomRobot(
      const std::vector<MotorConfig>& configs);
};

}  // namespace motor_control_lib_simple

#endif  // MOTOR_CONTROL_LIB_SIMPLE__MOTOR_FACTORY_HPP_

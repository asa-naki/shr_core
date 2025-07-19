#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <chrono>
#include <thread>

#include "esc_motor_control/esc_motor_control_component.hpp"

using namespace std::chrono_literals;

class ESCMotorControlJoyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    // Create node options with test parameters
    rclcpp::NodeOptions options;
    options.append_parameter_override("pwm_pin", 18);
    options.append_parameter_override("max_speed", 1.0);
    options.append_parameter_override("min_speed", -1.0);
    options.append_parameter_override("pwm_frequency", 50);
    options.append_parameter_override("enable_safety_stop", false);     // Disable for testing
    options.append_parameter_override("test_mode", true);               // Enable test mode
    options.append_parameter_override("full_speed_button", 1);
    options.append_parameter_override("full_speed_value", 1.0);
    options.append_parameter_override("pwm_chip", 0);
    options.append_parameter_override("pwm_channel", 0);
    options.append_parameter_override("joy_topic", "test_joy");     // Use test topic

    // Create test node
    esc_node_ = std::make_shared<esc_motor_control::ESCMotorControlComponent>(options);

    // Create test publisher for joy messages
    joy_publisher_ = std::make_shared<rclcpp::Node>("joy_test_publisher");
    joy_pub_ = joy_publisher_->create_publisher<sensor_msgs::msg::Joy>("test_joy", 10);

    // Create feedback subscriber
    feedback_subscriber_ = std::make_shared<rclcpp::Node>("feedback_subscriber");
    feedback_sub_ = feedback_subscriber_->create_subscription<std_msgs::msg::Float32>(
      "motor_feedback", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg)
      {
        latest_feedback_ = msg->data;
        feedback_received_ = true;
      });

    // Wait for connections and ensure all publishers and subscribers are ready
    // BUT avoid spinning in a way that could trigger empty joy messages
    std::this_thread::sleep_for(100ms);     // Reduced wait time to minimize interfering messages
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  sensor_msgs::msg::Joy createJoyMessage()
  {
    sensor_msgs::msg::Joy joy_msg;
    joy_msg.header.stamp = rclcpp::Clock().now();

    // Initialize with 8 buttons and 8 axes (typical gamepad)
    joy_msg.buttons.resize(8, 0);
    joy_msg.axes.resize(8, 0.0);

    // Set default axis values (triggers not pressed)
    joy_msg.axes[2] = 1.0;     // Left trigger (not pressed)
    joy_msg.axes[5] = 1.0;     // Right trigger (not pressed)

    return joy_msg;
  }

  bool waitForFeedback(int timeout_ms = 1000)
  {
    feedback_received_ = false;
    auto start = std::chrono::steady_clock::now();
    auto timeout_duration = std::chrono::milliseconds(timeout_ms);

    while (!feedback_received_ && (std::chrono::steady_clock::now() - start < timeout_duration)) {
      rclcpp::spin_some(esc_node_);
      rclcpp::spin_some(joy_publisher_);
      rclcpp::spin_some(feedback_subscriber_);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return feedback_received_;
  }

  std::shared_ptr<esc_motor_control::ESCMotorControlComponent> esc_node_;
  std::shared_ptr<rclcpp::Node> joy_publisher_;
  std::shared_ptr<rclcpp::Node> feedback_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr feedback_sub_;

  float latest_feedback_ = 0.0;
  bool feedback_received_ = false;
};

TEST_F(ESCMotorControlJoyTest, TestEmergencyStopButton)
{
  // Verify initial state
  EXPECT_FALSE(esc_node_->isEmergencyStopActive()) <<
    "Initial state should not have emergency stop active";

  // Create joy message with emergency stop button pressed (button 0)
  auto joy_msg = createJoyMessage();
  joy_msg.buttons[0] = 1;   // Emergency stop button

  joy_pub_->publish(joy_msg);

  // Process the message
  std::this_thread::sleep_for(50ms);
  rclcpp::spin_some(esc_node_);
  rclcpp::spin_some(joy_publisher_);

  // Emergency stop should be active
  EXPECT_TRUE(esc_node_->isEmergencyStopActive()) <<
    "Emergency stop should be active after button press";
  EXPECT_FLOAT_EQ(esc_node_->getSimulatedPWMDuty(), 0.0);

  // Create a new message with emergency stop button released
  auto release_msg = createJoyMessage();
  release_msg.buttons[0] = 0;   // Release emergency stop button
  joy_pub_->publish(release_msg);

  // Process the release message
  std::this_thread::sleep_for(50ms);
  rclcpp::spin_some(esc_node_);
  rclcpp::spin_some(joy_publisher_);

  // Emergency stop should still be active (doesn't auto-clear)
  EXPECT_TRUE(esc_node_->isEmergencyStopActive()) <<
    "Emergency stop should persist after button release";

  // Manually clear for cleanup
  esc_node_->clearEmergencyStop();
  EXPECT_FALSE(esc_node_->isEmergencyStopActive()) << "Emergency stop should be cleared manually";
}

TEST_F(ESCMotorControlJoyTest, TestNodeInitialization)
{
  // Test that the node was created successfully
  EXPECT_NE(esc_node_, nullptr);
  EXPECT_NE(joy_publisher_, nullptr);
  EXPECT_NE(feedback_subscriber_, nullptr);
  EXPECT_TRUE(esc_node_->isTestMode());

  // Motor should be initially stopped
  EXPECT_FALSE(esc_node_->isSimulatedMotorRotating());
  EXPECT_FLOAT_EQ(esc_node_->getSimulatedPWMDuty(), 0.0);
}

TEST_F(ESCMotorControlJoyTest, TestMotorRotationSimulation)
{
  // Create joy message with full speed button pressed
  auto joy_msg = createJoyMessage();
  joy_msg.buttons[1] = 1;   // Full speed button

  // Publish the message once
  joy_pub_->publish(joy_msg);

  // Give time for message processing - just a short wait
  std::this_thread::sleep_for(50ms);

  // Single spin to process the message
  rclcpp::spin_some(esc_node_);
  rclcpp::spin_some(joy_publisher_);
  rclcpp::spin_some(feedback_subscriber_);

  // Check motor simulation state through ESC node
  EXPECT_TRUE(esc_node_->isSimulatedMotorRotating()) << "Motor should be rotating at full speed";
  EXPECT_FLOAT_EQ(esc_node_->getSimulatedPWMDuty(), 1.0) << "PWM duty should be 1.0";
  EXPECT_GT(
    std::abs(esc_node_->getSimulatedRotationSpeed()),
    0.0)
    << "Rotation speed should be non-zero";
  EXPECT_TRUE(esc_node_->isFullSpeedActive()) << "Full speed mode should be active";
}

TEST_F(ESCMotorControlJoyTest, TestMotorStopSimulation)
{
  // First start the motor
  auto joy_msg = createJoyMessage();
  joy_msg.buttons[1] = 1;   // Full speed button

  joy_pub_->publish(joy_msg);
  std::this_thread::sleep_for(50ms);
  rclcpp::spin_some(esc_node_);
  rclcpp::spin_some(joy_publisher_);

  EXPECT_TRUE(esc_node_->isSimulatedMotorRotating()) << "Motor should be rotating";
  EXPECT_TRUE(esc_node_->isFullSpeedActive()) << "Full speed should be active";

  // Now stop the motor by releasing the button
  joy_msg.buttons[1] = 0;   // Release button
  joy_pub_->publish(joy_msg);

  std::this_thread::sleep_for(50ms);
  rclcpp::spin_some(esc_node_);
  rclcpp::spin_some(joy_publisher_);

  EXPECT_FALSE(esc_node_->isSimulatedMotorRotating()) << "Motor should have stopped";
  EXPECT_FLOAT_EQ(esc_node_->getSimulatedPWMDuty(), 0.0) << "PWM duty should be 0.0";
  EXPECT_FLOAT_EQ(esc_node_->getSimulatedRotationSpeed(), 0.0) << "Rotation speed should be 0.0";
  EXPECT_FALSE(esc_node_->isFullSpeedActive()) << "Full speed mode should be inactive";
}

TEST_F(ESCMotorControlJoyTest, TestRightTriggerRotationSimulation)
{
  // Create joy message with right trigger pressed (axis 5)
  auto joy_msg = createJoyMessage();
  joy_msg.axes[5] = -1.0;   // Right trigger fully pressed

  joy_pub_->publish(joy_msg);
  std::this_thread::sleep_for(50ms);
  rclcpp::spin_some(esc_node_);
  rclcpp::spin_some(joy_publisher_);

  // Check motor simulation state
  EXPECT_TRUE(esc_node_->isSimulatedMotorRotating()) <<
    "Motor should be rotating with trigger input";
  EXPECT_FLOAT_EQ(
    esc_node_->getSimulatedPWMDuty(),
    1.0)
    << "PWM duty should be 1.0 for fully pressed trigger";
  EXPECT_GT(esc_node_->getSimulatedRotationSpeed(), 0.0) << "Rotation speed should be positive";
}

TEST_F(ESCMotorControlJoyTest, TestEmergencyStopStopsRotation)
{
  // First start the motor with full speed
  auto joy_msg = createJoyMessage();
  joy_msg.buttons[1] = 1;   // Full speed button

  joy_pub_->publish(joy_msg);
  std::this_thread::sleep_for(50ms);
  rclcpp::spin_some(esc_node_);
  rclcpp::spin_some(joy_publisher_);

  EXPECT_TRUE(esc_node_->isSimulatedMotorRotating()) <<
    "Motor should be rotating before emergency stop";
  EXPECT_TRUE(esc_node_->isFullSpeedActive()) << "Full speed should be active";

  // Now trigger emergency stop
  joy_msg.buttons[0] = 1;   // Emergency stop button
  joy_msg.buttons[1] = 0;   // Release full speed button
  joy_pub_->publish(joy_msg);

  std::this_thread::sleep_for(50ms);
  rclcpp::spin_some(esc_node_);
  rclcpp::spin_some(joy_publisher_);

  EXPECT_FALSE(esc_node_->isSimulatedMotorRotating()) <<
    "Motor should have stopped after emergency stop";
  EXPECT_FLOAT_EQ(
    esc_node_->getSimulatedPWMDuty(),
    0.0)
    << "PWM duty should be 0.0 after emergency stop";
  EXPECT_TRUE(esc_node_->isEmergencyStopActive()) << "Emergency stop should be active";
  EXPECT_FALSE(esc_node_->isFullSpeedActive()) << "Full speed mode should be inactive";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

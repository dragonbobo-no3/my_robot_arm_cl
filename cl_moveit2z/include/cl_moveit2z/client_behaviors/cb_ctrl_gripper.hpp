#pragma once

#include <chrono>
#include <memory>
#include <string>

#include <je_software/msg/end_effector_command.hpp>
#include <je_software/msg/end_effector_command_lr.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_moveit2z
{
class CbCtrlGripper : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbCtrlGripper() = default;

  CbCtrlGripper(
    int mode, double position, int preset = 0, bool leftValid = true, bool rightValid = false,
    std::string topic = "/end_effector_cmd_lr")
  : mode_(mode),
    position_(position),
    preset_(preset),
    left_valid_(leftValid),
    right_valid_(rightValid),
    topic_(std::move(topic))
  {
  }

  static CbCtrlGripper Position(
    double position, bool leftValid = true, bool rightValid = false,
    std::string topic = "/end_effector_cmd_lr")
  {
    return CbCtrlGripper(
      je_software::msg::EndEffectorCommand::MODE_POSITION, position, 0, leftValid, rightValid,
      std::move(topic));
  }

  static CbCtrlGripper Preset(
    int preset, bool leftValid = true, bool rightValid = false,
    std::string topic = "/end_effector_cmd_lr")
  {
    return CbCtrlGripper(
      je_software::msg::EndEffectorCommand::MODE_PRESET, 0.0, preset, leftValid, rightValid,
      std::move(topic));
  }

  void onEntry() override
  {
    if (!left_valid_ && !right_valid_)
    {
      RCLCPP_WARN(getLogger(), "[CbCtrlGripper] Both left_valid and right_valid are false.");
      this->postFailureEvent();
      return;
    }

    if (
      mode_ != je_software::msg::EndEffectorCommand::MODE_POSITION &&
      mode_ != je_software::msg::EndEffectorCommand::MODE_PRESET)
    {
      RCLCPP_WARN(getLogger(), "[CbCtrlGripper] Invalid gripper mode: %d", mode_);
      this->postFailureEvent();
      return;
    }

    if (!publisher_)
    {
      publisher_ =
        getNode()->create_publisher<je_software::msg::EndEffectorCommandLR>(topic_, rclcpp::QoS(10).reliable());
    }

    je_software::msg::EndEffectorCommandLR msg;
    msg.left_valid = left_valid_;
    msg.right_valid = right_valid_;

    auto fill_command = [&](je_software::msg::EndEffectorCommand & command)
    {
      command.mode = mode_;
      command.position = position_;
      command.preset = preset_;
    };

    if (left_valid_)
    {
      fill_command(msg.left);
    }
    if (right_valid_)
    {
      fill_command(msg.right);
    }

    publisher_->publish(msg);

    RCLCPP_INFO(
      getLogger(),
      "[CbCtrlGripper] Published gripper cmd topic=%s mode=%d left_valid=%d right_valid=%d "
      "position=%.4f preset=%d",
      topic_.c_str(), mode_, left_valid_, right_valid_, position_, preset_);

    this->postSuccessEvent();
  }

  void onExit() override {}

private:
  int mode_{je_software::msg::EndEffectorCommand::MODE_POSITION};
  double position_{0.0};
  int preset_{0};
  bool left_valid_{true};
  bool right_valid_{false};
  std::string topic_{"/end_effector_cmd_lr"};

  rclcpp::Publisher<je_software::msg::EndEffectorCommandLR>::SharedPtr publisher_;
};
}  // namespace cl_moveit2z

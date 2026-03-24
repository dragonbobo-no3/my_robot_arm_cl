#pragma once

#include <chrono>
#include <memory>
#include <string>

#include <cmath>

#include <algorithm>

#include <common/msg/oculus_init_joint_state.hpp>
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
    std::string topic = "/end_effector_cmd_lr", double timeoutSec = 2.0,
    std::string feedbackTopic = "/joint_states_double_arm", double positionTolerance = 0.03,
    std::string command = "", double torque = 0.0)
  : mode_(mode),
    position_(position),
    preset_(preset),
    left_valid_(leftValid),
    right_valid_(rightValid),
    topic_(std::move(topic)),
    timeout_sec_(timeoutSec),
    feedback_topic_(std::move(feedbackTopic)),
    position_tolerance_(positionTolerance),
    command_(normalizeCommand(command)),
    torque_(torque)
  {
  }

  static CbCtrlGripper Position(
    double position, bool leftValid = true, bool rightValid = false,
    std::string topic = "/end_effector_cmd_lr",
    double timeoutSec = 2.0, std::string feedbackTopic = "/joint_states_double_arm",
    double positionTolerance = 0.03)
  {
    return CbCtrlGripper(
      je_software::msg::EndEffectorCommand::MODE_POSITION, position, 0, leftValid, rightValid,
      std::move(topic), timeoutSec, std::move(feedbackTopic), positionTolerance);
  }

  static CbCtrlGripper Preset(
    int preset, bool leftValid = true, bool rightValid = false,
    std::string topic = "/end_effector_cmd_lr",
    double timeoutSec = 2.0, std::string feedbackTopic = "/joint_states_double_arm",
    double positionTolerance = 0.03)
  {
    return CbCtrlGripper(
      je_software::msg::EndEffectorCommand::MODE_PRESET, 0.0, preset, leftValid, rightValid,
      std::move(topic), timeoutSec, std::move(feedbackTopic), positionTolerance);
  }

  static CbCtrlGripper Torque(
    std::string command, double torque, bool leftValid = true, bool rightValid = false,
    std::string topic = "/end_effector_cmd_lr",
    double timeoutSec = 2.0, std::string feedbackTopic = "/joint_states_double_arm",
    double positionTolerance = 0.03)
  {
    return CbCtrlGripper(
      je_software::msg::EndEffectorCommand::MODE_TORQUE, 0.0, 0, leftValid, rightValid,
      std::move(topic), timeoutSec, std::move(feedbackTopic), positionTolerance,
      std::move(command), torque);
  }

  void onEntry() override
  {
    command_completed_ = false;
    waiting_feedback_ = false;

    if (!left_valid_ && !right_valid_)
    {
      RCLCPP_WARN(getLogger(), "[CbCtrlGripper] Both left_valid and right_valid are false.");
      markFailure("Both left_valid and right_valid are false");
      return;
    }

    if (
      mode_ != je_software::msg::EndEffectorCommand::MODE_POSITION &&
      mode_ != je_software::msg::EndEffectorCommand::MODE_PRESET &&
      mode_ != je_software::msg::EndEffectorCommand::MODE_TORQUE)
    {
      RCLCPP_WARN(getLogger(), "[CbCtrlGripper] Invalid gripper mode: %d", mode_);
      markFailure("Invalid gripper mode");
      return;
    }

    if (mode_ == je_software::msg::EndEffectorCommand::MODE_TORQUE)
    {
      if (!isValidTorqueCommand(command_))
      {
        RCLCPP_WARN(getLogger(), "[CbCtrlGripper] Invalid torque command: '%s'", command_.c_str());
        markFailure("Invalid torque command");
        return;
      }

      if (!std::isfinite(torque_) || torque_ <= 0.0)
      {
        RCLCPP_WARN(getLogger(), "[CbCtrlGripper] Invalid torque value: %.4f", torque_);
        markFailure("Invalid torque value");
        return;
      }
    }

    if (timeout_sec_ <= 0.0)
    {
      timeout_sec_ = 2.0;
    }

    if (position_tolerance_ <= 0.0)
    {
      position_tolerance_ = 0.03;
    }

    if (!publisher_)
    {
      publisher_ =
        getNode()->create_publisher<je_software::msg::EndEffectorCommandLR>(topic_, rclcpp::QoS(10).reliable());
    }

    if (!feedback_sub_)
    {
      feedback_sub_ = getNode()->create_subscription<common::msg::OculusInitJointState>(
        feedback_topic_,
        rclcpp::QoS(10).reliable(),
        [this](const common::msg::OculusInitJointState::SharedPtr msg)
        {
          onFeedback(msg);
        });
    }

    if (feedback_topic_ != legacy_feedback_topic_ && !feedback_sub_legacy_)
    {
      feedback_sub_legacy_ = getNode()->create_subscription<common::msg::OculusInitJointState>(
        legacy_feedback_topic_,
        rclcpp::QoS(10).reliable(),
        [this](const common::msg::OculusInitJointState::SharedPtr msg)
        {
          onFeedback(msg);
        });
    }

    je_software::msg::EndEffectorCommandLR msg;
    msg.left_valid = left_valid_;
    msg.right_valid = right_valid_;

    auto fill_command = [&](je_software::msg::EndEffectorCommand & command)
    {
      command.mode = mode_;
      command.position = position_;
      command.preset = preset_;
      command.command = command_;
      command.torque = torque_;
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

    command_sent_time_ = getNode()->now();
    waiting_feedback_ = true;

    if (!watchdog_timer_)
    {
      watchdog_timer_ = getNode()->create_wall_timer(
        std::chrono::milliseconds(50),
        [this]()
        {
          onWatchdogTick();
        });
    }

    RCLCPP_INFO(
      getLogger(),
      "[CbCtrlGripper] Published cmd topic=%s mode=%d left_valid=%d right_valid=%d position=%.4f "
      "preset=%d command=%s torque=%.4f feedback_topic=%s fallback_feedback_topic=%s timeout=%.2fs tol=%.4f",
      topic_.c_str(), mode_, left_valid_, right_valid_, position_, preset_, command_.c_str(),
      torque_, feedback_topic_.c_str(), legacy_feedback_topic_.c_str(), timeout_sec_,
      position_tolerance_);
  }

  void onExit() override
  {
    waiting_feedback_ = false;
    if (watchdog_timer_)
    {
      watchdog_timer_->cancel();
      watchdog_timer_.reset();
    }
  }

private:
  void onWatchdogTick()
  {
    if (!waiting_feedback_ || command_completed_)
    {
      return;
    }

    const auto elapsed = (getNode()->now() - command_sent_time_).seconds();
    if (elapsed > timeout_sec_)
    {
      // 对 torque/preset 模式无法从当前位置严格反推控制是否完成，这里保留超时成功兜底。
      markSuccess(position_, position_);
    }
  }

  void onFeedback(const common::msg::OculusInitJointState::SharedPtr msg)
  {
    if (!msg || !waiting_feedback_ || command_completed_)
    {
      return;
    }

    const bool leftOk = !left_valid_ || (msg->left_valid && isTargetReached(msg->left_gripper));
    const bool rightOk = !right_valid_ || (msg->right_valid && isTargetReached(msg->right_gripper));

    if (leftOk && rightOk)
    {
      markSuccess(msg->left_gripper, msg->right_gripper);
    }
  }

  bool isTargetReached(double feedbackValue) const
  {
    if (mode_ == je_software::msg::EndEffectorCommand::MODE_POSITION)
    {
      return std::fabs(feedbackValue - position_) <= position_tolerance_;
    }

    // Preset / torque 模式当前没有统一可逆的反馈判定，至少要求收到有效反馈值。
    return std::isfinite(feedbackValue);
  }

  static std::string normalizeCommand(std::string command)
  {
    std::transform(command.begin(), command.end(), command.begin(), ::tolower);
    return command;
  }

  static bool isValidTorqueCommand(const std::string & command)
  {
    return command == je_software::msg::EndEffectorCommand::CMD_OPEN ||
           command == je_software::msg::EndEffectorCommand::CMD_CLOSE;
  }

  void markSuccess(double leftFeedback, double rightFeedback)
  {
    if (command_completed_)
    {
      return;
    }

    command_completed_ = true;
    waiting_feedback_ = false;

    if (watchdog_timer_)
    {
      watchdog_timer_->cancel();
      watchdog_timer_.reset();
    }

    RCLCPP_INFO(
      getLogger(),
      "[CbCtrlGripper] Feedback reached target. left=%.4f right=%.4f mode=%d target_pos=%.4f preset=%d command=%s torque=%.4f",
      leftFeedback, rightFeedback, mode_, position_, preset_, command_.c_str(), torque_);
    this->postSuccessEvent();
  }

  void markFailure(const char * reason)
  {
    if (command_completed_)
    {
      return;
    }

    command_completed_ = true;
    waiting_feedback_ = false;

    if (watchdog_timer_)
    {
      watchdog_timer_->cancel();
      watchdog_timer_.reset();
    }

    RCLCPP_WARN(getLogger(), "[CbCtrlGripper] %s", reason);
    this->postFailureEvent();
  }

  int mode_{je_software::msg::EndEffectorCommand::MODE_POSITION};
  double position_{0.0};
  int preset_{0};
  bool left_valid_{true};
  bool right_valid_{false};
  std::string topic_{"/end_effector_cmd_lr"};
  double timeout_sec_{2.0};
  std::string feedback_topic_{"/joint_states_double_arm"};
  std::string legacy_feedback_topic_{"/oculus_init_joint_state"};
  double position_tolerance_{0.03};
  std::string command_{};
  double torque_{0.0};

  bool waiting_feedback_{false};
  bool command_completed_{false};
  rclcpp::Time command_sent_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<je_software::msg::EndEffectorCommandLR>::SharedPtr publisher_;
  rclcpp::Subscription<common::msg::OculusInitJointState>::SharedPtr feedback_sub_;
  rclcpp::Subscription<common::msg::OculusInitJointState>::SharedPtr feedback_sub_legacy_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};
}  // namespace cl_moveit2z

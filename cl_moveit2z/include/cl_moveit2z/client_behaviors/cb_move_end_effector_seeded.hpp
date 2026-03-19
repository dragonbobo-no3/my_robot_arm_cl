// Copyright 2021 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <chrono>
#include <future>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

#include <cl_moveit2z/cl_moveit2z.hpp>

namespace cl_moveit2z
{

/**
 * @brief CbMoveEndEffectorSeeded
 *
 * Plans and executes a move to a target end-effector pose, seeding the IK
 * solver from the current robot state to avoid wrist flips / large-jump
 * solutions. Includes isShutdownRequested() guards so that a Pause/cancel
 * mid-execution does not trigger a use-after-free crash.
 *
 * Usage (inline):
 *   configure_orthogonal<OrArm, CbMoveEndEffectorSeeded>(targetPose, "Link7");
 *
 * Usage (derived):
 *   class MyCb : public CbMoveEndEffectorSeeded {
 *     void onEntry() override {
 *       targetPose = buildPoseFromBlackboard();
 *       CbMoveEndEffectorSeeded::onEntry();  // delegate planning/execution
 *     }
 *   };
 */
class CbMoveEndEffectorSeeded : public smacc2::SmaccAsyncClientBehavior
{
public:
  geometry_msgs::msg::PoseStamped targetPose;
  std::string tip_link_;
  double planningTimeSec_{1.5};

  CbMoveEndEffectorSeeded() = default;

  CbMoveEndEffectorSeeded(
    geometry_msgs::msg::PoseStamped target_pose,
    std::string tip_link = "",
    double planning_time_sec = 1.5)
  : targetPose(target_pose), tip_link_(tip_link), planningTimeSec_(planning_time_sec)
  {
  }

  virtual void onEntry() override
  {
    this->requiresClient(movegroupClient_);
    auto & mgi = *(movegroupClient_->moveGroupClientInterface);

    if (isShutdownRequested())
    {
      RCLCPP_WARN(getLogger(), "[CbMoveEndEffectorSeeded] canceled before planning");
      return;
    }

    mgi.setPlanningTime(planningTimeSec_);
    mgi.setPoseReferenceFrame(targetPose.header.frame_id);
    mgi.setStartStateToCurrentState();

    const bool ikOk = mgi.setApproximateJointValueTarget(targetPose, tip_link_);
    if (!ikOk)
    {
      if (!isShutdownRequested())
      {
        RCLCPP_ERROR(
          getLogger(),
          "[CbMoveEndEffectorSeeded] IK target failed. frame=%s tip=%s",
          targetPose.header.frame_id.c_str(), tip_link_.c_str());
        movegroupClient_->postEventMotionExecutionFailed();
        this->postFailureEvent();
      }
      return;
    }

    if (isShutdownRequested())
    {
      RCLCPP_WARN(getLogger(), "[CbMoveEndEffectorSeeded] canceled after IK target set");
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planOk = (mgi.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!planOk)
    {
      if (!isShutdownRequested())
      {
        RCLCPP_WARN(getLogger(), "[CbMoveEndEffectorSeeded] Planning failed (seeded from current state)");
        movegroupClient_->postEventMotionExecutionFailed();
        this->postFailureEvent();
      }
      return;
    }

    if (isShutdownRequested())
    {
      RCLCPP_WARN(getLogger(), "[CbMoveEndEffectorSeeded] canceled after planning");
      return;
    }

    auto execFuture = std::async(std::launch::async, [&mgi, &plan]() { return mgi.execute(plan); });

    bool stopRequested = false;
    while (execFuture.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready)
    {
      if (isShutdownRequested() && !stopRequested)
      {
        RCLCPP_WARN(
          getLogger(),
          "[CbMoveEndEffectorSeeded] PAUSE/CANCEL requested during execute -> stop() sent");
        mgi.stop();
        stopRequested = true;
      }
    }

    const auto execResult = execFuture.get();
    const bool execOk = (execResult == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

    if (isShutdownRequested())
    {
      RCLCPP_WARN(
        getLogger(),
        "[CbMoveEndEffectorSeeded] canceled after execute (no success/failure event will be posted)");
      return;
    }

    if (execOk)
    {
      movegroupClient_->postEventMotionExecutionSucceded();
      this->postSuccessEvent();
    }
    else
    {
      RCLCPP_WARN(getLogger(), "[CbMoveEndEffectorSeeded] Execution failed");
      movegroupClient_->postEventMotionExecutionFailed();
      this->postFailureEvent();
    }
  }

protected:
  ClMoveit2z * movegroupClient_ = nullptr;
};

}  // namespace cl_moveit2z

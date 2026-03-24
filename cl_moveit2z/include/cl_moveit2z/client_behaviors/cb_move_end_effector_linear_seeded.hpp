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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <vector>
#include <optional>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>

#include <cl_moveit2z/cl_moveit2z.hpp>

namespace cl_moveit2z
{

/**
 * @brief CbMoveEndEffectorLinearSeeded
 *
 * Moves from current EE pose to target pose using linear Cartesian interpolation
 * in task space and computes a single cartesian trajectory for one-shot execution.
 * computeCartesianPath internally propagates along waypoints using sequential IK,
 * equivalent to using prior points as seeds.
 */
class CbMoveEndEffectorLinearSeeded : public smacc2::SmaccAsyncClientBehavior
{
public:
  geometry_msgs::msg::PoseStamped targetPose;
  std::string tip_link_;
  double planningTimeSec_{1.0};
  double linearStepMeters_{0.01};
  int maxSegments_{200};
  double jumpThreshold_{0.0};
  double minPathFraction_{0.98};
  std::optional<double> velocityScaling_;  // Optional velocity scaling (resets inherited value)

  CbMoveEndEffectorLinearSeeded() = default;

  CbMoveEndEffectorLinearSeeded(
    geometry_msgs::msg::PoseStamped target_pose,
    std::string tip_link = "",
    double linear_step_meters = 0.01,
    double planning_time_sec = 1.0)
  : targetPose(target_pose),
    tip_link_(tip_link),
    planningTimeSec_(planning_time_sec),
    linearStepMeters_(linear_step_meters)
  {
  }

  void onEntry() override
  {
    this->requiresClient(movegroupClient_);
    auto & mgi = *(movegroupClient_->moveGroupClientInterface);

    // Auto-load cartesian motion parameters from YAML (if available)
    // This allows tuning velocity/trajectory params without recompilation
    loadConfigFromYaml("cl_moveit2z", "config/cartesian.yaml");

    // Reset MoveGroup velocity scaling to prevent inheritance from prior joint moves
    if (velocityScaling_)
    {
      mgi.setMaxVelocityScalingFactor(*velocityScaling_);
    }
    else
    {
      mgi.setMaxVelocityScalingFactor(1.0);  // Reset to 100% by default
    }

    if (isShutdownRequested())
    {
      RCLCPP_WARN(getLogger(), "[CbMoveEndEffectorLinearSeeded] canceled before planning");
      return;
    }

    if (linearStepMeters_ <= 0.0)
    {
      linearStepMeters_ = 0.01;
    }

    mgi.setPlanningTime(planningTimeSec_);
    mgi.setPoseReferenceFrame(targetPose.header.frame_id);

    const std::string tipLink = tip_link_.empty() ? mgi.getEndEffectorLink() : tip_link_;
    const auto currentPose = mgi.getCurrentPose(tipLink);

    const double dx = targetPose.pose.position.x - currentPose.pose.position.x;
    const double dy = targetPose.pose.position.y - currentPose.pose.position.y;
    const double dz = targetPose.pose.position.z - currentPose.pose.position.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    int segments = std::max(1, static_cast<int>(std::ceil(distance / linearStepMeters_)));
    segments = std::min(segments, maxSegments_);

    tf2::Quaternion qStart;
    tf2::Quaternion qTarget;
    tf2::fromMsg(currentPose.pose.orientation, qStart);
    tf2::fromMsg(targetPose.pose.orientation, qTarget);
    qStart.normalize();
    qTarget.normalize();

    RCLCPP_INFO(
      getLogger(),
      "[CbMoveEndEffectorLinearSeeded] linear move start -> target, distance=%.4f m, segments=%d, step=%.4f m",
      distance, segments, linearStepMeters_);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(static_cast<size_t>(segments));

    for (int i = 1; i <= segments; i++)
    {
      if (isShutdownRequested())
      {
        RCLCPP_WARN(getLogger(), "[CbMoveEndEffectorLinearSeeded] canceled before building waypoint %d", i);
        return;
      }

      const double t = static_cast<double>(i) / static_cast<double>(segments);
      geometry_msgs::msg::Pose waypoint;
      waypoint.position.x = currentPose.pose.position.x + t * dx;
      waypoint.position.y = currentPose.pose.position.y + t * dy;
      waypoint.position.z = currentPose.pose.position.z + t * dz;

      const tf2::Quaternion qInterp = qStart.slerp(qTarget, t);
      waypoint.orientation = tf2::toMsg(qInterp);

      waypoints.push_back(waypoint);
    }

    if (isShutdownRequested())
    {
      RCLCPP_WARN(getLogger(), "[CbMoveEndEffectorLinearSeeded] canceled before cartesian path computation");
      return;
    }

    mgi.setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double pathFraction =
      mgi.computeCartesianPath(waypoints, linearStepMeters_, jumpThreshold_, trajectory, true);

    if (pathFraction < minPathFraction_)
    {
      if (!isShutdownRequested())
      {
        RCLCPP_WARN(
          getLogger(),
          "[CbMoveEndEffectorLinearSeeded] cartesian path incomplete: fraction=%.3f (< %.3f)",
          pathFraction, minPathFraction_);
        movegroupClient_->postEventMotionExecutionFailed();
        this->postFailureEvent();
      }
      return;
    }

    if (isShutdownRequested())
    {
      RCLCPP_WARN(getLogger(), "[CbMoveEndEffectorLinearSeeded] canceled before trajectory execution");
      return;
    }

    const bool execOk = executeTrajectoryInterruptible(mgi, trajectory);
    if (!execOk)
    {
      if (!isShutdownRequested())
      {
        RCLCPP_WARN(getLogger(), "[CbMoveEndEffectorLinearSeeded] trajectory execution failed");
        movegroupClient_->postEventMotionExecutionFailed();
        this->postFailureEvent();
      }
      return;
    }

    if (isShutdownRequested())
    {
      RCLCPP_WARN(
        getLogger(),
        "[CbMoveEndEffectorLinearSeeded] canceled after execute (no success/failure event will be posted)");
      return;
    }

    movegroupClient_->postEventMotionExecutionSucceded();
    this->postSuccessEvent();
  }

  /**
   * Load cartesian motion parameters from YAML file.
   * YAML format:
   *   linear_step_meters: 0.01
   *   planning_time_sec: 1.0
   *   max_segments: 200
   *   jump_threshold: 0.0
   *   min_path_fraction: 0.98
   *   velocity_scaling: 1.0  # optional - sets scaling (1.0 = 100%)
   */
  void loadConfigFromYaml(const std::string & pkg, const std::string & filepath)
  {
    std::string pkgpath = ament_index_cpp::get_package_share_directory(pkg);

    if (pkgpath.empty())
    {
      RCLCPP_ERROR_STREAM(
        getLogger(), "[CbMoveEndEffectorLinearSeeded] package not found: " << pkg);
      return;
    }

    std::string fullpath = pkgpath + "/" + filepath;

    if (!std::filesystem::exists(fullpath))
    {
      RCLCPP_WARN_STREAM(getLogger(), "[CbMoveEndEffectorLinearSeeded] config file not found: " << fullpath);
      return;
    }

    try
    {
      std::ifstream ifs(fullpath);
      YAML::Node node = YAML::Load(ifs);

      if (node["linear_step_meters"])
      {
        linearStepMeters_ = node["linear_step_meters"].as<double>();
      }
      if (node["planning_time_sec"])
      {
        planningTimeSec_ = node["planning_time_sec"].as<double>();
      }
      if (node["max_segments"])
      {
        maxSegments_ = node["max_segments"].as<int>();
      }
      if (node["jump_threshold"])
      {
        jumpThreshold_ = node["jump_threshold"].as<double>();
      }
      if (node["min_path_fraction"])
      {
        minPathFraction_ = node["min_path_fraction"].as<double>();
      }
      if (node["velocity_scaling"])
      {
        velocityScaling_ = node["velocity_scaling"].as<double>();
      }

      RCLCPP_INFO(
        getLogger(),
        "[CbMoveEndEffectorLinearSeeded] loaded config from %s: step=%.4f, planTime=%.2f, maxSegs=%d, jumpThresh=%.4f, minPathFrac=%.3f, velScale=%s",
        fullpath.c_str(),
        linearStepMeters_,
        planningTimeSec_,
        maxSegments_,
        jumpThreshold_,
        minPathFraction_,
        velocityScaling_ ? std::to_string(*velocityScaling_).c_str() : "default(1.0)");
    }
    catch (const YAML::Exception & ex)
    {
      RCLCPP_ERROR_STREAM(
        getLogger(),
        "[CbMoveEndEffectorLinearSeeded] YAML error loading " << fullpath << ": " << ex.what());
    }
  }

protected:
  bool executeTrajectoryInterruptible(
    moveit::planning_interface::MoveGroupInterface & mgi,
    const moveit_msgs::msg::RobotTrajectory & trajectory)
  {
    auto execFuture =
      std::async(std::launch::async, [&mgi, &trajectory]() { return mgi.execute(trajectory); });

    bool stopRequested = false;
    while (execFuture.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready)
    {
      if (isShutdownRequested() && !stopRequested)
      {
        RCLCPP_WARN(
          getLogger(),
          "[CbMoveEndEffectorLinearSeeded] PAUSE/CANCEL requested during execute -> stop() sent");
        mgi.stop();
        stopRequested = true;
      }
    }

    const auto execResult = execFuture.get();
    return (execResult == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  }

  ClMoveit2z * movegroupClient_ = nullptr;
};

}  // namespace cl_moveit2z

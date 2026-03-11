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

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "cb_move_end_effector.hpp"

namespace cl_moveit2z
{
class CbMoveCartesianState : public CbMoveEndEffector
{
public:
  CbMoveCartesianState(std::string pkg, std::string config_path)
  : pkg_(pkg), config_path_(config_path), tip_link_("tool0")
  {
  }

  CbMoveCartesianState(std::string pkg, std::string config_path, std::string tip_link)
  : pkg_(pkg), config_path_(config_path), tip_link_(tip_link)
  {
  }

  virtual ~CbMoveCartesianState() {}

  void onEntry() override
  {
    targetPose = loadCartesianStateFromFile(pkg_, config_path_);
    tip_link_ = tip_link_;  // Ensure tip_link is set
    CbMoveEndEffector::onEntry();
  }

private:
  std::string pkg_;
  std::string config_path_;
  std::string tip_link_;

  geometry_msgs::msg::PoseStamped loadCartesianStateFromFile(
    std::string pkg, std::string filepath)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_link";

    std::string pkgpath = ament_index_cpp::get_package_share_directory(pkg);

    if (pkgpath == "")
    {
      RCLCPP_ERROR_STREAM(
        getLogger(), "[" << getName() << "] package not found for the cartesian pose file: "
                         << pkg << std::endl
                         << " [IGNORING BEHAVIOR]");
      return pose;
    }

    filepath = pkgpath + "/" + filepath;

    RCLCPP_INFO_STREAM(
      getLogger(), "[" << getName() << "] Opening file with cartesian pose: " << filepath);

    if (std::filesystem::exists(filepath))
    {
      RCLCPP_INFO_STREAM(
        getLogger(), "[" << getName() << "] Cartesian pose file exists: " << filepath);
    }
    else
    {
      RCLCPP_ERROR_STREAM(
        getLogger(), "[" << getName() << "] Cartesian pose file does not exist: " << filepath);
      return pose;
    }

    std::ifstream ifs(filepath.c_str(), std::ifstream::in);
    if (ifs.good() == false)
    {
      RCLCPP_ERROR_STREAM(
        getLogger(), "[" << getName() << "] Error opening cartesian pose file: " << filepath);
      throw std::string("cartesian pose file not found");
    }

    try
    {
      YAML::Node node = YAML::Load(ifs);

      // Load position
      if (node["x"] && node["y"] && node["z"])
      {
        pose.pose.position.x = node["x"].as<double>();
        pose.pose.position.y = node["y"].as<double>();
        pose.pose.position.z = node["z"].as<double>();

        RCLCPP_INFO_STREAM(
          getLogger(), "[" << getName() << "] Loaded position: (" << pose.pose.position.x
                           << ", " << pose.pose.position.y << ", " << pose.pose.position.z
                           << ")");
      }
      else
      {
        RCLCPP_ERROR_STREAM(getLogger(), "[" << getName() << "] Missing x, y, or z in YAML");
        return pose;
      }

      // Load orientation (quaternion)
      if (node["qx"] && node["qy"] && node["qz"] && node["qw"])
      {
        pose.pose.orientation.x = node["qx"].as<double>();
        pose.pose.orientation.y = node["qy"].as<double>();
        pose.pose.orientation.z = node["qz"].as<double>();
        pose.pose.orientation.w = node["qw"].as<double>();

        RCLCPP_INFO_STREAM(
          getLogger(),
          "[" << getName() << "] Loaded orientation: (" << pose.pose.orientation.x << ", "
              << pose.pose.orientation.y << ", " << pose.pose.orientation.z << ", "
              << pose.pose.orientation.w << ")");
      }
      else
      {
        RCLCPP_WARN_STREAM(
          getLogger(),
          "[" << getName() << "] Missing quaternion in YAML, using default (0,0,0,1)");
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
      }

      // Load frame_id if provided
      if (node["frame_id"])
      {
        pose.header.frame_id = node["frame_id"].as<std::string>();
      }

      return pose;
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR_STREAM(getLogger(), "[" << getName() << "] Exception loading YAML: " << e.what());
      throw std::string("Error parsing cartesian pose YAML");
    }
  }
};
}  // namespace cl_moveit2z

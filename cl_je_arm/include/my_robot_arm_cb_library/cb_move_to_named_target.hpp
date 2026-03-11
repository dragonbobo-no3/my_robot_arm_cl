#pragma once

#include <smacc2/smacc.hpp>
#include <moveit2z_client/cl_moveit2z.hpp>

namespace my_robot_arm_cb_library
{

/**
 * @brief Client behavior to move arm to a named target position
 * 
 * Wraps MoveIt2 named target functionality for convenience.
 * Posts success/failure events based on motion result.
 */
template <typename TSuccess, typename TFailure>
class CbMoveToNamedTarget : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbMoveToNamedTarget(const std::string& target_name)
    : target_name_(target_name)
  {
  }

  virtual ~CbMoveToNamedTarget()
  {
  }

  void onEntry() override
  {
    RCLCPP_INFO(getLogger(), "[CbMoveToNamedTarget] Moving to: %s", target_name_.c_str());
    
    // TODO: Get MoveIt2 client and execute motion
    // auto moveit_client = this->getClient<cl_moveit2z::ClMoveIt2z>();
    // moveit_client->setNamedTarget(target_name_);
    // if (moveit_client->execute()) {
    //   this->postEvent<TSuccess>();
    // } else {
    //   this->postEvent<TFailure>();
    // }
  }

  void onExit() override
  {
    RCLCPP_INFO(getLogger(), "[CbMoveToNamedTarget] Motion complete/cancelled");
  }

private:
  std::string target_name_;
};

}  // namespace my_robot_arm_cb_library

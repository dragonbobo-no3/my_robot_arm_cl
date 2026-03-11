#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

namespace my_robot_arm_cb_library
{

/**
 * @brief Client behavior for timeout monitoring
 * 
 * Monitors operation duration and posts timeout event when exceeded.
 * Can be used to ensure states don't hang indefinitely.
 */
template <typename TTimeout>
class CbTimeoutWatchdog : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbTimeoutWatchdog(double timeout_seconds = 30.0)
    : timeout_seconds_(timeout_seconds)
  {
  }

  virtual ~CbTimeoutWatchdog()
  {
  }

  void onEntry() override
  {
    RCLCPP_INFO(getLogger(), "[CbTimeoutWatchdog] Starting watchdog (%.1fs)", timeout_seconds_);
    
    start_time_ = this->getNode()->now();
    
    // TODO: Set up timer to check elapsed time
    // When timeout expires, post TTimeout event:
    // this->postEvent<TTimeout>();
  }

  void onExit() override
  {
    auto elapsed = (this->getNode()->now() - start_time_).seconds();
    RCLCPP_INFO(getLogger(), "[CbTimeoutWatchdog] Exiting after %.1fs", elapsed);
  }

private:
  double timeout_seconds_;
  rclcpp::Time start_time_;
};

}  // namespace my_robot_arm_cb_library

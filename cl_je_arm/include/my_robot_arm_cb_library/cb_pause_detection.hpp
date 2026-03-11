#pragma once

#include <smacc2/smacc.hpp>
#include <keyboard_client/cl_keyboard.hpp>

namespace my_robot_arm_cb_library
{

/**
 * @brief Client behavior to detect pause requests
 * 
 * Monitors keyboard input or other pause triggers and posts
 * EvPauseRequested when pause is detected.
 */
template <typename TPauseEvent>
class CbPauseDetection : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbPauseDetection(char pause_key = 'p')
    : pause_key_(pause_key)
  {
  }

  virtual ~CbPauseDetection()
  {
  }

  void onEntry() override
  {
    RCLCPP_INFO(getLogger(), "[CbPauseDetection] Monitoring for pause (key: %c)", pause_key_);
    
    // TODO: Subscribe to keyboard client events
    // When pause key detected:
    // this->postEvent<TPauseEvent>();
  }

  void onExit() override
  {
    RCLCPP_INFO(getLogger(), "[CbPauseDetection] Pause monitoring stopped");
  }

private:
  char pause_key_;
};

}  // namespace my_robot_arm_cb_library

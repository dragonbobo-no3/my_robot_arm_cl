# my_robot_arm_cb_library

Custom client behaviors for the my_robot_arm state machine.

## Overview

This package provides reusable client behaviors specific to the robot arm workflow:

- **CbCheckResources**: Monitors resource availability (PCB presence, slot availability)
- **CbTimeoutWatchdog**: Generic timeout monitoring with configurable duration
- **CbMoveToNamedTarget**: Convenient wrapper for MoveIt2 named target motions
- **CbPauseDetection**: Monitors for pause requests (keyboard or other triggers)

## Usage

Include the behaviors in your state machine states:

```cpp
#include <my_robot_arm_cb_library/cb_check_resources.hpp>
#include <my_robot_arm_cb_library/cb_timeout_watchdog.hpp>

// In state configure():
this->createAsyncClientBehavior<my_robot_arm_cb_library::CbCheckResources>();
```

## Dependencies

- smacc2
- cl_moveit2z
- cl_keyboard  
- cl_ros2_timer

## Development

Add new custom behaviors as header files in `include/my_robot_arm_cb_library/`.
This is a header-only library for simplicity.

cmake_minimum_required(VERSION 3.8)
project(my_robot_arm_cb_library)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(smacc2 REQUIRED)
find_package(cl_moveit2z REQUIRED)
find_package(cl_keyboard REQUIRED)
find_package(cl_ros2_timer REQUIRED)

include_directories(include)

# This is a header-only library
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  smacc2
  cl_moveit2z
  cl_keyboard
  cl_ros2_timer
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(thor_joystick)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(thor_server REQUIRED)

add_executable(joystick_moveit_controller src/joystick_moveit_controller.cpp)
target_include_directories(joystick_moveit_controller PUBLIC include)
ament_target_dependencies(joystick_moveit_controller rclcpp sensor_msgs geometry_msgs moveit_ros_planning_interface)

add_executable(joystick_joint_controller src/joystick_joint_controller.cpp)
ament_target_dependencies(joystick_joint_controller rclcpp rclcpp_action sensor_msgs thor_server)

add_executable(joystick_pose_controller src/joystick_pose_controller.cpp)
ament_target_dependencies(joystick_pose_controller rclcpp rclcpp_action sensor_msgs thor_server)

install(TARGETS
  joystick_moveit_controller
  joystick_joint_controller
  joystick_pose_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

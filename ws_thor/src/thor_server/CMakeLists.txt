cmake_minimum_required(VERSION 3.8)
project(thor_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/JointTask.action"
  "action/PoseTask.action"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(task_server src/task_server.cpp)
target_include_directories(task_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(task_server PRIVATE "TASK_SERVER_BUILDING_DLL")
ament_target_dependencies(task_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "moveit_ros_planning_interface"
)
rclcpp_components_register_node(task_server
  PLUGIN "thor_server::TaskServer"
  EXECUTABLE task_server_node
)

target_link_libraries(task_server "${cpp_typesupport_target}") 

install(DIRECTORY action
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    task_server
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  task_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

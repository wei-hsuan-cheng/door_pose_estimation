cmake_minimum_required(VERSION 3.10)
project(door_pose_estimation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/EstimateDoorPoses.action"
  DEPENDENCIES action_msgs geometry_msgs
)

add_executable(door_pose_estimation_action_server
  src/door_pose_estimation_action_server.cpp
)
ament_target_dependencies(door_pose_estimation_action_server
  geometry_msgs
  rclcpp
  rclcpp_action
  tf2
  tf2_geometry_msgs
  tf2_ros
)

if(COMMAND rosidl_get_typesupport_target)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(door_pose_estimation_action_server "${cpp_typesupport_target}")
else()
  rosidl_target_interfaces(door_pose_estimation_action_server ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

install(TARGETS
  door_pose_estimation_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY action launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

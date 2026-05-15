cmake_minimum_required(VERSION 3.8)
project(hardware_tools)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(tmc_control_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(head_interfaces REQUIRED)
find_package(torso_interfaces REQUIRED)
find_package(arms_interfaces REQUIRED)
find_package(grippers_interfaces REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${head_interfaces_INCLUDE_DIRS}
  ${torso_interfaces_INCLUDE_DIRS}
  ${arms_interfaces_INCLUDE_DIRS}
  ${grippers_interfaces_INCLUDE_DIRS}
)

add_library(hardware_tools SHARED
  src/Hardware.cpp
)

ament_target_dependencies(hardware_tools
  rclcpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  std_srvs
  tf2_ros
  head_interfaces
  torso_interfaces
  arms_interfaces
  grippers_interfaces
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS hardware_tools
  DESTINATION lib
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  std_srvs
  tf2_ros
  head_interfaces
  torso_interfaces
  arms_interfaces
  grippers_interfaces
)

ament_export_include_directories(include)
ament_export_libraries(hardware_tools)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(hardware_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(actionlib_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

add_library(hardware_tools SHARED
  src/Hardware.cpp
)

add_executable(head_control_node src/head_control_node.cpp)


ament_target_dependencies(hardware_tools
  rclcpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  std_srvs
  tf2_ros
)

ament_target_dependencies(head_control_node
  rclcpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  actionlib_msgs
)

ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  std_srvs
  tf2_ros
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS hardware_tools
  DESTINATION lib
)

install(Target
  head_control_node
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(hardware_tools)

ament_package()

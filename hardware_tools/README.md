# hardware_tools
This repository provides a utility class to interface with hardware in ROS 2. 

## Description
The `Hardware` class takes the shared pointer to a node and makes it publisher to the following topics:
* `/hardware/arms/goal_gripper` [`std_msgs::msg::Float64`]

* `/hardware/arms/goal_gripper` [`std_msgs::msg::Float64MultiArray`]

And subscribe it as a client to the folowing service:
* `/hardware/door_detector/check` [`std_srvs::srv::Trigger`]

Therefore, the class provides an interface to open, close and relax the gripper; to move the head to a given pan and tilt angles, and to verify if the obstacle in front of the robot is door-shaped.

## Usage example
Include the header:
```cpp
#include "hardware_tools/Hardware.hpp"
```
Then:
```cpp
auto hardware = std::make_shared<Hardware>(node);

bool door = hardware->CheckDoor();

if (door)
{
  RCLCPP_INFO(node->get_logger(), "Door detected");
}
else
{
  RCLCPP_WARN(node->get_logger(), "Continue");
}
```

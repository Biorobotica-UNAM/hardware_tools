#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "tf2_ros/transform_broadcaster.h"

class Hardware
{
public:
    // Constructor
    Hardware(rclcpp::Node::SharedPtr node);

    // Methods
    void OpenGripper();

    void CloseGripper();

    void RelaxGripper();
    
    void MoveHead(double pan, double tilt);
private:
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_goal_gripper;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_head_angles;
    
    // Attributes
    rclcpp::Node::SharedPtr node;    
};
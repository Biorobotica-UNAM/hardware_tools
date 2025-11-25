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
private:
    // Constructor
    Hardware(rclcpp::Node::SharedPtr node);

    // Publishers


    //Subscribers


    // Clients


    // Attributes
    rclcpp::Node::SharedPtr node;
    
public:
    // Methods
    
};
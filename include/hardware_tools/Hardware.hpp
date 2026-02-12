#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "std_srvs/srv/trigger.hpp"

class Hardware
{
public:
    explicit Hardware(const rclcpp::Node::SharedPtr& node);

    void OpenGripper();
    void CloseGripper();
    void RelaxGripper();
    void MoveHead(double pan, double tilt);
    void MoveTorso(float target);

    bool CheckDoor();

private:
    bool waitForService(
        const rclcpp::ClientBase::SharedPtr& client,
        const std::chrono::milliseconds& timeout) const;

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_goal_gripper_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_head_angles_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_torso_target_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_check_door_;
};


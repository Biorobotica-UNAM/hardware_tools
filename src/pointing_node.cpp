// Created by joshua on 6/11/24.

//C++ headers
#include <cmath>
#include <algorithm>

//ROS headers
#include <rclcpp/rclcpp.hpp>
//Actionlib messages
#include <actionlib_msgs/msg/goal_status.hpp>
//Geometry messages
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
//Standard messages
#include <std_msgs/msg/float64_multi_array.hpp>
//Standard services
#include <std_srvs/srv/trigger.hpp>
//TF2 headers
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>


class PointingNode : public rclcpp::Node
{
public:
  PointingNode() : Node("pointing_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {

    // Declare parameters
    this->declare_parameter<std::string>( "head_goal_topic",          "/hardware/head/goal_pose");
    this->declare_parameter<std::string>( "head_current_pose_topic",  "/hardware/head/current_pose");
    //this->declare_parameter<std::string>( "head_goal_reached_topic",  "/hardware/head/goal_reached");

    this->declare_parameter<std::string>("camera_frame",                "neck_link");
    this->declare_parameter<std::string>("base_frame",                  "base_link");
    this->declare_parameter<std::string>("neck_frame",                  "torso_lift_link");

    this->declare_parameter<double>("pan_min",              -1.57);
    this->declare_parameter<double>("pan_max",              1.57);
    this->declare_parameter<double>("tilt_min",             -0.9);
    this->declare_parameter<double>("tilt_max",             0.47);
    this->declare_parameter<double>("dead_zone_threshold",  0.05);

    head_goal_topic_    = this->get_parameter("head_goal_topic").as_string();
    head_cp_topic_      = this->get_parameter("head_current_pose_topic").as_string();

    camera_frame_   = this->get_parameter("camera_frame").as_string();
    base_frame_     = this->get_parameter("base_frame").as_string();
    neck_frame_     = this->get_parameter("neck_frame").as_string();

    pan_min     = static_cast<float>(this->get_parameter("pan_min").as_double());
    pan_max     = static_cast<float>(this->get_parameter("pan_max").as_double());
    tilt_min    = static_cast<float>(this->get_parameter("tilt_min").as_double());
    tilt_max    = static_cast<float>(this->get_parameter("tilt_max").as_double());
    dead_zone_threshold = static_cast<float>(this->get_parameter("dead_zone_threshold").as_double());
    
    // Service Server
    srv_pointing_ = this->create_service<std_srvs::srv::Trigger>(
        "enable", std::bind(&PointingNode::enablePointingControl, this, std::placeholders::_1, std::placeholders::_2));
    
    // Subscribers
    head_cp_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        head_cp_topic_, 10,
        std::bind(&PointingNode::headCurrentPoseCallback, this, std::placeholders::_1));
        
    // Publishers
    head_goal_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        head_goal_topic_, 10);

    base_cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/hardware/base/cmd_vel", 10);
    
    // Timers
    //control_timer_ = this->create_wall_timer(
    //    std::chrono::milliseconds(40), std::bind(&PointingNode::control_cycle, this));
            
    RCLCPP_INFO(this->get_logger(), "HEAD CONTROL NODE BRIDGE BY RUSANROD HAS BEEN STARTED.");
}
private:
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr head_cp_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr look_at_subscriber_;
    rclcpp::Subscription<actionlib_msgs::msg::GoalStatus>::SharedPtr nav_status_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr head_goal_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_cmd_vel_publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_pointing_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Parameters
    std::string head_goal_topic_;
    std::string head_cp_topic_;

    std::string camera_frame_;
    std::string base_frame_;
    std::string neck_frame_;
    
    geometry_msgs::msg::PointStamped::SharedPtr latest_target_point_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // System variables
    float pan_min, pan_max, tilt_min, tilt_max, dead_zone_threshold;

    float current_pan = 0.0;
    float current_tilt = 0.0;
    float alpha = 0.85;

    bool new_target_ = false;

    // Utility functions

    std::vector<float> clampHeadPose(float goalPan, float goalTilt)
    {
        // Pan limits
        float clampedPan = std::clamp(goalPan, pan_min, pan_max);
        float clampedTilt = std::clamp(goalTilt, tilt_min, tilt_max);

        return {clampedPan, clampedTilt};
    }

    // Callbacks
    void headCurrentPoseCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Current pose message has less than 2 elements.");
            return;
        }

        current_pan = static_cast<float>(msg->data[0]);
        current_tilt = static_cast<float>(msg->data[1]);
    }

    void lookAtCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg){
        latest_target_point_ = msg;
        new_target_ = true;
    }

    void enablePointingControl(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        geometry_msgs::msg::Twist cmd_vel_msgs;

            auto init = this->now();
                        rclcpp::Rate rate(10);
        std_msgs::msg::Float64MultiArray head_goal_msg;
        float current_pan_rad = current_pan;
        float current_tilt_rad = current_tilt;

        head_goal_msg.data = {current_pan_rad/2, current_tilt_rad};
        try {
            // 1. Transformar el punto al frame fijo del cuello (torso_lift_link o similar)
            //geometry_msgs::msg::PointStamped p_neck;
            //tf_buffer_.transform(*latest_target_point_, p_neck, neck_frame_, tf2::durationFromSec(0.01));

            // 2. Obtener la posición de la CÁMARA respecto al cuello (OFFSET estático)
            // Esto nos dice dónde está el "ojo" respecto al eje de rotación.
                RCLCPP_INFO(this->get_logger(), "Trrrry");
            geometry_msgs::msg::TransformStamped cam_tf = 
                tf_buffer_.lookupTransform(base_frame_, camera_frame_, tf2::TimePointZero);
            
            double off_x = cam_tf.transform.translation.x;
            double off_y = cam_tf.transform.translation.y;
            double off_z = cam_tf.transform.translation.z;

            double off_theta = cam_tf.transform.rotation.z; 
            
            RCLCPP_INFO(this->get_logger(), "off_theta: %f", off_theta);

            cmd_vel_msgs.linear.x = 0.0;
            cmd_vel_msgs.linear.y = 0.0;
            
            cmd_vel_msgs.angular.z = off_theta < 0.0 ? -0.65 : 0.65;

            float time_out = abs(off_theta/cmd_vel_msgs.angular.z) + 0.25;

            while (rclcpp::ok() && (this->now() - init).seconds() < time_out)
            {
                RCLCPP_INFO(this->get_logger(), "Publishing CMD_VEL: %f", cmd_vel_msgs.angular.z);
                base_cmd_vel_publisher_->publish(cmd_vel_msgs);
                head_goal_publisher_->publish(head_goal_msg);
                rate.sleep();
            }

            head_goal_msg.data = {0.0, current_tilt_rad};
            
            head_goal_publisher_->publish(head_goal_msg);
            response->success = true;
            response->message = "Pointing control enabled and okaaaz.";


        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF wait: %s", ex.what());
        }
    }

    void navigationStatusCallback(const actionlib_msgs::msg::GoalStatus::SharedPtr msg)
    {
        // Check if the navigation goal has been reached
        if (msg->status == actionlib_msgs::msg::GoalStatus::SUCCEEDED) {
            std_msgs::msg::Float64MultiArray head_goal_msg;
            head_goal_msg.data = {0.0, 0.0}; // Reset to default pose
            head_goal_publisher_->publish(head_goal_msg);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
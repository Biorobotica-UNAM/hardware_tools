// ----- C++ headers -----
#include <cmath>
#include <algorithm>

// ----- ROS -----
#include <rclcpp/rclcpp.hpp>
// Geometry msgs
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
// Standard msgs
#include <std_msgs/msg/float64_multi_array.hpp>
// Services
#include <std_srvs/srv/trigger.hpp>
// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class PointingNode : public rclcpp::Node
{
public:
    PointingNode()
    : Node("pointing_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        // Params
        this->declare_parameter("use_takeshi", false);
        this->declare_parameter("head_goal_topic", "/hardware/head/goal_pose");
        this->declare_parameter("head_state_topic", "/hardware/head/current_pose");
        this->declare_parameter("arm_goal_topic", "/hardware/arm/goal_pose");

        this->declare_parameter("camera_frame", "neck_link");
        this->declare_parameter("base_frame", "base_link");

        use_takeshi_ = this->get_parameter("use_takeshi").as_bool();
        head_goal_topic_ = this->get_parameter("head_goal_topic").as_string();
        head_state_topic_ = this->get_parameter("head_state_topic").as_string();
        arm_goal_topic_ = this->get_parameter("arm_goal_topic").as_string();

        camera_frame_ = this->get_parameter("camera_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();

        // Subs
        head_state_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            head_state_topic_, 10,
            std::bind(&PointingNode::headStateCallback, this, std::placeholders::_1));

        // Pubs
        head_goal_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(head_goal_topic_, 10);
        base_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/hardware/base/cmd_vel", 10);
        arm_goal_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(arm_goal_topic_, 10);

        // Service
        srv_ = this->create_service<std_srvs::srv::Trigger>(
            "enable",
            std::bind(&PointingNode::executeMotion, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Pointing node already running. --- By Joshua M");
    }

private:
    // Subs / Pubs
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr head_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr head_goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_cmd_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Params
    std::string head_goal_topic_, head_state_topic_, arm_goal_topic_;
    std::string camera_frame_, base_frame_;
    bool use_takeshi_;

    // State
    float pan_ = 0.0;
    float tilt_ = 0.0;

    // Smooth factor
    float alpha_ = 0.1;

    void headStateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) return;
        pan_ = msg->data[0];
        tilt_ = msg->data[1];
    }

    float smooth(float current, float target)
    {
        return current + alpha_ * (target - current);
    }

    void executeMotion(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        rclcpp::Rate rate(30);

        try {
            auto tf = tf_buffer_.lookupTransform(base_frame_, camera_frame_, tf2::TimePointZero);

            double theta = tf.transform.rotation.z;

            geometry_msgs::msg::Twist cmd;
            cmd.angular.z = theta < 0 ? -0.6 : 0.6;

            double duration = std::abs(theta / cmd.angular.z);

            auto start = this->now();

            float target_pan = 0.0;

            while ((this->now() - start).seconds() < duration)
            {
                float desired_pan = smooth(pan_, target_pan);

                std_msgs::msg::Float64MultiArray head_msg;
                head_msg.data = {desired_pan, tilt_};

                base_cmd_pub_->publish(cmd);
                head_goal_pub_->publish(head_msg);

                rate.sleep();
            }

            // STOP base
            cmd.angular.z = 0.0;
            base_cmd_pub_->publish(cmd);

            // Final head align
            std_msgs::msg::Float64MultiArray head_msg;
            head_msg.data = {0.0, tilt_};
            head_goal_pub_->publish(head_msg);

            // Arm pointing
            std_msgs::msg::Float64MultiArray arm_msg;

            arm_msg.data = use_takeshi_ ? std::vector<double>{0.0, 0.0, 0.0, 0.0} : std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            arm_goal_pub_->publish(arm_msg);

            res->success = true;
            res->message = "Pointing motion executed successfully.";

        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF error: %s", ex.what());
            res->success = false;
        }
    }
};
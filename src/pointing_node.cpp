// ----- C++ headers -----
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

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
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

        use_takeshi_     = this->get_parameter("use_takeshi").as_bool();
        head_goal_topic_ = this->get_parameter("head_goal_topic").as_string();
        head_state_topic_= this->get_parameter("head_state_topic").as_string();
        arm_goal_topic_  = this->get_parameter("arm_goal_topic").as_string();
        camera_frame_    = this->get_parameter("camera_frame").as_string();
        base_frame_      = this->get_parameter("base_frame").as_string();

        // Subs
        head_state_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            head_state_topic_,
            10,
            std::bind(&PointingNode::headStateCallback, this, std::placeholders::_1));

        // Pubs
        head_goal_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(head_goal_topic_, 10);
        base_cmd_pub_  = this->create_publisher<geometry_msgs::msg::Twist>("/hardware/base/cmd_vel", 10);
        arm_goal_pub_  = this->create_publisher<std_msgs::msg::Float64MultiArray>(arm_goal_topic_, 10);

        // Service
        srv_ = this->create_service<std_srvs::srv::Trigger>(
            "enable",
            std::bind(&PointingNode::executeMotion, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Pointing node started.");
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
    std::string head_goal_topic_;
    std::string head_state_topic_;
    std::string arm_goal_topic_;
    std::string camera_frame_;
    std::string base_frame_;
    bool use_takeshi_{false};

    // State
    double pan_{0.0};
    double tilt_{0.0};

    // Smooth factor
    double alpha_{0.1};

    void headStateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (!msg || msg->data.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Invalid head state message.");
            return;
        }

        pan_ = msg->data[0];
        tilt_ = msg->data[1];
    }

    double smooth(double current, double target) const
    {
        return current + alpha_ * (target - current);
    }

    void stopBase()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = 0.0;
        base_cmd_pub_->publish(cmd);
    }

    void publishFinalArmPose()
    {
        std_msgs::msg::Float64MultiArray arm_msg;

        if (use_takeshi_) {
            arm_msg.data = {0.0, 0.0, 0.0, 0.0};
        } else {
            arm_msg.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        }

        arm_goal_pub_->publish(arm_msg);
    }

    void executeMotion(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        rclcpp::Rate rate(30.0);

        try {
            const auto tf = tf_buffer_.lookupTransform(base_frame_, camera_frame_, tf2::TimePointZero);

            const double theta = tf2::getYaw(tf.transform.rotation);


            double current_pan = pan_;
            const double current_tilt = tilt_;
            const double target_pan = 0.0;

            if (std::abs(theta) > 1e-3) {
                geometry_msgs::msg::Twist cmd;
                const double angular_speed = 0.6;

                cmd.angular.z = (theta < 0.0) ? -angular_speed : angular_speed;
                const double duration = std::abs(theta / angular_speed);

                const auto start = this->now();

                while (rclcpp::ok() && (this->now() - start).seconds() < duration) {
                    current_pan = smooth(current_pan, target_pan);

                    std_msgs::msg::Float64MultiArray head_msg;
                    head_msg.data = {current_pan, current_tilt};

                    base_cmd_pub_->publish(cmd);
                    head_goal_pub_->publish(head_msg);

                    rate.sleep();
                }
            }

            // STOP base
            stopBase();

            // Final head align
            std_msgs::msg::Float64MultiArray head_msg;
            head_msg.data = {0.0, current_tilt};
            head_goal_pub_->publish(head_msg);

            // Arm pointing
            publishFinalArmPose();

            res->success = true;
            res->message = "Pointing motion executed successfully.";
        }
        catch (const tf2::TransformException & ex) {
            stopBase();
            RCLCPP_WARN(this->get_logger(), "TF error: %s", ex.what());
            res->success = false;
            res->message = std::string("TF error: ") + ex.what();
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
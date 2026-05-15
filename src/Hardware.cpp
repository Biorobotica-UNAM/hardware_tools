#include "hardware_tools/Hardware.hpp"

// ============================================================
// Constructor
// ============================================================

Hardware::Hardware(const rclcpp::Node::SharedPtr& node)
: node_(node)
{
    cb_group_clients_ =
        node_->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

    const rclcpp::QoS qos = rclcpp::ServicesQoS();

    // =========================================================
    // Service clients
    // =========================================================

    torso_client_ =
        node_->create_client<
            torso_interfaces::srv::MoveTorso>(
                "/hardware/torso/move_torso",
                qos,
                cb_group_clients_);

    head_client_ =
        node_->create_client<
            head_interfaces::srv::MoveHead>(
                "/hardware/head/move_head",
                qos,
                cb_group_clients_);

    left_arm_client_ =
        node_->create_client<
            arms_interfaces::srv::MoveArms>(
                "/hardware/arms/move_left_arm",
                qos,
                cb_group_clients_);

    right_arm_client_ =
        node_->create_client<
            arms_interfaces::srv::MoveArms>(
                "/hardware/arms/move_right_arm",
                qos,
                cb_group_clients_);

    left_gripper_client_ =
        node_->create_client<
            grippers_interfaces::srv::MoveGrippers>(
                "/hardware/grippers/move_left_gripper",
                qos,
                cb_group_clients_);

    right_gripper_client_ =
        node_->create_client<
            grippers_interfaces::srv::MoveGrippers>(
                "/hardware/grippers/move_right_gripper",
                qos,
                cb_group_clients_);

	check_door_client_ =
    node_->create_client<
        std_srvs::srv::Trigger>(
            "/hardware/door_detector/check",
            qos,
            cb_group_clients_);

    // =========================================================
    // Base cmd_vel publisher
    // =========================================================

    cmd_vel_pub_ =
        node_->create_publisher<geometry_msgs::msg::Twist>(
            "/hardware/base/cmd_vel",
            rclcpp::QoS(rclcpp::KeepLast(10))
                .reliable()
                .transient_local());
}

// ============================================================
// MoveTorso
// ============================================================

bool Hardware::MoveTorso(double position,
                         std::chrono::seconds timeout)
{
    auto request =
        std::make_shared<
            torso_interfaces::srv::MoveTorso::Request>();

    request->position = position;

    bool success = false;

    bool completed =
        call_service<
            torso_interfaces::srv::MoveTorso>(
                torso_client_,
                request,
                [&](auto response)
                {
                    success = response->success;

                    RCLCPP_INFO(
                        node_->get_logger(),
                        "MoveTorso %s: %s",
                        success ? "SUCCESS" : "FAILED",
                        response->message.c_str());
                },
                timeout,
                "MoveTorso");

    return completed && success;
}

// ============================================================
// MoveHead
// ============================================================

bool Hardware::MoveHead(const std::vector<double>& positions,
                        std::chrono::seconds timeout)
{
    auto request =
        std::make_shared<
            head_interfaces::srv::MoveHead::Request>();

    request->positions = positions;

    bool success = false;

    bool completed =
        call_service<
            head_interfaces::srv::MoveHead>(
                head_client_,
                request,
                [&](auto response)
                {
                    success = response->success;

                    RCLCPP_INFO(
                        node_->get_logger(),
                        "MoveHead %s: %s",
                        success ? "SUCCESS" : "FAILED",
                        response->message.c_str());
                },
                timeout,
                "MoveHead");

    return completed && success;
}

// ============================================================
// MoveRightArm
// ============================================================

bool Hardware::MoveRightArm(
    const std::vector<double>& positions,
    int speed,
    std::chrono::seconds timeout)
{
    auto request =
        std::make_shared<
            arms_interfaces::srv::MoveArms::Request>();

    request->positions = positions;
    request->speed = speed;

    bool success = false;

    bool completed =
        call_service<
            arms_interfaces::srv::MoveArms>(
                right_arm_client_,
                request,
                [&](auto response)
                {
                    success = response->success;

                    RCLCPP_INFO(
                        node_->get_logger(),
                        "MoveRightArm %s: %s",
                        success ? "SUCCESS" : "FAILED",
                        response->message.c_str());
                },
                timeout,
                "MoveRightArm");

    return completed && success;
}

// ============================================================
// MoveLeftArm
// ============================================================

bool Hardware::MoveLeftArm(
    const std::vector<double>& positions,
    int speed,
    std::chrono::seconds timeout)
{
    auto request =
        std::make_shared<
            arms_interfaces::srv::MoveArms::Request>();

    request->positions = positions;
    request->speed = speed;

    bool success = false;

    bool completed =
        call_service<
            arms_interfaces::srv::MoveArms>(
                left_arm_client_,
                request,
                [&](auto response)
                {
                    success = response->success;

                    RCLCPP_INFO(
                        node_->get_logger(),
                        "MoveLeftArm %s: %s",
                        success ? "SUCCESS" : "FAILED",
                        response->message.c_str());
                },
                timeout,
                "MoveLeftArm");

    return completed && success;
}

// ============================================================
// MoveRightGripper
// ============================================================

bool Hardware::MoveRightGripper(
    const std::string& command,
    std::chrono::seconds timeout)
{
    auto request =
        std::make_shared<
            grippers_interfaces::srv::MoveGrippers::Request>();

    request->cmd = command;

    bool success = false;

    bool completed =
        call_service<
            grippers_interfaces::srv::MoveGrippers>(
                right_gripper_client_,
                request,
                [&](auto response)
                {
                    success = response->success;

                    RCLCPP_INFO(
                        node_->get_logger(),
                        "MoveRightGripper %s [%s]: %s",
                        success ? "SUCCESS" : "FAILED",
                        response->status.c_str(),
                        response->message.c_str());
                },
                timeout,
                "MoveRightGripper");

    return completed && success;
}

// ============================================================
// MoveLeftGripper
// ============================================================

bool Hardware::MoveLeftGripper(
    const std::string& command,
    std::chrono::seconds timeout)
{
    auto request =
        std::make_shared<
            grippers_interfaces::srv::MoveGrippers::Request>();

    request->cmd = command;

    bool success = false;

    bool completed =
        call_service<
            grippers_interfaces::srv::MoveGrippers>(
                left_gripper_client_,
                request,
                [&](auto response)
                {
                    success = response->success;

                    RCLCPP_INFO(
                        node_->get_logger(),
                        "MoveLeftGripper %s [%s]: %s",
                        success ? "SUCCESS" : "FAILED",
                        response->status.c_str(),
                        response->message.c_str());
                },
                timeout,
                "MoveLeftGripper");

    return completed && success;
}

// ============================================================
// MoveBase
// ============================================================

void Hardware::MoveBase(double linear_x,
                        double linear_y,
                        double angular_z)
{
    geometry_msgs::msg::Twist cmd;

    cmd.linear.x  = linear_x;
    cmd.linear.y  = linear_y;
    cmd.angular.z = angular_z;

    cmd_vel_pub_->publish(cmd);
}

// ============================================================
// CheckDoor
// ============================================================

bool Hardware::CheckDoor(std::chrono::seconds timeout)
{
    auto request =
        std::make_shared<
            std_srvs::srv::Trigger::Request>();

    bool success = false;

    bool completed =
        call_service<
            std_srvs::srv::Trigger>(
                check_door_client_,
                request,
                [&](auto response)
                {
                    success = response->success;

                    RCLCPP_INFO(
                        node_->get_logger(),
                        "CheckDoor %s: %s",
                        success ? "OPEN" : "CLOSED",
                        response->message.c_str());
                },
                timeout,
                "CheckDoor");

    return completed && success;
}

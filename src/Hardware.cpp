#include "hardware_tools/Hardware.hpp"

#include <future>

Hardware::Hardware(const rclcpp::Node::SharedPtr& node)
    : node_(node)
{
    pub_goal_gripper_ =
        node_->create_publisher<std_msgs::msg::Float64>(
            "/hardware/arms/goal_gripper", 10);

    pub_head_angles_ =
        node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/hardware/head/goal_pose", 10);
    
    pub_torso_target_ =
        node_->create_publisher<std_msgs::msg::Float32>(
            "/hardware/torso/goal", 10);

    cli_check_door_ =
        node_->create_client<std_srvs::srv::Trigger>(
            "/hardware/door_detector/check");
}

// ===== Gripper =====

void Hardware::OpenGripper()
{
    std_msgs::msg::Float64 msg;
    msg.data = -0.5236;
    pub_goal_gripper_->publish(msg);
}

void Hardware::CloseGripper()
{
    std_msgs::msg::Float64 msg;
    msg.data = 0.2618;
    pub_goal_gripper_->publish(msg);
}

void Hardware::RelaxGripper()
{
    std_msgs::msg::Float64 msg;
    msg.data = 0.0;
    pub_goal_gripper_->publish(msg);
}

// ===== Head =====

void Hardware::MoveHead(double pan, double tilt)
{
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {pan, tilt};
    pub_head_angles_->publish(msg);
}

// ===== Torso =====

void Hardware::MoveTorso(float target)
{
    std_msgs::msg::Float32 msg;
    msg.data = target;
    pub_torso_target_->publish(msg);
}

// ---------------------------------------------------------
// Wait for service
// ---------------------------------------------------------

bool Hardware::waitForService(
    const rclcpp::ClientBase::SharedPtr& client,
    const std::chrono::milliseconds& timeout) const
{
    return client->wait_for_service(timeout);
}

// ---------------------------------------------------------
// Check door
// ---------------------------------------------------------

bool Hardware::CheckDoor()
{
    using namespace std::chrono_literals;

    if (!waitForService(cli_check_door_, 500ms))
    {
        RCLCPP_WARN(node_->get_logger(),
            "Servicio /hardware/door_detector/check no disponible");
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    auto prom = std::make_shared<std::promise<bool>>();
    auto fut = prom->get_future();

    cli_check_door_->async_send_request(
        request,
        [this, prom](
            std::shared_future<
                std_srvs::srv::Trigger::Response::SharedPtr> future)
        {
            try
            {
                prom->set_value(future.get()->success);
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN(node_->get_logger(),
                    "Excepción en DoorDetector: %s", e.what());
                prom->set_value(false);
            }
        });

    if (fut.wait_for(500ms) == std::future_status::ready)
        return fut.get();

    RCLCPP_WARN(node_->get_logger(),
        "Timeout esperando DoorDetector");
    return false;
}


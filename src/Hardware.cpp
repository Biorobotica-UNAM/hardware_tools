#include "hardware_tools/Hardware.hpp"

Hardware::Hardware(rclcpp::Node::SharedPtr node) 
		: node(node) {
	// Initialize publishers
	pub_goal_gripper = node->create_publisher<std_msgs::msg::Float64>("/hardware/arms/goal_gripper", 10);
	pub_head_angles = node->create_publisher<std_msgs::msg::Float64MultiArray>("/hardware/head/angles", 10);
	
	// Initialize client
	cli_check_door = node->create_client<std_srvs::srv::Trigger>("/hardware/door_detector/check");

}

// Methods
void Hardware::OpenGripper() {
	auto message = std_msgs::msg::Float64();
	message.data = -0.5236;
	this->pub_goal_gripper->publish(message);
}

void Hardware::CloseGripper() {
	auto message = std_msgs::msg::Float64();
	message.data = 0.2618;
	this->pub_goal_gripper->publish(message);
}

void Hardware::RelaxGripper() {
	auto message = std_msgs::msg::Float64();
	message.data = 0.0;
	this->pub_goal_gripper->publish(message);
}

void Hardware::MoveHead(double pan, double tilt)
{
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {pan, tilt};
    pub_head_angles->publish(msg);
}

bool Hardware::CheckDoor()
{
    if (!cli_check_door->wait_for_service(std::chrono::milliseconds(500)))
    {
        RCLCPP_WARN(node->get_logger(),
            "DoorDetector service not available");
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = cli_check_door->async_send_request(request);

    if (rclcpp::spin_until_future_complete(
            node, future, std::chrono::milliseconds(500))
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_WARN(node->get_logger(),
            "Failed to call DoorDetector service");
        return false;
    }

    return future.get()->success;
}

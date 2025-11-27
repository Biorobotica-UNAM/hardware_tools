#include "hardware_tools/Hardware.hpp"

Hardware::Hardware(rclcpp::Node::SharedPtr node) 
		: node(node) {
	// Initialize publishers
	pub_goal_gripper = node->create_publisher<std_msgs::msg::Float64>("/hardware/arms/goal_gripper", 10);

	pub_head_angles = node->create_publisher<std_msgs::msg::Float64MultiArray>("/hardware/head/angles", 10);

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
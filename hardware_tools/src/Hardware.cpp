#include "hardware_tools/Hardware.cpp"

Hardware::Hardware(rclcpp::Node::SharedPtr node) 
		: node(node) {
	// Initialize publishers
	pub_goal_gripper = node->create_publisher<std_msgs::msg::Float64>("/hardware/arms/goal_gripper", 10);

	pub_head_angles = node->create_publisher<std_msgs::msg::Float64MultiArray>("/hardware/head/angles", 10);

}

// Methods
void Hardware::OpenGripper() {
	;
}

void Hardware::CloseGripper() {
	;
}

void Hardware::MoveHead(double pan, double tilt)
{
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {pan, tilt};
    pub_head_angles->publish(msg);
}
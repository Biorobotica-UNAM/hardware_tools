#pragma once

/**
 * @brief Facade over robot hardware ROS2 interfaces.
 *
 * Responsibility:
 *   Provide a clean synchronous API for robot hardware control:
 *     - head
 *     - torso
 *     - arms
 *     - grippers
 *     - base velocity
 *
 * Design patterns:
 *   Facade
 *   Template Method
 *
 * Principles:
 *   SRP
 *   DRY
 *   DIP
 *
 * BioRobotics Laboratory, Facultad de Ingenieria, UNAM
 */

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <unordered_map>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "rclcpp/rclcpp.hpp"

#include "arms_interfaces/srv/move_arms.hpp"
#include "grippers_interfaces/srv/move_grippers.hpp"
#include "head_interfaces/srv/move_head.hpp"
#include "torso_interfaces/srv/move_torso.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class Hardware
{
public:

    explicit Hardware(const rclcpp::Node::SharedPtr& node);

    // =========================================================
    // Torso
    // =========================================================

    bool MoveTorso(double position,
                   std::chrono::seconds timeout = 10s);

    // =========================================================
    // Head
    // =========================================================

    bool MoveHead(const std::vector<double>& positions,
                  std::chrono::seconds timeout = 10s);

    // =========================================================
    // Arms
    // =========================================================

    bool MoveRightArm(const std::vector<double>& positions,
                      int speed = 20,
                      std::chrono::seconds timeout = 15s);

    bool MoveLeftArm(const std::vector<double>& positions,
                     int speed = 20,
                     std::chrono::seconds timeout = 15s);

    // =========================================================
    // Grippers
    // =========================================================

    bool MoveRightGripper(const std::string& command,
                          std::chrono::seconds timeout = 10s);

    bool MoveLeftGripper(const std::string& command,
                         std::chrono::seconds timeout = 10s);

    // =========================================================
    // Base
    // =========================================================

    void MoveBase(double linear_x,
                  double linear_y,
                  double angular_z);

	// =========================================================
    // CheckDoor
    // =========================================================

	bool CheckDoor(std::chrono::seconds timeout = 3s);

	// ============================================================
	// GetCurrentPose (Complete)
	// ============================================================

	std::vector<double> GetCurrentPose(
		const std::string& component) const;

	// ============================================================
	// GetCurrentPose (Indexed)
	// ============================================================

	double GetCurrentPose(
		const std::string& component,
		size_t index) const;

    // ============================================================
    // Look At
    // ============================================================
    void LookAt(const geometry_msgs::msg::PointStamped &point) const;

    void LookAt(const geometry_msgs::msg::PoseStamped &pose) const;

private:

    rclcpp::Node::SharedPtr node_;

    rclcpp::CallbackGroup::SharedPtr cb_group_clients_;

	mutable std::mutex pose_mutex_;

	std::unordered_map<std::string, std::vector<double>> current_pose_;

    // =========================================================
    // Clients
    // =========================================================

    rclcpp::Client<torso_interfaces::srv::MoveTorso>::SharedPtr
        torso_client_;

    rclcpp::Client<head_interfaces::srv::MoveHead>::SharedPtr
        head_client_;

    rclcpp::Client<arms_interfaces::srv::MoveArms>::SharedPtr
        left_arm_client_;

    rclcpp::Client<arms_interfaces::srv::MoveArms>::SharedPtr
        right_arm_client_;

    rclcpp::Client<grippers_interfaces::srv::MoveGrippers>::SharedPtr
        left_gripper_client_;

    rclcpp::Client<grippers_interfaces::srv::MoveGrippers>::SharedPtr
        right_gripper_client_;

	rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
    	check_door_client_;

	// =========================================================
    // Subscribers
    // =========================================================

	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    	head_pose_sub_;

	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
		left_arm_pose_sub_;

	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
		right_arm_pose_sub_;

	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
		torso_pose_sub_;

    // =========================================================
    // Publishers
    // =========================================================

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_pub_;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
        look_at_pub_;

    // =========================================================
    // Template Method
    // =========================================================

    template<typename ServiceT, typename CallbackT>
    bool call_service(
        typename rclcpp::Client<ServiceT>::SharedPtr& client,
        typename ServiceT::Request::SharedPtr request,
        CallbackT on_response,
        std::chrono::seconds timeout,
        const std::string& label);
};

// ============================================================
// Template implementation
// ============================================================

template<typename ServiceT, typename CallbackT>
bool Hardware::call_service(
    typename rclcpp::Client<ServiceT>::SharedPtr& client,
    typename ServiceT::Request::SharedPtr request,
    CallbackT on_response,
    std::chrono::seconds timeout,
    const std::string& label)
{
    if (!client->wait_for_service(1s))
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "%s service not available.",
                     label.c_str());

        return false;
    }

    auto future = client->async_send_request(request);

    if (future.wait_for(timeout) != std::future_status::ready)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "Timeout calling %s service after %ld s.",
                     label.c_str(),
                     timeout.count());

        return false;
    }

    try
    {
        on_response(future.get());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "%s service exception: %s",
                     label.c_str(),
                     e.what());

        return false;
    }

    return true;
}

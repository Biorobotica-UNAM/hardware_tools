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

#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "rclcpp/rclcpp.hpp"

#include "arms_interfaces/srv/move_arms.hpp"
#include "grippers_interfaces/srv/move_grippers.hpp"
#include "head_interfaces/srv/move_head.hpp"
#include "torso_interfaces/srv/move_torso.hpp"

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

private:

    rclcpp::Node::SharedPtr node_;

    rclcpp::CallbackGroup::SharedPtr cb_group_clients_;

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
    // Base publisher
    // =========================================================

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_pub_;

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

#pragma once
#include "hardware_manager/trajectory_builder.hpp"

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace hardware_manager
{
using NativeAction = control_msgs::action::FollowJointTrajectory;
using NativeHandle = rclcpp_action::ClientGoalHandle<NativeAction>;
using StateMessage = control_msgs::msg::JointTrajectoryControllerState;

struct ControllerState
{
  Steady::time_point received{};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  Positions positions, errors;
};

enum class NativeState {Pending, Active, Rejected, Succeeded, Failed, Canceled, Unknown};

struct ControllerExecution
{
  NativeState state = NativeState::Pending;
  Steady::time_point sent = Steady::now();
  NativeHandle::SharedPtr handle;
  std::optional<rclcpp::Time> terminal_time;
  bool submitted = false;
  bool cancel_requested = false;
  bool cancel_sent = false;
  std::string failure;
  bool terminal() const;
};

class TrajectoryController
{
public:
  TrajectoryController(rclcpp::Node & node, const ControllerConfig & config,
    const rclcpp::CallbackGroup::SharedPtr & group);
  bool ready() const;
  bool available() const;
  const ControllerState * state(double max_age) const;
  std::shared_ptr<ControllerExecution> send(const NativeAction::Goal & goal);
  void cancel(const std::shared_ptr<ControllerExecution> & execution);

private:
  void update(const StateMessage & message);
  rclcpp::Node & node_;
  const ControllerConfig & config_;
  ControllerState state_;
  rclcpp_action::Client<NativeAction>::SharedPtr client_;
  rclcpp::Subscription<StateMessage>::SharedPtr subscriber_;
  std::shared_ptr<ControllerExecution> outstanding_;
};
}  // namespace hardware_manager

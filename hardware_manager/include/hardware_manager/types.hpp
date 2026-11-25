#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include <hardware_manager/action/execute.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace hardware_manager
{
using Action = action::Execute;
using Result = Action::Result;
using Feedback = Action::Feedback;
using MotionRequest = Action::Goal;
using Steady = std::chrono::steady_clock;
using Values = std::vector<double>;
using Names = std::vector<std::string>;
using Resources = std::set<std::string>;

// TODO(DESIGN_REVIEW): Review the wire codes and the shared execution model.
enum class Component : uint8_t {Head = 1, Torso, LeftArm, RightArm};
enum class LimitPolicy {Strict, Clamp};
enum class JointType {Revolute, Prismatic};

struct MotionError : std::runtime_error
{
  MotionError(uint8_t code, const std::string & message)
  : std::runtime_error(message), code(code) {}
  uint8_t code;
};

struct JointConfig
{
  JointType type;
  double lower, upper, velocity;
  std::optional<double> acceleration;
  double tolerance;
};

struct ControllerConfig
{
  std::string action, state_topic;
  Names joints;
};

struct ComponentConfig
{
  std::string controller;
  Names joints;
  LimitPolicy policy;
  double min_duration;
};

struct ManagerConfig
{
  std::string action_name, hardware_file, motions_file;
  double server_timeout, goal_response_timeout, state_timeout, cancel_timeout;
  double execution_margin, execution_margin_factor, queue_timeout, feedback_rate;
};

using ComponentMotion = std::map<Component, Values>;
struct MotionWaypoint
{
  double time;
  ComponentMotion positions;
};

struct MotionDefinition
{
  std::string description;
  std::vector<MotionWaypoint> waypoints;
};

struct ResolvedMotion
{
  MotionDefinition definition;
  std::set<Component> components;
  Resources controllers;
};

struct PreparedMotion
{
  std::map<std::string, trajectory_msgs::msg::JointTrajectory> trajectories;
  std::set<Component> components;
  Names joints;
  Values targets;
  Names warnings;
  double duration = 0;
};

inline double elapsed(Steady::time_point since)
{
  return std::chrono::duration<double>(Steady::now() - since).count();
}
}  // namespace hardware_manager

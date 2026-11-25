#include "hardware_manager/trajectory_builder.hpp"
#include <algorithm>
#include <cmath>

namespace hardware_manager
{
namespace
{
constexpr double max_duration = 2147483647.0;

trajectory_msgs::msg::JointTrajectoryPoint point(const Values & values, double time)
{
  trajectory_msgs::msg::JointTrajectoryPoint result;
  result.positions = values;
  result.velocities.resize(values.size(), 0.0);
  result.accelerations.resize(values.size(), 0.0);
  result.time_from_start = rclcpp::Duration::from_seconds(time);
  return result;
}
}  // namespace

TrajectoryBuilder::TrajectoryBuilder(const HardwareConfig & hardware) : hardware_(hardware)
{
  for (const auto & [id, config] : hardware.components) {
    switch (id) {
      case Component::Head:
        handlers_[id] = std::make_unique<HeadHandler>(config);
        break;
      case Component::Torso:
        handlers_[id] = std::make_unique<TorsoHandler>(config);
        break;
      case Component::LeftArm:
      case Component::RightArm:
        handlers_[id] = std::make_unique<ArmHandler>(config);
        break;
    }
  }
}

ResolvedMotion TrajectoryBuilder::resolve(
  const MotionRequest & request, const MotionLibrary & library, Names & warnings) const
{
  if (!std::isfinite(request.duration) || request.duration < 0 || request.duration > max_duration) {
    throw MotionError(Result::INVALID_GOAL, "Invalid duration");
  }
  ResolvedMotion motion;
  if (request.type == MotionRequest::DIRECT && request.component >= MotionRequest::HEAD &&
    request.component <= MotionRequest::RIGHT_ARM && !request.positions.empty() &&
    request.motion_name.empty())
  {
    motion.definition.waypoints.push_back(
      {1.0, {{static_cast<Component>(request.component), request.positions}}});
  } else if (request.type == MotionRequest::PREDEFINED && request.component == MotionRequest::NONE &&
    request.positions.empty() && !request.motion_name.empty())
  {
    motion.definition = library.find(request.motion_name);
  } else {
    throw MotionError(Result::INVALID_GOAL, "Invalid motion fields");
  }

  for (auto & waypoint : motion.definition.waypoints) {
    for (auto & [id, positions] : waypoint.positions) {
      const auto component = hardware_.components.find(id);
      if (component == hardware_.components.end()) {
        throw MotionError(Result::COMPONENT_NOT_AVAILABLE, "Requested component is unavailable");
      }
      handlers_.at(id)->validate(positions, hardware_, warnings);
      motion.components.insert(id);
      motion.controllers.insert(component->second.controller);
    }
  }
  for (const auto & id : motion.controllers) {
    for (const auto & joint : hardware_.controllers.at(id).joints) {
      if (!hardware_.joints.at(joint).acceleration) {
        throw MotionError(Result::COMPONENT_NOT_AVAILABLE,
          "Verified joint acceleration limit required for " + joint);
      }
    }
  }
  return motion;
}

PreparedMotion TrajectoryBuilder::prepare(
  const ResolvedMotion & motion, double duration, const Positions & current,
  const Names & warnings, bool predefined) const
{
  PreparedMotion result;
  result.components = motion.components;
  result.warnings = warnings;
  Positions previous = current;
  std::vector<Positions> targets;
  double min_duration = 0;
  for (const auto id : motion.components) {
    min_duration = std::max(min_duration, hardware_.components.at(id).min_duration);
  }
  for (const auto & name : motion.controllers) {
    for (const auto & joint : hardware_.controllers.at(name).joints) {
      const auto position = previous.find(joint);
      if (position == previous.end() || !std::isfinite(position->second)) {
        throw MotionError(Result::STATE_UNAVAILABLE, "Missing current position for " + joint);
      }
      const auto & limits = hardware_.joints.at(joint);
      if (position->second < limits.lower || position->second > limits.upper) {
        throw MotionError(Result::JOINT_LIMIT_VIOLATION, "Measured position outside limits for " + joint);
      }
    }
  }

  // TODO(DESIGN_REVIEW): Verify rest-to-rest quintic interpolation on native controllers.
  const double nominal = motion.definition.waypoints.back().time;
  double scale = min_duration / nominal;
  double previous_time = 0.0;
  for (const auto & waypoint : motion.definition.waypoints) {
    Positions next = previous;
    double segment_duration = 0.0;
    for (const auto & [id, positions] : waypoint.positions) {
      const auto & joints = hardware_.components.at(id).joints;
      for (size_t i = 0; i < joints.size(); ++i) {
        const auto & joint = joints[i];
        const auto & limits = hardware_.joints.at(joint);
        const double distance = std::abs(positions[i] - previous.at(joint));
        segment_duration = std::max({segment_duration,
          1.875 * distance / limits.velocity,
          std::sqrt((10.0 / std::sqrt(3.0)) * distance / limits.acceleration.value())});
        next.at(joint) = positions[i];
      }
    }
    scale = std::max(scale, std::max(segment_duration, 1e-6) / (waypoint.time - previous_time));
    targets.push_back(next);
    previous = std::move(next);
    previous_time = waypoint.time;
  }
  const double viable = nominal * scale;
  const double requested = duration == 0 && predefined ? nominal : duration;
  result.duration = std::max(requested, viable);
  if (!std::isfinite(result.duration) || result.duration > max_duration) {
    throw MotionError(Result::INVALID_GOAL, "Required duration cannot be represented");
  }
  if (requested > 0 && viable > requested) {
    result.warnings.push_back("duration_adjusted");
  }
  for (const auto & name : motion.controllers) {
    auto & trajectory = result.trajectories[name];
    trajectory.joint_names = hardware_.controllers.at(name).joints;
    auto values = [&](const Positions & positions) {
      Values ordered;
      for (const auto & joint : trajectory.joint_names) {
        ordered.push_back(positions.at(joint));
      }
      return ordered;
    };
    trajectory.points.push_back(point(values(current), 0));
    for (size_t i = 0; i < targets.size(); ++i) {
      const auto ratio = motion.definition.waypoints[i].time / nominal;
      trajectory.points.push_back(point(values(targets[i]), ratio * result.duration));
    }
    result.joints.insert(result.joints.end(), trajectory.joint_names.begin(), trajectory.joint_names.end());
    const auto & final = trajectory.points.back().positions;
    result.targets.insert(result.targets.end(), final.begin(), final.end());
  }
  return result;
}
}  // namespace hardware_manager

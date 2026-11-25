#include "hardware_manager/motion_executor.hpp"

#include <algorithm>
#include <cmath>

namespace hardware_manager
{
namespace
{
const char * reason_name(uint8_t code)
{
  switch (code) {
    case Result::SUCCESS: return "success";
    case Result::SUCCESS_WITH_WARNING: return "success_with_warning";
    case Result::INVALID_GOAL: return "invalid_goal";
    case Result::MOTION_NOT_FOUND: return "motion_not_found";
    case Result::COMPONENT_NOT_AVAILABLE: return "component_not_available";
    case Result::CONTROLLER_NOT_AVAILABLE: return "controller_not_available";
    case Result::JOINT_LIMIT_VIOLATION: return "joint_limit_violation";
    case Result::STATE_UNAVAILABLE: return "state_unavailable";
    case Result::CONTROL_FAILED: return "control_failed";
    case Result::RESOURCE_BUSY: return "resource_busy";
    case Result::TIMEOUT: return "timeout";
    case Result::CANCELED: return "canceled";
    case Result::PREEMPTED: return "preempted";
    default: return "internal_error";
  }
}
}  // namespace

MotionExecutor::MotionExecutor(
  rclcpp::Node & node, const ManagerConfig & config, const HardwareConfig & hardware,
  const MotionLibrary & library, const rclcpp::CallbackGroup::SharedPtr & group)
: node_(node), config_(config), hardware_(hardware), library_(library), builder_(hardware)
{
  for (const auto & [id, controller] : hardware.controllers) {
    controllers_[id] = std::make_unique<TrajectoryController>(node, controller, group);
  }
}

void MotionExecutor::accept(const std::shared_ptr<GoalHandle> & handle)
{
  auto goal = std::make_shared<GoalContext>();
  goal->uuid = handle->get_goal_id();
  goal->handle = handle;
  RCLCPP_INFO(node_.get_logger(), "Received goal %s", rclcpp_action::to_string(goal->uuid).c_str());
  transition(*goal, GoalState::Resolving, "Resolving motion");
  try {
    const auto & request = *handle->get_goal();
    goal->motion = builder_.resolve(request, library_, goal->result.warnings);
    if (!goal->result.warnings.empty()) {
      RCLCPP_WARN(node_.get_logger(), "Joint positions clamped for goal %s",
        rclcpp_action::to_string(goal->uuid).c_str());
      transition(*goal, GoalState::Resolving, "Joint positions clamped");
    }
    const auto conflicts = scheduler_.conflicts(*goal, goals_);
    if (std::any_of(conflicts.begin(), conflicts.end(),
      [](const auto & other) {return other->state != GoalState::Queued;}))
    {
      // Validate against live conditions before interfering with an active motion.
      for (const auto & name : goal->motion.controllers) {
        if (!controllers_.at(name)->ready()) {
          throw MotionError(Result::CONTROLLER_NOT_AVAILABLE, "Action server unavailable: " + name);
        }
      }
      build(*goal);
    }
    for (const auto & other : conflicts) {
      if (other->handle->is_canceling()) {
        interrupt(*other, Result::CANCELED, "Client requested cancellation");
      } else {
        interrupt(*other, Result::PREEMPTED, "Replaced by a goal for the same component");
      }
    }
    goals_.push_back(goal);
    transition(*goal, GoalState::Queued, "Waiting for controller resources");
    if (!scheduler_.can_start(*goal, goals_)) {
      RCLCPP_INFO(node_.get_logger(), "Queued goal %s", rclcpp_action::to_string(goal->uuid).c_str());
    }
  } catch (const MotionError & error) {
    finish(*goal, error.code, error.what());
  } catch (const std::exception & error) {
    finish(*goal, Result::INTERNAL_ERROR, error.what());
  }
}

void MotionExecutor::tick()
{
  for (const auto & goal : goals_) {
    if (goal->state == GoalState::Finished) {
      continue;
    }
    try {
      if (goal->handle->is_canceling()) {
        interrupt(*goal, Result::CANCELED, "Client requested cancellation");
      } else if (stopping_) {
        interrupt(*goal, Result::CONTROL_FAILED, "Hardware manager is shutting down");
      }
      step(*goal);
      if (goal->state != GoalState::Finished &&
        elapsed(goal->last_feedback) >= 1.0 / config_.feedback_rate)
      {
        feedback(*goal);
      }
    } catch (const MotionError & error) {
      interrupt(*goal, error.code, error.what());
    } catch (const std::exception & error) {
      interrupt(*goal, Result::INTERNAL_ERROR, error.what());
    }
  }
  goals_.erase(std::remove_if(goals_.begin(), goals_.end(), [](const auto & goal) {
      return goal->state == GoalState::Finished;
    }), goals_.end());
}

void MotionExecutor::step(GoalContext & goal)
{
  switch (goal.state) {
    case GoalState::Queued:
      if (elapsed(goal.accepted) >= config_.queue_timeout) {
        finish(goal, Result::RESOURCE_BUSY, "Controller queue timeout");
      } else if (scheduler_.can_start(goal, goals_)) {
        transition(goal, GoalState::Preparing, "Checking action servers and joint state");
        prepare(goal);
      }
      break;
    case GoalState::Preparing:
      prepare(goal);
      break;
    case GoalState::Executing:
      monitor(goal);
      break;
    case GoalState::Verifying:
      verify(goal);
      break;
    case GoalState::Cancelling:
    {
      const bool terminated = std::all_of(goal.native.begin(), goal.native.end(),
        [](const auto & item) {return item.second->terminal();});
      if (terminated) {
        finish(goal, *goal.interruption, goal.status);
      } else if (elapsed(goal.phase_start) >= config_.cancel_timeout) {
        for (const auto & [id, execution] : goal.native) {
          if (!execution->terminal()) {
            execution->state = NativeState::Unknown;
            RCLCPP_ERROR(node_.get_logger(), "Controller %s UNKNOWN: native termination unconfirmed",
              id.c_str());
          }
        }
        const uint8_t code = *goal.interruption == Result::CANCELED ?
          Result::CONTROL_FAILED : *goal.interruption;
        finish(goal, code, goal.status + "; native termination unconfirmed");
      }
      break;
    }
    default:
      break;
  }
}

PreparedMotion MotionExecutor::build(const GoalContext & goal) const
{
  Positions positions;
  for (const auto & id : goal.motion.controllers) {
    const auto * sample = controllers_.at(id)->state(config_.state_timeout);
    if (!sample) {
      throw MotionError(Result::STATE_UNAVAILABLE, "No recent joint state for " + id);
    }
    for (const auto & joint : hardware_.controllers.at(id).joints) {
      positions.emplace(joint, sample->positions.at(joint));
    }
  }
  const auto & request = *goal.handle->get_goal();
  return builder_.prepare(goal.motion, request.duration, positions, goal.result.warnings,
    request.type == MotionRequest::PREDEFINED);
}

void MotionExecutor::prepare(GoalContext & goal)
{
  for (const auto & id : goal.motion.controllers) {
    const auto & controller = *controllers_.at(id);
    if (!controller.available()) {
      throw MotionError(Result::CONTROLLER_NOT_AVAILABLE, "Unconfirmed previous trajectory on " + id);
    }
    if (!controller.ready()) {
      if (elapsed(goal.phase_start) >= config_.server_timeout) {
        throw MotionError(Result::CONTROLLER_NOT_AVAILABLE, "Action server unavailable: " + id);
      }
      return;
    }
  }
  goal.prepared = build(goal);
  if (std::find(goal.prepared.warnings.begin(), goal.prepared.warnings.end(), "duration_adjusted") !=
    goal.prepared.warnings.end())
  {
    goal.result.warnings = goal.prepared.warnings;
    RCLCPP_WARN(node_.get_logger(), "Duration adjusted to %.3f s", goal.prepared.duration);
    transition(goal, GoalState::Preparing, "Duration adjusted to " +
      std::to_string(goal.prepared.duration) + " s");
  }
  goal.dispatched_at = node_.now();
  // TODO(DESIGN_REVIEW): Measure native start skew and required lead time under DDS load.
  const bool synchronized = goal.prepared.trajectories.size() > 1;
  goal.trajectory_start = synchronized ?
    *goal.dispatched_at + rclcpp::Duration::from_seconds(0.05) : *goal.dispatched_at;
  for (const auto & [id, trajectory] : goal.prepared.trajectories) {
    NativeAction::Goal native;
    native.trajectory = trajectory;
    if (synchronized) {
      native.trajectory.header.stamp = *goal.trajectory_start;
    }
    for (const auto & name : trajectory.joint_names) {
      control_msgs::msg::JointTolerance tolerance;
      tolerance.name = name;
      tolerance.position = hardware_.joints.at(name).tolerance;
      native.goal_tolerance.push_back(tolerance);
    }
    const auto execution = controllers_.at(id)->send(native);
    goal.native.emplace(id, execution);
    if (execution->state == NativeState::Unknown) {
      interrupt(goal, Result::CONTROL_FAILED, "Native goal dispatch failed on " + id);
      return;
    }
    const auto & targets = trajectory.points.back().positions;
    goal.result.joint_names.insert(
      goal.result.joint_names.end(), trajectory.joint_names.begin(), trajectory.joint_names.end());
    goal.result.target_positions.insert(
      goal.result.target_positions.end(), targets.begin(), targets.end());
  }
  RCLCPP_INFO(node_.get_logger(), "Started goal %s (%.3f s)",
    rclcpp_action::to_string(goal.uuid).c_str(), goal.prepared.duration);
  transition(goal, GoalState::Executing, "Executing motion");
}

void MotionExecutor::monitor(GoalContext & goal)
{
  bool succeeded = true;
  double completion_time = 0;
  for (const auto & [id, native] : goal.native) {
    switch (native->state) {
      case NativeState::Succeeded:
        completion_time = std::max(completion_time,
          (*native->terminal_time - *goal.trajectory_start).seconds());
        continue;
      case NativeState::Pending:
        if (elapsed(native->sent) >= config_.goal_response_timeout) {
          interrupt(goal, Result::CONTROLLER_NOT_AVAILABLE, "Native acceptance timeout on " + id);
          return;
        }
        break;
      case NativeState::Active:
        break;
      default:
        interrupt(goal, Result::CONTROL_FAILED, "Native trajectory failed on " + id +
          (native->failure.empty() ? "" : " (" + native->failure + ")"));
        return;
    }
    succeeded = false;
  }
  // TODO(DESIGN_REVIEW): Check ROS time under pauses/jumps and tune execution margin.
  const double time = (node_.now() - *goal.trajectory_start).seconds();
  goal.progress = std::max(goal.progress, static_cast<float>(
    std::clamp(time / goal.prepared.duration, 0.0, 1.0)));
  const double deadline = goal.prepared.duration +
    std::max(config_.execution_margin, config_.execution_margin_factor * goal.prepared.duration);
  if ((succeeded ? completion_time : time) >= deadline) {
    interrupt(goal, Result::TIMEOUT, "Execution timeout");
  } else if (succeeded) {
    transition(goal, GoalState::Verifying, "Verifying final joint positions");
    verify(goal);
  }
}

void MotionExecutor::verify(GoalContext & goal)
{
  const auto positions = samples(goal, false, true);
  bool within_tolerance = !positions.empty();
  for (size_t i = 0; i < positions.size(); ++i) {
    const auto & joint = hardware_.joints.at(goal.prepared.joints[i]);
    within_tolerance &= std::abs(goal.prepared.targets[i] - positions[i]) <= joint.tolerance;
  }
  if (within_tolerance) {
    finish(goal, goal.result.warnings.empty() ? Result::SUCCESS : Result::SUCCESS_WITH_WARNING,
      "Motion completed");
  } else if (elapsed(goal.phase_start) >= config_.state_timeout) {
    finish(goal, positions.empty() ? Result::STATE_UNAVAILABLE : Result::CONTROL_FAILED,
      positions.empty() ? "Final joint state unavailable" : "Final position outside tolerance");
  }
}

Values MotionExecutor::samples(const GoalContext & goal, bool error, bool after_dispatch) const
{
  Values values;
  for (const auto & [id, execution] : goal.native) {
    if (!execution->submitted) {
      continue;
    }
    const auto & trajectory = goal.prepared.trajectories.at(id);
    const auto * state = controllers_.at(id)->state(config_.state_timeout);
    if (!state || (after_dispatch && state->received <= execution->sent)) {
      return {};
    }
    const auto & source = error ? state->errors : state->positions;
    for (const auto & joint : trajectory.joint_names) {
      const auto found = source.find(joint);
      if (found == source.end()) {
        return {};
      }
      values.push_back(found->second);
    }
  }
  return values;
}

void MotionExecutor::transition(GoalContext & goal, GoalState state, const std::string & status)
{
  goal.state = state;
  goal.phase_start = Steady::now();
  goal.status = status;
  feedback(goal);
}

void MotionExecutor::interrupt(GoalContext & goal, uint8_t code, const std::string & reason)
{
  if (goal.state == GoalState::Finished || goal.interruption) {
    return;
  }
  goal.interruption = code;
  if (goal.native.empty()) {
    finish(goal, code, reason);
    return;
  }
  RCLCPP_INFO(node_.get_logger(), "Stopping goal %s: %s",
    rclcpp_action::to_string(goal.uuid).c_str(), reason.c_str());
  transition(goal, GoalState::Cancelling, reason);
  for (const auto & [id, execution] : goal.native) {
    controllers_.at(id)->cancel(execution);
  }
}

void MotionExecutor::feedback(GoalContext & goal)
{
  if (!goal.handle->is_executing() && !goal.handle->is_canceling()) {
    return;
  }
  auto message = std::make_shared<Feedback>();
  message->status_code = static_cast<uint8_t>(goal.state);
  message->status_message = goal.status;
  message->progress = goal.progress;
  if (!goal.result.joint_names.empty()) {
    message->joint_names = goal.result.joint_names;
    message->current_positions = samples(goal, false);
    message->position_errors = samples(goal, true);
  }
  goal.handle->publish_feedback(message);
  goal.last_feedback = Steady::now();
}

void MotionExecutor::finish(GoalContext & goal, uint8_t code, const std::string & reason)
{
  auto & result = goal.result;
  result.result_code = code;
  result.reason = reason_name(code);
  result.result_message = reason;
  result.planned_duration = goal.prepared.duration;
  if (goal.dispatched_at) {
    result.execution_time = std::max(0.0, (node_.now() - *goal.dispatched_at).seconds());
  }
  if (!result.joint_names.empty()) {
    result.final_positions = samples(goal, false, true);
    for (size_t i = 0; i < result.final_positions.size(); ++i) {
      result.position_errors.push_back(result.target_positions[i] - result.final_positions[i]);
    }
  }
  const bool success = code == Result::SUCCESS || code == Result::SUCCESS_WITH_WARNING;
  if (code == Result::CONTROL_FAILED || code == Result::CONTROLLER_NOT_AVAILABLE ||
    code == Result::TIMEOUT || code == Result::INTERNAL_ERROR || code == Result::RESOURCE_BUSY)
  {
    RCLCPP_ERROR(node_.get_logger(), "Goal %s: %s (%s)", rclcpp_action::to_string(goal.uuid).c_str(),
      result.reason.c_str(), reason.c_str());
  } else {
    RCLCPP_INFO(node_.get_logger(), "Goal %s: %s", rclcpp_action::to_string(goal.uuid).c_str(),
      result.reason.c_str());
  }
  transition(goal, GoalState::Finished, reason);
  const auto message = std::make_shared<Result>(result);
  if (code == Result::CANCELED && goal.handle->is_canceling()) {
    goal.handle->canceled(message);
  } else if (success) {
    goal.handle->succeed(message);
  } else {
    goal.handle->abort(message);
  }
}

void MotionExecutor::stop() {stopping_ = true;}

bool MotionExecutor::drained() const
{
  return goals_.empty() && std::all_of(controllers_.begin(), controllers_.end(),
    [](const auto & item) {return item.second->available();});
}
}  // namespace hardware_manager

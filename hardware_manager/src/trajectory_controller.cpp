#include "hardware_manager/trajectory_controller.hpp"
#include <cmath>
#include <type_traits>
#include <utility>

namespace hardware_manager
{
namespace
{
template<typename T, typename = void>
struct HasLegacyState : std::false_type {};

template<typename T>
struct HasLegacyState<T, std::void_t<decltype(std::declval<T>().actual),
    decltype(std::declval<T>().desired)>> : std::true_type {};

template<typename T, typename = void>
struct HasModernState : std::false_type {};

template<typename T>
struct HasModernState<T, std::void_t<decltype(std::declval<T>().feedback),
    decltype(std::declval<T>().reference)>> : std::true_type {};

template<typename T>
const trajectory_msgs::msg::JointTrajectoryPoint & measured(const T & message)
{
  if constexpr (HasModernState<T>::value) {
    if constexpr (!HasLegacyState<T>::value) {
      return message.feedback;
    } else if (!message.feedback.positions.empty()) {
      return message.feedback;
    }
  }
  if constexpr (HasLegacyState<T>::value) {
    return message.actual;
  }
}

template<typename T>
const trajectory_msgs::msg::JointTrajectoryPoint & reference(const T & message)
{
  if constexpr (HasModernState<T>::value) {
    if constexpr (!HasLegacyState<T>::value) {
      return message.reference;
    } else if (!message.feedback.positions.empty()) {
      return message.reference;
    }
  }
  if constexpr (HasLegacyState<T>::value) {
    return message.desired;
  }
}
}  // namespace

bool ControllerExecution::terminal() const
{
  return state == NativeState::Rejected || state == NativeState::Succeeded ||
         state == NativeState::Failed || state == NativeState::Canceled;
}

TrajectoryController::TrajectoryController(
  rclcpp::Node & node, const ControllerConfig & config,
  const rclcpp::CallbackGroup::SharedPtr & group)
: node_(node), config_(config)
{
  client_ = rclcpp_action::create_client<NativeAction>(&node, config.action, group);
  rclcpp::SubscriptionOptions options;
  options.callback_group = group;
  subscriber_ = node.create_subscription<StateMessage>(
    config.state_topic, rclcpp::SensorDataQoS().keep_last(1),
    [this](const StateMessage & message) {update(message);}, options);
}

void TrajectoryController::update(const StateMessage & message)
{
  ControllerState next;
  const auto & actual = measured(message).positions;
  const auto & desired = reference(message).positions;
  if (actual.size() != message.joint_names.size()) {
    state_ = {};
    return;
  }
  try {
    next.stamp = rclcpp::Time(message.header.stamp, node_.get_clock()->get_clock_type());
  } catch (const std::runtime_error &) {
    state_ = {};
    return;
  }
  for (size_t i = 0; i < actual.size(); ++i) {
    const auto & name = message.joint_names[i];
    if (!std::isfinite(actual[i]) || !next.positions.emplace(name, actual[i]).second) {
      state_ = {};
      return;
    }
    if (desired.size() == actual.size() && std::isfinite(desired[i] - actual[i])) {
      next.errors.emplace(name, desired[i] - actual[i]);
    } else if (message.error.positions.size() == actual.size() &&
      std::isfinite(message.error.positions[i]))
    {
      next.errors.emplace(name, message.error.positions[i]);
    }
  }
  for (const auto & name : config_.joints) {
    if (!next.positions.count(name)) {
      state_ = {};
      return;
    }
  }
  next.received = Steady::now();
  state_ = std::move(next);
}

const ControllerState * TrajectoryController::state(double max_age) const
{
  if (state_.positions.empty() || elapsed(state_.received) > max_age) {
    return nullptr;
  }
  if (state_.stamp.nanoseconds() != 0) {
    const auto age = (node_.now() - state_.stamp).seconds();
    if (age < 0 || age > max_age) {
      return nullptr;
    }
  }
  return &state_;
}

bool TrajectoryController::ready() const {return client_->action_server_is_ready();}
bool TrajectoryController::available() const
{
  return !outstanding_ || outstanding_->terminal();
}

std::shared_ptr<ControllerExecution> TrajectoryController::send(const NativeAction::Goal & goal)
{
  auto execution = std::make_shared<ControllerExecution>();
  outstanding_ = execution;
  rclcpp_action::Client<NativeAction>::SendGoalOptions options;
  const std::weak_ptr<ControllerExecution> weak = execution;
  options.goal_response_callback = [this, weak](NativeHandle::SharedPtr handle) {
      const auto execution = weak.lock();
      if (!execution) {
        return;
      }
      execution->handle = std::move(handle);
      execution->state = execution->handle ? NativeState::Active : NativeState::Rejected;
      if (execution->cancel_requested) {
        cancel(execution);
      }
    };
  options.result_callback = [this, weak](const NativeHandle::WrappedResult & result) {
      const auto execution = weak.lock();
      if (!execution) {
        return;
      }
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          execution->state = result.result && result.result->error_code == NativeAction::Result::SUCCESSFUL ?
            NativeState::Succeeded : NativeState::Failed;
          break;
        case rclcpp_action::ResultCode::ABORTED:
          execution->state = NativeState::Failed;
          break;
        case rclcpp_action::ResultCode::CANCELED:
          execution->state = NativeState::Canceled;
          break;
        default:
          execution->state = NativeState::Unknown;
          break;
      }
      if (execution->terminal()) {
        execution->terminal_time = node_.now();
      }
      if (result.result) {
        execution->failure = result.result->error_string;
      }
    };
  try {
    client_->async_send_goal(goal, options);
    execution->submitted = true;
  } catch (const std::exception & error) {
    execution->state = NativeState::Unknown;
    execution->failure = error.what();
  }
  return execution;
}

void TrajectoryController::cancel(const std::shared_ptr<ControllerExecution> & execution)
{
  execution->cancel_requested = true;
  if (!execution->handle || execution->terminal() || execution->cancel_sent) {
    return;
  }
  execution->cancel_sent = true;
  try {
    // Only the native terminal result releases the controller.
    client_->async_cancel_goal(execution->handle);
  } catch (const std::exception & error) {
    RCLCPP_WARN(node_.get_logger(), "Native cancel request failed for %s: %s",
      config_.action.c_str(), error.what());
  }
}
}  // namespace hardware_manager

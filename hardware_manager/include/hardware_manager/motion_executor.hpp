#pragma once
#include "hardware_manager/motion_scheduler.hpp"
#include "hardware_manager/trajectory_controller.hpp"

namespace hardware_manager
{
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

enum class GoalState : uint8_t
{
  Idle = Feedback::IDLE, Resolving = Feedback::RESOLVING, Queued = Feedback::QUEUED,
  Preparing = Feedback::PREPARING, Executing = Feedback::EXECUTING,
  Verifying = Feedback::VERIFYING, Cancelling = Feedback::CANCELLING,
  Finished = Feedback::FINISHED
};

struct GoalContext
{
  rclcpp_action::GoalUUID uuid;
  std::shared_ptr<GoalHandle> handle;
  GoalState state = GoalState::Idle;
  ResolvedMotion motion;
  PreparedMotion prepared;
  std::map<std::string, std::shared_ptr<ControllerExecution>> native;
  Result result;
  Steady::time_point accepted = Steady::now(), phase_start = accepted, last_feedback{};
  std::optional<rclcpp::Time> dispatched_at, trajectory_start;
  std::optional<uint8_t> interruption;
  std::string status;
  float progress = 0;
};

class MotionExecutor
{
public:
  MotionExecutor(rclcpp::Node & node, const ManagerConfig & config,
    const HardwareConfig & hardware, const MotionLibrary & library,
    const rclcpp::CallbackGroup::SharedPtr & group);
  void accept(const std::shared_ptr<GoalHandle> & handle);
  void tick();
  void stop();
  bool stopping() const {return stopping_;}
  bool drained() const;

private:
  PreparedMotion build(const GoalContext & goal) const;
  void step(GoalContext & goal);
  void prepare(GoalContext & goal);
  void monitor(GoalContext & goal);
  void verify(GoalContext & goal);
  void transition(GoalContext & goal, GoalState state, const std::string & status);
  void interrupt(GoalContext & goal, uint8_t code, const std::string & reason);
  void finish(GoalContext & goal, uint8_t code, const std::string & reason);
  void feedback(GoalContext & goal);
  Values samples(const GoalContext & goal, bool error, bool after_dispatch = false) const;

  rclcpp::Node & node_;
  const ManagerConfig & config_;
  const HardwareConfig & hardware_;
  const MotionLibrary & library_;
  TrajectoryBuilder builder_;
  MotionScheduler scheduler_;
  MotionScheduler::Goals goals_;
  std::map<std::string, std::unique_ptr<TrajectoryController>> controllers_;
  bool stopping_ = false;
};
}  // namespace hardware_manager

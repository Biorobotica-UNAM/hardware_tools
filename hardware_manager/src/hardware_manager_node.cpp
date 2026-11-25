#include "hardware_manager/motion_executor.hpp"

#include <algorithm>
#include <atomic>
#include <csignal>

namespace
{
static_assert(std::atomic<bool>::is_always_lock_free);
std::atomic<bool> shutdown_requested{false};

void request_shutdown(int)
{
  shutdown_requested.store(true, std::memory_order_relaxed);
}
}  // namespace

namespace hardware_manager
{
class HardwareManagerNode final : public rclcpp::Node
{
public:
  explicit HardwareManagerNode(rclcpp::Executor & ros_executor)
  : Node("hardware_manager"),
    config_(HardwareConfig::parameters(*this)),
    hardware_(HardwareConfig::load(config_.hardware_file)),
    library_(config_.motions_file),
    group_(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
    motions_(*this, config_, hardware_, library_, group_)
  {
    // TODO(DESIGN_REVIEW): Accept then abort invalid requests to return a typed result.
    // ACCEPT_AND_EXECUTE permits feedback while waiting for scheduler resources.
    server_ = rclcpp_action::create_server<Action>(
      this, config_.action_name,
      [](const rclcpp_action::GoalUUID &, MotionRequest::ConstSharedPtr) {
        return shutdown_requested.load(std::memory_order_relaxed) ? rclcpp_action::GoalResponse::REJECT :
               rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle> &) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const std::shared_ptr<GoalHandle> & goal) {motions_.accept(goal);},
      rcl_action_server_get_default_options(), group_);

    // TODO(DESIGN_REVIEW): Serialize manager callbacks in one group. Each goal advances
    // independently and native action operations return without waiting for responses.
    const auto tick = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(std::min(0.01, 1.0 / config_.feedback_rate)));
    timer_ = create_wall_timer(tick, [this, &ros_executor]() {
        if (shutdown_requested.load(std::memory_order_relaxed) && !motions_.stopping()) {
          stop_started_ = Steady::now();
          motions_.stop();
        }
        motions_.tick();
        if (motions_.stopping() && (motions_.drained() ||
          elapsed(stop_started_) > config_.cancel_timeout + 0.1))
        {
          if (!motions_.drained()) {
            RCLCPP_WARN(get_logger(), "Shutdown with unconfirmed native termination");
          }
          ros_executor.cancel();
        }
      }, group_);
    for (const auto & [joint, limits] : hardware_.joints) {
      if (!limits.acceleration) {
        RCLCPP_WARN(get_logger(), "Motion disabled for controllers using %s: missing joint acceleration limit",
          joint.c_str());
      }
    }
    RCLCPP_INFO(get_logger(), "Ready on %s with %zu controllers",
      config_.action_name.c_str(), hardware_.controllers.size());
  }

private:
  ManagerConfig config_;
  HardwareConfig hardware_;
  MotionLibrary library_;
  rclcpp::CallbackGroup::SharedPtr group_;
  MotionExecutor motions_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
  Steady::time_point stop_started_;
};
}  // namespace hardware_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_shutdown);
  std::signal(SIGTERM, request_shutdown);
  int result = 0;
  try {
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    auto node = std::make_shared<hardware_manager::HardwareManagerNode>(executor);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("hardware_manager"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}

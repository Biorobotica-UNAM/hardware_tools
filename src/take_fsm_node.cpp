#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "tmc_control_msgs/action/gripper_apply_effort.hpp"

using namespace std::chrono_literals;

class TakeFSMNode : public rclcpp::Node
{
public:
  using Trigger = std_srvs::srv::Trigger;
  using GripperApplyEffort = tmc_control_msgs::action::GripperApplyEffort;

  TakeFSMNode()
  : Node("take_fsm_node"), busy_(false)
  {
    torso_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/hardware/torso/goal_pose", 10);

    arm_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/hardware/arm/goal_pose", 10);

    srv_ = this->create_service<Trigger>(
      "/tasks/take_fsm",
      std::bind(
        &TakeFSMNode::handle_take_fsm,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Service ready: /tasks/take_fsm");
  }

private:
  enum class State
  {
    STEP_1,
    STEP_2,
    STEP_3,
    STEP_4,
    STEP_5,
    STEP_6,
    DONE,
    ERROR
  };

  void handle_take_fsm(
    const std::shared_ptr<Trigger::Request> /*request*/,
    std::shared_ptr<Trigger::Response> response)
  {
    if (busy_) {
      response->success = false;
      response->message = "take_fsm is already running";
      return;
    }

    busy_ = true;
    bool success = execute_fsm();
    busy_ = false;

    response->success = success;
    response->message = success ? "take_fsm finished successfully"
                                : "take_fsm failed";
  }

  bool execute_fsm()
  {
    State state = State::STEP_1;
    bool motion_completed = true;

    while (rclcpp::ok()) {
      switch (state) {
        case State::STEP_1:
          RCLCPP_INFO(this->get_logger(), "[TAKE_FSM] Step 1: torso -> 0.4");
          publish_torso(0.4);
          rclcpp::sleep_for(2s);
          state = State::STEP_2;
          break;

        case State::STEP_2:
          RCLCPP_INFO(this->get_logger(), "[TAKE_FSM] Step 2: arm -> [-1.8, 0, -1.4, 0]");
          publish_arm({-1.8, 0.0, -1.4, 0.0});
          rclcpp::sleep_for(2s);
          state = State::STEP_3;
          break;

        case State::STEP_3:
          RCLCPP_INFO(this->get_logger(), "[TAKE_FSM] Step 3: torso -> 0.06");
          publish_torso(0.06);
          rclcpp::sleep_for(2s);
          state = State::STEP_4;
          break;

        case State::STEP_4:
          RCLCPP_INFO(this->get_logger(), "[TAKE_FSM] Step 4: close gripper");
          motion_completed = close_gripper();
          state = motion_completed ? State::STEP_5 : State::ERROR;
          break;

        case State::STEP_5:
          RCLCPP_INFO(this->get_logger(), "[TAKE_FSM] Step 5: torso -> 0.2");
          publish_torso(0.2);
          rclcpp::sleep_for(2s);
          state = State::STEP_6;
          break;

        case State::STEP_6:
          RCLCPP_INFO(this->get_logger(), "[TAKE_FSM] Step 6: arm -> [0, -1.57, -1.57, 0]");
          publish_arm({0.0, -1.57, -1.57, 0.0});
          rclcpp::sleep_for(2s);
          state = State::DONE;
          break;

        case State::DONE:
          RCLCPP_INFO(this->get_logger(), "[TAKE_FSM] Finished successfully");
          return true;

        case State::ERROR:
          RCLCPP_ERROR(this->get_logger(), "[TAKE_FSM] Failed during execution");
          return false;
      }
    }

    return false;
  }

  void publish_torso(double value)
  {
    std_msgs::msg::Float64 msg;
    msg.data = value;
    torso_pub_->publish(msg);
  }

  void publish_arm(const std::vector<double> &values)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = values;
    arm_pub_->publish(msg);
  }

  bool close_gripper()
  {
    bool motion_completed = false;

    rclcpp::sleep_for(3s);
    RCLCPP_INFO(this->get_logger(), "TakeFSM: Sending Grasp effort -0.1 to gripper...");

    auto self = this->shared_from_this();
    auto client = rclcpp_action::create_client<GripperApplyEffort>(
      self, "/gripper_controller/grasp");

    if (!client->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(this->get_logger(), "Action server /gripper_controller/grasp not available");
      motion_completed = false;
    } else {
      auto goal_msg = GripperApplyEffort::Goal();
      goal_msg.effort = -0.1;
      goal_msg.do_control_stop = true;

      auto send_goal_options =
        rclcpp_action::Client<GripperApplyEffort>::SendGoalOptions();

      auto goal_handle_future = client->async_send_goal(goal_msg, send_goal_options);

      if (rclcpp::spin_until_future_complete(self, goal_handle_future) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed while sending gripper goal");
        motion_completed = false;
      } else {
        auto goal_handle = goal_handle_future.get();

        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Gripper goal was rejected");
          motion_completed = false;
        } else {
          auto result_future = client->async_get_result(goal_handle);

          if (rclcpp::spin_until_future_complete(self, result_future) ==
              rclcpp::FutureReturnCode::SUCCESS) {
            auto wrapped_result = result_future.get();

            if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
              motion_completed = true;
              RCLCPP_INFO(this->get_logger(), "Grasp effort applied successfully.");
            } else {
              RCLCPP_ERROR(this->get_logger(), "Gripper action finished but not successfully.");
              motion_completed = false;
            }
          } else {
            RCLCPP_ERROR(this->get_logger(), "Failed while waiting for gripper result");
            motion_completed = false;
          }
        }
      }
    }

    return motion_completed;
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr torso_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_pub_;
  rclcpp::Service<Trigger>::SharedPtr srv_;

  std::atomic_bool busy_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TakeFSMNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
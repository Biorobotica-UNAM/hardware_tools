#pragma once
#include "hardware_manager/types.hpp"
#include <memory>

namespace hardware_manager
{
struct GoalContext;

class MotionScheduler
{
public:
  using Goals = std::vector<std::shared_ptr<GoalContext>>;
  Goals conflicts(const GoalContext & goal, const Goals & goals) const;
  bool can_start(const GoalContext & goal, const Goals & goals) const;
};
}  // namespace hardware_manager

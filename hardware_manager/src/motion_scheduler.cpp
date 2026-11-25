#include "hardware_manager/motion_executor.hpp"

namespace hardware_manager
{
namespace
{
template<typename Set>
bool overlaps(const Set & first, const Set & second)
{
  for (const auto & item : first) {
    if (second.count(item)) {
      return true;
    }
  }
  return false;
}
}  // namespace

// TODO(DESIGN_REVIEW): Review FIFO and whole-motion preemption for logical conflicts.
MotionScheduler::Goals MotionScheduler::conflicts(const GoalContext & goal, const Goals & goals) const
{
  Goals conflicts;
  for (const auto & other : goals) {
    if (other->state != GoalState::Finished &&
      overlaps(goal.motion.components, other->motion.components))
    {
      conflicts.push_back(other);
    }
  }
  return conflicts;
}

bool MotionScheduler::can_start(const GoalContext & goal, const Goals & goals) const
{
  bool earlier = true;
  for (const auto & other : goals) {
    if (other->uuid == goal.uuid) {
      earlier = false;
      continue;
    }
    const auto state = other->state;
    const bool owns = state == GoalState::Preparing || state == GoalState::Executing ||
      state == GoalState::Verifying || state == GoalState::Cancelling;
    if ((owns || (earlier && state == GoalState::Queued)) &&
      overlaps(goal.motion.controllers, other->motion.controllers))
    {
      return false;
    }
  }
  return true;
}
}  // namespace hardware_manager

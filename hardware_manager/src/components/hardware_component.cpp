#include "hardware_manager/components/hardware_component.hpp"
#include <algorithm>
#include <cmath>

namespace hardware_manager
{
void HardwareComponent::validate(
  Values & positions, const HardwareConfig & hardware, Names & warnings) const
{
  if (positions.size() != config_.joints.size()) {
    throw MotionError(Result::INVALID_GOAL, "Incorrect component position count");
  }
  for (size_t i = 0; i < positions.size(); ++i) {
    const auto & name = config_.joints[i];
    const auto & joint = hardware.joints.at(name);
    auto & position = positions[i];
    if (!std::isfinite(position)) {
      throw MotionError(Result::INVALID_GOAL, "Non-finite position for " + name);
    }
    if (position < joint.lower || position > joint.upper) {
      if (config_.policy == LimitPolicy::Strict) {
        throw MotionError(Result::JOINT_LIMIT_VIOLATION, "Position outside limits for " + name);
      }
      position = std::clamp(position, joint.lower, joint.upper);
      if (std::find(warnings.begin(), warnings.end(), "position_clamped") == warnings.end()) {
        warnings.push_back("position_clamped");
      }
    }
  }
}
}  // namespace hardware_manager

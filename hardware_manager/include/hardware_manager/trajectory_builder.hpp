#pragma once
#include "hardware_manager/components/hardware_component.hpp"
#include "hardware_manager/motion_library.hpp"
#include <memory>

namespace hardware_manager
{
using Positions = std::map<std::string, double>;

class TrajectoryBuilder
{
public:
  explicit TrajectoryBuilder(const HardwareConfig & hardware);
  ResolvedMotion resolve(const MotionRequest & request, const MotionLibrary & library,
    Names & warnings) const;
  PreparedMotion prepare(const ResolvedMotion & motion, double duration,
    const Positions & current, const Names & warnings, bool predefined = false) const;

private:
  const HardwareConfig & hardware_;
  std::map<Component, std::unique_ptr<HardwareComponent>> handlers_;
};
}  // namespace hardware_manager

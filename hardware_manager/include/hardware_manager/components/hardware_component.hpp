#pragma once
#include "hardware_manager/hardware_config.hpp"

namespace hardware_manager
{
// TODO(DESIGN_REVIEW): Keep component-specific hooks independent of ROS controllers.
class HardwareComponent
{
public:
  explicit HardwareComponent(const ComponentConfig & config) : config_(config) {}
  virtual ~HardwareComponent() = default;
  void validate(Values & positions, const HardwareConfig & hardware, Names & warnings) const;

private:
  const ComponentConfig & config_;
};

class HeadHandler final : public HardwareComponent
{
  using HardwareComponent::HardwareComponent;
};

class TorsoHandler final : public HardwareComponent
{
  using HardwareComponent::HardwareComponent;
};

class ArmHandler final : public HardwareComponent
{
  using HardwareComponent::HardwareComponent;
};
}  // namespace hardware_manager

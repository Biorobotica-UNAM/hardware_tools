#pragma once
#include "hardware_manager/hardware_config.hpp"
#include <unordered_map>

namespace hardware_manager
{
class MotionLibrary
{
public:
  explicit MotionLibrary(const std::string & path);
  const MotionDefinition & find(const std::string & name) const;

private:
  std::unordered_map<std::string, MotionDefinition> motions_;
};
}  // namespace hardware_manager

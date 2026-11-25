#include "hardware_manager/motion_library.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>

namespace hardware_manager
{
MotionLibrary::MotionLibrary(const std::string & path)
{
  const auto root = YAML::LoadFile(path);
  if (!root["motions"].IsMap()) {
    throw std::runtime_error("Expected a motions mapping in " + path);
  }
  for (const auto & entry : root["motions"]) {
    const auto name = entry.first.as<std::string>();
    MotionDefinition motion;
    motion.description = entry.second["description"].as<std::string>("");
    if (!entry.second["waypoints"].IsSequence()) {
      throw std::runtime_error("Expected a waypoint sequence for " + name);
    }
    double previous = 0;
    for (const auto & data : entry.second["waypoints"]) {
      MotionWaypoint waypoint{data["time"].as<double>(), {}};
      if (!std::isfinite(waypoint.time) || waypoint.time <= previous ||
        waypoint.time > 2147483647.0 || !data["positions"].IsMap())
      {
        throw std::runtime_error("Invalid waypoint in " + name);
      }
      previous = waypoint.time;
      for (const auto & component : data["positions"]) {
        const auto id = component_id(component.first.as<std::string>());
        auto values = component.second.as<Values>();
        if (values.empty() || !std::all_of(values.begin(), values.end(),
          [](double value) {return std::isfinite(value);}) ||
          !waypoint.positions.emplace(id, std::move(values)).second)
        {
          throw std::runtime_error("Invalid component positions in " + name);
        }
      }
      if (waypoint.positions.empty()) {
        throw std::runtime_error("Empty waypoint in " + name);
      }
      motion.waypoints.push_back(std::move(waypoint));
    }
    if (name.empty() || motion.waypoints.empty() ||
      !motions_.emplace(name, std::move(motion)).second)
    {
      throw std::runtime_error("Empty or duplicate motion " + name);
    }
  }
}

const MotionDefinition & MotionLibrary::find(const std::string & name) const
{
  const auto found = motions_.find(name);
  if (found == motions_.end()) {
    throw MotionError(Result::MOTION_NOT_FOUND, "Unknown motion: " + name);
  }
  return found->second;
}
}  // namespace hardware_manager

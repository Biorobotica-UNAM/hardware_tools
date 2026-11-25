#pragma once

#include "hardware_manager/types.hpp"
#include <rclcpp/rclcpp.hpp>

namespace hardware_manager
{
struct HardwareConfig
{
  std::map<std::string, JointConfig> joints;
  std::map<std::string, ControllerConfig> controllers;
  std::map<Component, ComponentConfig> components;

  static HardwareConfig load(const std::string & path);
  static ManagerConfig parameters(rclcpp::Node & node);
};

Component component_id(const std::string & name);
}  // namespace hardware_manager

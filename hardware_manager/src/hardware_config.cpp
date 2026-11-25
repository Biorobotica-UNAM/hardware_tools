#include "hardware_manager/hardware_config.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <filesystem>

namespace hardware_manager
{
namespace
{
void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

double positive(double value, const std::string & name)
{
  require(std::isfinite(value) && value > 0, "Invalid positive value for " + name);
  return value;
}
}  // namespace

Component component_id(const std::string & name)
{
  static const std::map<std::string, Component> ids{
    {"HEAD", Component::Head}, {"TORSO", Component::Torso},
    {"LEFT_ARM", Component::LeftArm}, {"RIGHT_ARM", Component::RightArm}};
  return ids.at(name);
}

ManagerConfig HardwareConfig::parameters(rclcpp::Node & node)
{
  // TODO(DESIGN_REVIEW): Review startup parameters and timeout defaults.
  const auto share = ament_index_cpp::get_package_share_directory("hardware_manager");
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  auto file = [&](const char * name, const char * fallback) {
    const std::filesystem::path path = node.declare_parameter<std::string>(name, fallback, descriptor);
    require(!path.empty(), std::string("Empty configuration path for ") + name);
    return (share / path).string();
  };
  auto time = [&](const char * name, double fallback) {
    return positive(node.declare_parameter<double>(name, fallback, descriptor), name);
  };
  auto margin = [&](const char * name, double fallback) {
    const auto value = node.declare_parameter<double>(name, fallback, descriptor);
    require(std::isfinite(value) && value >= 0, std::string("Invalid margin ") + name);
    return value;
  };
  ManagerConfig config{
    node.declare_parameter<std::string>("action_name", "/hardware/execute", descriptor),
    file("hardware_file", "config/takeshi/hardware.yaml"),
    file("motions_file", "config/takeshi/motions.yaml"),
    time("server_timeout", 2.0), time("goal_response_timeout", 2.0),
    time("state_timeout", 0.5), time("cancel_timeout", 2.0),
    margin("execution_margin", 1.0), margin("execution_margin_factor", 0.5),
    time("queue_timeout", 30.0), time("feedback_rate", 10.0)};
  require(!config.action_name.empty(), "Empty action name");
  require(config.feedback_rate <= 1000.0, "Feedback rate exceeds 1000 Hz");
  return config;
}

HardwareConfig HardwareConfig::load(const std::string & path)
{
  const auto yaml = YAML::LoadFile(path);
  HardwareConfig config;
  Resources actions, topics, controlled, assigned;
  for (const auto & entry : yaml["joints"]) {
    const auto name = entry.first.as<std::string>();
    const auto data = entry.second;
    const auto type = data["type"].as<std::string>();
    require(type == "revolute" || type == "prismatic", "Invalid joint type for " + name);
    JointConfig joint{
      type == "revolute" ? JointType::Revolute : JointType::Prismatic,
      data["lower"].as<double>(), data["upper"].as<double>(),
      positive(data["velocity"].as<double>(), name), std::nullopt,
      positive(data["tolerance"].as<double>(), name)};
    require(std::isfinite(joint.lower) && std::isfinite(joint.upper) &&
      joint.lower < joint.upper, "Invalid position limits for " + name);
    if (data["acceleration"] && !data["acceleration"].IsNull()) {
      joint.acceleration = positive(data["acceleration"].as<double>(), name);
    }
    require(!name.empty() && config.joints.emplace(name, joint).second,
      "Empty or duplicate joint " + name);
  }
  for (const auto & entry : yaml["controllers"]) {
    const auto name = entry.first.as<std::string>();
    const auto data = entry.second;
    ControllerConfig controller{
      data["action"].as<std::string>(), data["state_topic"].as<std::string>(),
      data["joints"].as<Names>()};
    require(!name.empty() && !controller.action.empty() && !controller.state_topic.empty() &&
      !controller.joints.empty(), "Incomplete controller " + name);
    require(actions.insert(controller.action).second && topics.insert(controller.state_topic).second,
      "Duplicate controller endpoint for " + name);
    for (const auto & joint : controller.joints) {
      require(config.joints.count(joint) && controlled.insert(joint).second,
        "Unknown or multiply controlled joint " + joint);
    }
    require(config.controllers.emplace(name, std::move(controller)).second,
      "Duplicate controller " + name);
  }
  std::set<Component> declared;
  for (const auto & entry : yaml["components"]) {
    const auto name = entry.first.as<std::string>();
    const auto component = component_id(name);
    const auto data = entry.second;
    require(declared.insert(component).second, "Duplicate component " + name);
    const auto policy = data["policy"].as<std::string>();
    require(policy == "STRICT" || policy == "CLAMP", "Invalid limit policy for " + name);
    if (!data["available"].as<bool>(true)) {
      continue;
    }
    ComponentConfig definition{
      data["controller"].as<std::string>(), data["joints"].as<Names>(),
      policy == "STRICT" ? LimitPolicy::Strict : LimitPolicy::Clamp,
      positive(data["min_duration"].as<double>(), name)};
    const auto & joints = config.controllers.at(definition.controller).joints;
    require(!definition.joints.empty(), "Empty component " + name);
    for (const auto & joint : definition.joints) {
      require(std::find(joints.begin(), joints.end(), joint) != joints.end() &&
        assigned.insert(joint).second, "Invalid component assignment for " + joint);
    }
    config.components.emplace(component, std::move(definition));
  }
  require(!assigned.empty() && assigned == controlled && controlled.size() == config.joints.size(),
    "Each configured joint must belong to one controller and one available component");
  return config;
}
}  // namespace hardware_manager

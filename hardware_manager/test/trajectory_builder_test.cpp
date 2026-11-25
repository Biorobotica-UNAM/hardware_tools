#include <cmath>
#include <gtest/gtest.h>
#include "hardware_manager/trajectory_builder.hpp"

using namespace hardware_manager;

TEST(TrajectoryBuilder, ScalesAllSegmentsAndHoldsSharedJoints)
{
  HardwareConfig hardware;
  hardware.joints["lift"] = {JointType::Prismatic, 0, 1, 0.15, 0.15, 0.005};
  hardware.joints["arm"] = {JointType::Revolute, -2, 0, 1.0, 1.0, 0.01};
  hardware.components[Component::Torso] = {"arm", {"lift"}, LimitPolicy::Strict, 0.1};
  hardware.controllers["arm"] = {"arm_action", "arm_state", {"lift", "arm"}};
  const ResolvedMotion motion{
    {"", {{1.0, {{Component::Torso, {0.4}}}}, {3.0, {{Component::Torso, {0.2}}}}}},
    {Component::Torso}, {"arm"}};
  const auto result = TrajectoryBuilder(hardware).prepare(
    motion, 0.01, {{"lift", 0.2}, {"arm", -0.5}}, {});
  ASSERT_EQ(result.warnings, Names({"duration_adjusted"}));
  ASSERT_EQ(result.joints, Names({"lift", "arm"}));
  EXPECT_EQ(result.targets, Values({0.2, -0.5}));
  const auto & points = result.trajectories.at("arm").points;
  ASSERT_EQ(points.size(), 3u);
  EXPECT_DOUBLE_EQ(points.front().positions[0], 0.2);
  EXPECT_DOUBLE_EQ(points[1].positions[1], -0.5);
  EXPECT_NEAR(rclcpp::Duration(points[1].time_from_start).seconds(), result.duration / 3, 1e-8);
  EXPECT_NEAR(result.duration, 3 * std::sqrt(10.0 * 0.2 / (std::sqrt(3.0) * 0.15)), 1e-8);
}

TEST(TrajectoryBuilder, KeepsFeasibleStoredDuration)
{
  HardwareConfig hardware;
  hardware.joints["pan"] = {JointType::Revolute, -1, 1, 1, 1, 0.05};
  hardware.components[Component::Head] = {"head", {"pan"}, LimitPolicy::Clamp, 0.1};
  hardware.controllers["head"] = {"head_action", "head_state", {"pan"}};
  const ResolvedMotion motion{
    {"", {{3.0, {{Component::Head, {0.0}}}}}}, {Component::Head}, {"head"}};
  const auto result = TrajectoryBuilder(hardware).prepare(
    motion, 0, {{"pan", 0}}, {}, true);
  EXPECT_DOUBLE_EQ(result.duration, 3.0);
  EXPECT_TRUE(result.warnings.empty());
}

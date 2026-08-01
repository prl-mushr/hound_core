#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/node.hpp>

#include "hound_core/low_level_controller.hpp"

namespace hound_core
{

class MavlinkBridge;

struct LlControllerEntry
{
  std::unique_ptr<LowLevelController> controller;
  /** Called once at startup to wire vehicle-specific ROS subscriptions/pubs. */
  std::function<void (rclcpp::Node &)> setup_subscriptions;
};

using LlControllerFactory =
  std::function<LlControllerEntry(rclcpp::Node &, MavlinkBridge &)>;

class LlControllerRegistry
{
public:
  static void register_factory(const std::string & name, LlControllerFactory factory);
  static LlControllerEntry create(
    const std::string & name, rclcpp::Node & node, MavlinkBridge & bridge);

private:
  static std::unordered_map<std::string, LlControllerFactory> & factories();
};

#define HOUND_REGISTER_LL_CONTROLLER(name, fn) \
  namespace { \
  const bool _registered_##fn = \
    (hound_core::LlControllerRegistry::register_factory(name, fn), true); \
  }

}  // namespace hound_core

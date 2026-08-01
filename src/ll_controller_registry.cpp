#include "hound_core/ll_controller_registry.hpp"

#include <stdexcept>

namespace hound_core
{

std::unordered_map<std::string, LlControllerFactory> & LlControllerRegistry::factories()
{
  static std::unordered_map<std::string, LlControllerFactory> map;
  return map;
}

void LlControllerRegistry::register_factory(
  const std::string & name, LlControllerFactory factory)
{
  factories()[name] = std::move(factory);
}

LlControllerEntry LlControllerRegistry::create(
  const std::string & name, rclcpp::Node & node, MavlinkBridge & bridge)
{
  const auto it = factories().find(name);
  if (it == factories().end()) {
    throw std::runtime_error(
      "Unknown ll_controller '" + name +
      "'. Register a factory with HOUND_REGISTER_LL_CONTROLLER.");
  }
  return it->second(node, bridge);
}

}  // namespace hound_core

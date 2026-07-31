#include "hound_core/hound_fcu_control_modular_node.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hound_core::HoundFcuControlModularNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

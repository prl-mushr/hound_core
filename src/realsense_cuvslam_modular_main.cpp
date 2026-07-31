#include "hound_core/realsense_cuvslam_modular_node.hpp"

#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<hound_core::RealsenseCuvslamModularNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "[realsense_cuvslam_modular_node] fatal: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

#include <algorithm>
#include <memory>

#include "hound_core/ll_controller_node.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<hound_core::LlControllerNode>();
  const int threads = std::max(2, static_cast<int>(node->get_parameter("executor_threads").as_int()));
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), threads);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

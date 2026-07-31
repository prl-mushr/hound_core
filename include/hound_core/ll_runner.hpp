#pragma once

#include <atomic>
#include <functional>
#include <thread>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/logger.hpp>

#include "hound_core/fcu_slots.hpp"
#include "hound_core/ll_controller.hpp"

namespace hound_core
{

/** IMU-paced LL worker (logic copied from HoundFcuControlNode::ll_worker). */
class LlRunner
{
public:
  using SendManualFn = std::function<void (const ManualControlCmd &)>;
  using DiagCallback = std::function<void (const diagnostic_msgs::msg::DiagnosticArray &)>;

  explicit LlRunner(rclcpp::Logger logger);
  ~LlRunner();

  LlRunner(const LlRunner &) = delete;
  LlRunner & operator=(const LlRunner &) = delete;

  /**
   * @param bus shared slots (IMU, RC, state)
   * @param ll controller (also updated from ROS for VESC / auto in the node)
   * @param send_manual typically MavlinkBridge::send_manual_control
   * @param ll_cpu CPU affinity (-1 to skip)
   * @param diag_cb optional diagnostics publisher hook
   */
  void start(
    FcuBus & bus, LlController & ll, SendManualFn send_manual, int ll_cpu = 3,
    DiagCallback diag_cb = {});
  void stop();

private:
  void worker();

  rclcpp::Logger logger_;
  FcuBus * bus_{nullptr};
  LlController * ll_{nullptr};
  SendManualFn send_manual_;
  DiagCallback diag_cb_;
  int ll_cpu_{-1};
  std::atomic<bool> running_{false};
  std::thread thread_;
};

}  // namespace hound_core

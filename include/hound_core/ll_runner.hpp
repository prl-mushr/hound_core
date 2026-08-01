#pragma once

#include <functional>
#include <memory>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/logger.hpp>

#include "hound_core/fcu_slots.hpp"
#include "hound_core/imu_paced_worker.hpp"
#include "hound_core/low_level_controller.hpp"

namespace hound_core
{

/** Generic IMU-paced LL worker over any LowLevelController. */
class LlRunner
{
public:
  using DiagCallback = std::function<void (const diagnostic_msgs::msg::DiagnosticArray &)>;

  explicit LlRunner(rclcpp::Logger logger);
  ~LlRunner();

  LlRunner(const LlRunner &) = delete;
  LlRunner & operator=(const LlRunner &) = delete;

  void start(
    FcuBus & bus, LowLevelController & controller, int ll_cpu = 3,
    DiagCallback diag_cb = {});
  void stop();

private:
  void loop(FcuBus & bus, std::atomic<bool> & running);

  rclcpp::Logger logger_;
  LowLevelController * controller_{nullptr};
  DiagCallback diag_cb_;
  ImuPacedWorker worker_;
};

}  // namespace hound_core

#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

#include "hound_core/fcu_slots.hpp"
#include "vesc_driver/vesc_interface.h"
#include "vesc_driver/vesc_packet.h"

namespace hound_core
{

/**
 * Wall-clock VESC telemetry poller: requestState @ telemetry_hz → bus.vesc.
 * Telemetry-only (no motor/servo commands).
 */
class VescRunner
{
public:
  struct Config
  {
    std::string port{"/dev/ttyACM0"};
    double telemetry_hz{200.0};
  };

  explicit VescRunner(rclcpp::Logger logger);
  ~VescRunner();

  VescRunner(const VescRunner &) = delete;
  VescRunner & operator=(const VescRunner &) = delete;

  /** Opens serial and starts the poll thread. Throws on connect failure. */
  void start(FcuBus & bus, const Config & config);
  void stop();

private:
  void on_packet(const vesc_driver::VescPacketConstPtr & packet);
  void on_error(const std::string & error);
  void loop();

  rclcpp::Logger logger_;
  rclcpp::Clock clock_{RCL_ROS_TIME};
  Config cfg_;
  FcuBus * bus_{nullptr};

  std::unique_ptr<vesc_driver::VescInterface> vesc_;
  std::atomic<bool> running_{false};
  std::thread thread_;

  enum Mode {kInitializing, kOperating};
  Mode mode_{kInitializing};
  int fw_major_{-1};
  int fw_minor_{-1};
};

}  // namespace hound_core

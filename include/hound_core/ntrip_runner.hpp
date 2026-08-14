#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/logger.hpp>

#include "hound_core/fcu_slots.hpp"

namespace hound_core
{

/**
 * Blocking NTRIP client: caster HTTP → complete RTCM3 frames on bus.rtcm.
 * Does not touch the FCU link; the ROS edge timer drains the queue.
 */
class NtripRunner
{
public:
  struct Config
  {
    std::string server;       // host or host:port (default port 2101)
    std::string user;
    std::string password;
    std::string mountpoint;
    /** bus = live GPS (origin fallback) | static | none */
    std::string gga{"bus"};
    std::string static_gga;
    double gga_period_s{10.0};
    double reconnect_s{2.0};
    double origin_lat{0.0};
    double origin_lon{0.0};
    double origin_alt{0.0};
  };

  explicit NtripRunner(rclcpp::Logger logger);
  ~NtripRunner();

  NtripRunner(const NtripRunner &) = delete;
  NtripRunner & operator=(const NtripRunner &) = delete;

  void start(FcuBus & bus, const Config & config);
  void stop();

private:
  void loop();
  bool run_session();
  bool send_gga(int fd);
  std::string build_gga() const;
  void feed_bytes(const uint8_t * data, size_t n);
  void close_fd();

  rclcpp::Logger logger_;
  Config cfg_;
  FcuBus * bus_{nullptr};

  std::atomic<bool> running_{false};
  std::atomic<int> fd_{-1};
  std::thread thread_;

  std::string host_;
  std::string port_;
  std::vector<uint8_t> acc_;
  std::chrono::steady_clock::time_point last_gga_{};
  uint64_t frames_{0};
  uint64_t bytes_{0};
};

}  // namespace hound_core

#pragma once

#include <atomic>
#include <functional>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/logger.hpp>

#include "hound_core/fcu_slots.hpp"
#include "hound_core/imu_paced_worker.hpp"

namespace hound_core
{

/** IMU-paced estimator_ekf worker. */
class EkfRunner
{
public:
  struct Config
  {
    bool enable_baro{true};
    bool enable_mag{true};
    bool enable_gps{true};
    bool fuse_gps{false};
    int ekf_odom_hz{50};
    double mag_max_hz{20.0};
    double baro_max_hz{20.0};
    /** gps_compass | lidar_icp */
    std::string ext_nav_align{"gps_compass"};
    /** Used only for wait-for-ICP log messages. */
    std::string icp_origin_topic{"/localization/icp_origin"};
    double ext_nav_origin_lat{0.0};
    double ext_nav_origin_lon{0.0};
    double ext_nav_origin_hgt{0.0};
    std::string odom_frame{"odom"};
    std::string base_frame{"base_link"};
    int ekf_cpu{2};
  };

  using OdomCallback = std::function<void (const nav_msgs::msg::Odometry &)>;

  explicit EkfRunner(rclcpp::Logger logger);
  ~EkfRunner();

  EkfRunner(const EkfRunner &) = delete;
  EkfRunner & operator=(const EkfRunner &) = delete;

  void start(FcuBus & bus, const Config & config, OdomCallback odom_cb);
  void stop();

  /** Soft reset: next IMU step re-inits at ext_nav_origin and clears VSLAM align. */
  void request_reset();

private:
  void loop(FcuBus & bus, std::atomic<bool> & running);

  rclcpp::Logger logger_;
  Config cfg_;
  OdomCallback odom_cb_;
  ImuPacedWorker worker_;
  std::atomic<bool> reset_requested_{false};
};

}  // namespace hound_core


#pragma once

#include <atomic>
#include <cstdint>
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
    /** Continuous mag fusion after AHRS init. false = mag heading at init only. */
    bool fuse_mag{true};
    bool fuse_gps{false};
    int ekf_odom_hz{50};
    double mag_max_hz{20.0};
    double baro_max_hz{20.0};
    /** Measurement→IMU time delays (ms) for state recall at fusion. */
    uint32_t gps_pos_delay_ms{200};
    uint32_t gps_vel_delay_ms{200};
    uint32_t vslam_pos_delay_ms{100};
    uint32_t vslam_vel_delay_ms{100};
    uint32_t vslam_yaw_delay_ms{100};
    /** Reserved for ICP absolute-pose fusion (align-only today). */
    uint32_t icp_pos_delay_ms{80};
    uint32_t icp_vel_delay_ms{80};
    uint32_t icp_yaw_delay_ms{80};
    uint32_t baro_delay_ms{50};
    uint32_t mag_delay_ms{25};
    /** Baro height measurement sigma (m) → AttPosEKF::BaroSigma / R_hgt. */
    float baro_sigma_m{1.0f};
    /** Mag measurement sigma (scaled field units) → magMeasurementSigma. */
    float mag_sigma{0.5f};
    /**
     * Inflate R_MAG by (1 + gain * sq(|B|/B0 - 1)) when |B| leaves expected
     * earth field (µT). 0 disables interference inflation.
     */
    float mag_interference_gain{0.0f};
    /** gps_compass | lidar_icp — selects init_ref only (IMU_AHRS vs EXT_NAV). */
    std::string ext_nav_align{"gps_compass"};
    /** Used only for wait-for-ICP log messages. */
    std::string icp_origin_topic{"/localization/icp_origin"};
    /**
     * Ext-nav sticky Mahalanobis n-sigma on consecutive raw pose Δ.
     * Fail → retune sticky to EKF and drop that sample. Default 5.
     */
    float ext_nav_sticky_gate_nsigma{5.0f};
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

  /**
   * Hard reset: stop the IMU worker (destroys the in-loop estimator_ekf) and
   * start a fresh thread — equivalent to killing and re-launching the EKF.
   */
  void hard_restart(FcuBus & bus);

private:
  void loop(FcuBus & bus, std::atomic<bool> & running);

  rclcpp::Logger logger_;
  Config cfg_;
  OdomCallback odom_cb_;
  ImuPacedWorker worker_;
};

}  // namespace hound_core


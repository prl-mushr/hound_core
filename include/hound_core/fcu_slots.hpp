#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/time.hpp>

namespace hound_core
{

/** Latest-wins slot (RealSense-style). Writers bump seq; waiters use cv. */
template<typename T>
struct LatestSlot
{
  mutable std::mutex mu;
  std::condition_variable cv;
  bool valid{false};
  uint64_t seq{0};
  T data{};

  void write(const T & v)
  {
    {
      std::lock_guard<std::mutex> lock(mu);
      data = v;
      valid = true;
      ++seq;
    }
    cv.notify_all();
  }

  bool copy_latest(T & out, uint64_t * seq_out = nullptr) const
  {
    std::lock_guard<std::mutex> lock(mu);
    if (!valid) {
      return false;
    }
    out = data;
    if (seq_out) {
      *seq_out = seq;
    }
    return true;
  }

  /** Block until seq advances past `last_seq`. Returns false on shutdown. */
  bool wait_new(uint64_t & last_seq, T & out, const std::atomic<bool> & running)
  {
    std::unique_lock<std::mutex> lock(mu);
    cv.wait(lock, [&] {
        return !running.load(std::memory_order_relaxed) || (valid && seq != last_seq);
      });
    if (!running.load(std::memory_order_relaxed)) {
      return false;
    }
    out = data;
    last_seq = seq;
    return true;
  }
};

struct ImuSample
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  float gx{0}, gy{0}, gz{0};
  float ax{0}, ay{0}, az{0};
  float qw{1}, qx{0}, qy{0}, qz{0};
  bool has_orientation{false};
};

struct MagSample
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  float mx{0}, my{0}, mz{0};  // Tesla (ROS MagneticField units)
};

struct BaroSample
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  float pressure_pa{101325.0f};
  float temperature_c{25.0f};
};

struct GpsSample
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  double lat_deg{0}, lon_deg{0};
  float alt_m{0};
  float eph_m{0}, epv_m{0};
  float vel_m_s{0}, cog_deg{0};
  uint8_t fix_type{0};
  uint8_t satellites_visible{0};
  uint16_t hdop_cdeg{0};  // GPS_RAW_INT eph is already cm; keep raw hAcc if present
};

struct RcSample
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  std::array<float, 18> channels{};
  uint8_t nchan{0};
};

struct FcuStateSample
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  bool connected{false};
  bool armed{false};
  bool guided{false};
  uint8_t system_status{0};
  std::string mode;
};

/** One mission item from MISSION_ITEM_INT (no mavros_msgs). */
struct MissionItem
{
  uint16_t seq{0};
  uint8_t frame{0};
  uint16_t command{0};
  bool is_current{false};
  bool autocontinue{true};
  float param1{0}, param2{0}, param3{0}, param4{0};
  double x_lat{0}, y_long{0}, z_alt{0};
};

struct ExtNavSample
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  float pos_enu[3]{0, 0, 0};
  float vel_enu[3]{0, 0, 0};
  float quat_wxyz[4]{1, 0, 0, 0};  // ENU
  bool has_vel{false};
};

/** Lidar-ICP (or other) absolute pose in map/ENU used to align VSLAM. */
struct IcpOriginSample
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  float pos_enu[3]{0, 0, 0};
  float quat_wxyz[4]{1, 0, 0, 0};
  bool valid{false};
};

struct ApLocalPoseSample
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  float x{0}, y{0}, z{0};
  float vx{0}, vy{0}, vz{0};
  float qw{1}, qx{0}, qy{0}, qz{0};
};

struct ManualControlCmd
{
  float x{0}, y{0}, z{0}, r{0};
  uint16_t buttons{0};
};

/** VESC COMM_GET_VALUES telemetry (in-process poller → LL / ROS edge). */
struct VescSample
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  float speed{0};  // eRPM
  float voltage_input{0};
  float duty_cycle{0};
  float current_input{0};
  float current_motor{0};
  float temperature_pcb{0};
  float charge_drawn{0};
  float charge_regen{0};
  float energy_drawn{0};
  float energy_regen{0};
  float displacement{0};
  float distance_traveled{0};
  int32_t fault_code{0};
};

/** Latest EKF pose/vel for packing control_state (no covariance). */
struct EkfNavSample
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  float pos_enu[3]{0, 0, 0};
  float rpy[3]{0, 0, 0};       // roll, pitch, yaw (ENU / FLU)
  float vel_body[3]{0, 0, 0};  // FLU body linear velocity
};

/**
 * BeamNG / UW_mppi plant state (17):
 *   [0:3]  pos ENU xyz
 *   [3:6]  rpy
 *   [6:9]  vel body xyz
 *   [9:12] accel body xyz
 *   [12:15] gyro body xyz
 *   [15]   steering (rad, +left)
 *   [16]   throttle / measured wheelspeed (m/s)
 */
struct ControlStateSample
{
  static constexpr size_t kDim = 17;
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  std::array<double, kDim> x{};
};

/** MAVLink RC_CHANNELS_OVERRIDE payload (unused channels = 65535 = release). */
struct RcOverrideCmd
{
  static constexpr uint16_t kRelease = 65535;
  static constexpr uint16_t kNeutral = 1500;
  std::array<uint16_t, 18> channels{};

  static RcOverrideCmd all_release()
  {
    RcOverrideCmd cmd;
    cmd.channels.fill(kRelease);
    return cmd;
  }

  static RcOverrideCmd neutral_on(uint8_t ch_a, uint8_t ch_b, uint8_t ch_c)
  {
    RcOverrideCmd cmd = all_release();
    if (ch_a < 18) {cmd.channels[ch_a] = kNeutral;}
    if (ch_b < 18) {cmd.channels[ch_b] = kNeutral;}
    if (ch_c < 18) {cmd.channels[ch_c] = kNeutral;}
    return cmd;
  }
};

/** FIFO of complete RTCM3 frames (not latest-wins — drops break RTK). */
struct RtcmQueue
{
  static constexpr size_t kMaxFrames = 64;

  void push(std::vector<uint8_t> && frame)
  {
    std::lock_guard<std::mutex> lock(mu_);
    if (q_.size() >= kMaxFrames) {
      q_.pop_front();
      ++dropped_;
    }
    q_.push_back(std::move(frame));
  }

  bool pop(std::vector<uint8_t> & out)
  {
    std::lock_guard<std::mutex> lock(mu_);
    if (q_.empty()) {
      return false;
    }
    out = std::move(q_.front());
    q_.pop_front();
    return true;
  }

  uint64_t dropped() const
  {
    std::lock_guard<std::mutex> lock(mu_);
    return dropped_;
  }

private:
  mutable std::mutex mu_;
  std::deque<std::vector<uint8_t>> q_;
  uint64_t dropped_{0};
};

struct FcuBus
{
  LatestSlot<ImuSample> imu;
  LatestSlot<MagSample> mag;
  LatestSlot<BaroSample> baro;
  LatestSlot<GpsSample> gps;
  LatestSlot<RcSample> rc;
  LatestSlot<FcuStateSample> state;
  LatestSlot<ExtNavSample> ext_nav;
  LatestSlot<IcpOriginSample> icp_origin;
  LatestSlot<ApLocalPoseSample> ap_local;
  LatestSlot<ManualControlCmd> manual_cmd;
  LatestSlot<VescSample> vesc;
  LatestSlot<EkfNavSample> ekf_nav;
  LatestSlot<ControlStateSample> control_state;
  RtcmQueue rtcm;
};

}  // namespace hound_core

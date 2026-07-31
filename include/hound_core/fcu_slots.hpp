#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>

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
};

}  // namespace hound_core

#include "hound_core/ekf_runner.hpp"

#include <algorithm>
#include <array>
#include <cmath>

// nav_filter macros collide with tf2 / angles headers.
#ifdef deg2rad
#undef deg2rad
#endif
#ifdef rad2deg
#undef rad2deg
#endif

#include "estimator_ekf.h"
#include "inertial_nav_ros2/frame_conversions.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hound_core
{
namespace
{

inline uint32_t stamp_ms(const rclcpp::Time & t)
{
  return static_cast<uint32_t>(t.nanoseconds() / 1000000ULL);
}

inline float isa_height_m(float pressure_pa, float sea_level_pa = 101325.0f)
{
  return 44330.0f * (1.0f - std::pow(pressure_pa / sea_level_pa, 0.190295f));
}

bool accept_rate_limited(uint32_t stamp_ms_now, uint32_t & last_ms, float max_hz)
{
  if (max_hz <= 0.0f) {
    last_ms = stamp_ms_now;
    return true;
  }
  const uint32_t min_interval = static_cast<uint32_t>(1000.0f / max_hz);
  if (last_ms != 0U && stamp_ms_now >= last_ms && (stamp_ms_now - last_ms) < min_interval) {
    return false;
  }
  last_ms = stamp_ms_now;
  return true;
}

float yaw_from_quat_wxyz(float w, float x, float y, float z)
{
  return std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

void yaw_quat_wxyz(float yaw, float q[4])
{
  const float h = 0.5f * yaw;
  q[0] = std::cos(h);
  q[1] = 0.0f;
  q[2] = 0.0f;
  q[3] = std::sin(h);
}

void quat_multiply_wxyz(const float a[4], const float b[4], float out[4])
{
  out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

void align_ext_nav_enu(ExtNavSample & nav, float yaw_corr, const float t_enu[3])
{
  const float c = std::cos(yaw_corr);
  const float s = std::sin(yaw_corr);
  const float x = nav.pos_enu[0];
  const float y = nav.pos_enu[1];
  nav.pos_enu[0] = c * x - s * y + t_enu[0];
  nav.pos_enu[1] = s * x + c * y + t_enu[1];
  nav.pos_enu[2] = nav.pos_enu[2] + t_enu[2];
  const float vx = nav.vel_enu[0];
  const float vy = nav.vel_enu[1];
  nav.vel_enu[0] = c * vx - s * vy;
  nav.vel_enu[1] = s * vx + c * vy;
  float q_yaw[4];
  yaw_quat_wxyz(yaw_corr, q_yaw);
  float q_out[4];
  quat_multiply_wxyz(q_yaw, nav.quat_wxyz, q_out);
  for (int i = 0; i < 4; ++i) {
    nav.quat_wxyz[i] = q_out[i];
  }
}

void set_symmetric_cov3(
  std::array<double, 36> & cov, std::size_t d0, std::size_t d1, std::size_t d2,
  double c00, double c01, double c02, double c11, double c12, double c22)
{
  cov[d0] = c00;
  cov[d0 + 1] = c01;
  cov[d0 + 2] = c02;
  cov[d1] = c01;
  cov[d1 + 1] = c11;
  cov[d1 + 2] = c12;
  cov[d2] = c02;
  cov[d2 + 1] = c12;
  cov[d2 + 2] = c22;
}

void fill_ekf_odom_covariance(
  const float P[22][22], const float q_ned_wxyz[4], nav_msgs::msg::Odometry & odom)
{
  odom.pose.covariance.fill(0.0);
  odom.twist.covariance.fill(0.0);

  const double pn = std::max(static_cast<double>(P[7][7]), 1e-9);
  const double pe = std::max(static_cast<double>(P[8][8]), 1e-9);
  const double pd = std::max(static_cast<double>(P[9][9]), 1e-9);
  const double pne = static_cast<double>(P[7][8]);
  const double pnd = static_cast<double>(P[7][9]);
  const double ped = static_cast<double>(P[8][9]);
  set_symmetric_cov3(odom.pose.covariance, 0, 6, 12, pe, pne, -ped, pn, -pnd, pd);

  const double vn = std::max(static_cast<double>(P[4][4]), 1e-9);
  const double ve = std::max(static_cast<double>(P[5][5]), 1e-9);
  const double vd = std::max(static_cast<double>(P[6][6]), 1e-9);
  const double vne = static_cast<double>(P[4][5]);
  const double vnd = static_cast<double>(P[4][6]);
  const double ved = static_cast<double>(P[5][6]);

  tf2::Matrix3x3 cov_vel_ned(vn, vne, vnd, vne, ve, ved, vnd, ved, vd);
  const tf2::Matrix3x3 r_ned_frd(
    tf2::Quaternion(q_ned_wxyz[1], q_ned_wxyz[2], q_ned_wxyz[3], q_ned_wxyz[0]));
  tf2::Matrix3x3 cov_vel_frd = r_ned_frd.transpose() * cov_vel_ned * r_ned_frd;
  const tf2::Matrix3x3 r_ros_from_frd(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0);
  tf2::Matrix3x3 cov_twist = r_ros_from_frd * cov_vel_frd * r_ros_from_frd.transpose();
  set_symmetric_cov3(
    odom.twist.covariance, 0, 6, 12,
    cov_twist[0][0], cov_twist[0][1], cov_twist[0][2],
    cov_twist[1][1], cov_twist[1][2], cov_twist[2][2]);

  const double att_var = std::max(
    0.5 * (static_cast<double>(P[1][1]) + static_cast<double>(P[2][2]) +
      static_cast<double>(P[3][3])),
    1e-9);
  odom.pose.covariance[21] = att_var;
  odom.pose.covariance[28] = att_var;
  odom.pose.covariance[35] = att_var;
}

}  // namespace

EkfRunner::EkfRunner(rclcpp::Logger logger)
: logger_(logger)
{
}

EkfRunner::~EkfRunner()
{
  stop();
}

void EkfRunner::start(FcuBus & bus, const Config & config, OdomCallback odom_cb)
{
  stop();
  cfg_ = config;
  odom_cb_ = std::move(odom_cb);
  worker_.start(
    bus, cfg_.ekf_cpu,
    [this](FcuBus & b, std::atomic<bool> & running) {loop(b, running);});
}

void EkfRunner::stop()
{
  worker_.stop();
  odom_cb_ = nullptr;
}

void EkfRunner::request_reset()
{
  reset_requested_.store(true, std::memory_order_relaxed);
  RCLCPP_WARN(
    logger_,
    "EKF reset requested — re-initializing at ext_nav_origin "
    "(lat=%.6f lon=%.6f hgt=%.1f) and clearing VSLAM align",
    cfg_.ext_nav_origin_lat, cfg_.ext_nav_origin_lon, cfg_.ext_nav_origin_hgt);
}

void EkfRunner::loop(FcuBus & bus, std::atomic<bool> & running)
{
  using inertial_nav_ros2::frames::BodyAxes;
  using inertial_nav_ros2::frames::enu_position_to_ned;
  using inertial_nav_ros2::frames::enu_quat_to_ned_frd;
  using inertial_nav_ros2::frames::enu_velocity_to_ned;
  using inertial_nav_ros2::frames::ned_frd_quat_to_enu;
  using inertial_nav_ros2::frames::ned_position_to_enu;
  using inertial_nav_ros2::frames::ned_velocity_to_frd_body;
  using inertial_nav_ros2::frames::ros_body_vector_to_frd;

  const bool align_gps_compass = (cfg_.ext_nav_align == "gps_compass");
  const bool align_lidar_icp = (cfg_.ext_nav_align == "lidar_icp");

  estimator_ekf ekf;
  ekf.set_ext_nav_origin(
    cfg_.ext_nav_origin_lat, cfg_.ext_nav_origin_lon,
    static_cast<float>(cfg_.ext_nav_origin_hgt));

  EkfSourceConfig cfg;
  cfg.init_ref = align_gps_compass ? InitReferenceSource::IMU_AHRS :
    InitReferenceSource::EXT_NAV;
  cfg.ext_nav[0].enabled = true;
  cfg.ext_nav[0].fuse_height = true;
  cfg.ext_nav[0].fuse_yaw = true;
  cfg.ext_nav[0].fuse_vel = false;
  cfg.ext_nav[0].fuse_pos = true;
  cfg.ext_nav[0].pos_delay_ms = 100;
  cfg.ext_nav[0].yaw_delay_ms = 100;
  cfg.mag[0].enabled = cfg_.enable_mag;
  cfg.gps[0].enabled = cfg_.enable_gps && cfg_.fuse_gps;
  cfg.baro_delay_ms = 50;
  ekf.set_ekf_config(cfg);
  ekf.set_baro_enabled(cfg_.enable_baro);
  ekf.set_ext_nav_enabled(0, true);
  ekf.set_mag_enabled(0, cfg_.enable_mag);

  RCLCPP_INFO(
    logger_,
    "EKF worker: align=%s init_ref=%s mag_max_hz=%.1f baro_max_hz=%.1f",
    cfg_.ext_nav_align.c_str(),
    align_gps_compass ? "imu_ahrs+compass" : "ext_nav+icp",
    cfg_.mag_max_hz, cfg_.baro_max_hz);

  uint64_t last_seq = 0;
  ImuSample imu;
  bool have_prev = false;
  uint32_t prev_ms = 0;
  bool filter_reset = true;
  const BodyAxes body_axes = BodyAxes::RosFlu;
  const int pub_every = std::max(1, 200 / std::max(1, cfg_.ekf_odom_hz));
  int imu_ticks_since_pub = 0;
  uint32_t last_mag_ms = 0;
  uint32_t last_baro_ms = 0;

  bool vslam_aligned = false;
  float yaw_corr = 0.0f;
  float t_corr_enu[3] = {0, 0, 0};
  bool logged_waiting_icp = false;

  while (running.load(std::memory_order_relaxed)) {
    if (!bus.imu.wait_new(last_seq, imu, running)) {
      break;
    }
    const uint32_t t_ms = stamp_ms(imu.stamp);
    float imu_dt = 0.005f;
    if (have_prev && t_ms > prev_ms && (t_ms - prev_ms) < 100U) {
      imu_dt = static_cast<float>(t_ms - prev_ms) * 0.001f;
    }
    prev_ms = t_ms;
    have_prev = true;

    float accel_ros[3] = {imu.ax, imu.ay, imu.az};
    float gyro_ros[3] = {imu.gx, imu.gy, imu.gz};
    float accel[3], gyro[3];
    ros_body_vector_to_frd(accel_ros, body_axes, accel);
    ros_body_vector_to_frd(gyro_ros, body_axes, gyro);
    ekf.setIMUData(
      accel, gyro, imu_dt, static_cast<float>(t_ms),
      static_cast<float>(static_cast<uint64_t>(t_ms) * 1000ULL));

    MagSample mag;
    if (cfg_.enable_mag && bus.mag.copy_latest(mag)) {
      const uint32_t m_ms = stamp_ms(mag.stamp);
      if (accept_rate_limited(m_ms, last_mag_ms, static_cast<float>(cfg_.mag_max_hz))) {
        float field_ut[3] = {
          mag.mx * 1.0e6f, mag.my * 1.0e6f, mag.mz * 1.0e6f};
        float field_frd[3];
        ros_body_vector_to_frd(field_ut, body_axes, field_frd);
        float zero[3] = {0, 0, 0};
        ekf.setMagData(field_frd, zero, true, 0);
      }
    }

    BaroSample baro;
    if (cfg_.enable_baro && bus.baro.copy_latest(baro)) {
      const uint32_t b_ms = stamp_ms(baro.stamp);
      if (accept_rate_limited(b_ms, last_baro_ms, static_cast<float>(cfg_.baro_max_hz))) {
        const float h = isa_height_m(baro.pressure_pa);
        ekf.setAirData(0.0f, h, 1.0f, imu_dt, true, false);
      }
    }

    GpsSample gps;
    if (cfg_.enable_gps && cfg.gps[0].enabled && bus.gps.copy_latest(gps) &&
      gps.fix_type >= 3)
    {
      float vel_ned[3] = {0, 0, 0};
      const float course_rad = gps.cog_deg * static_cast<float>(M_PI / 180.0);
      vel_ned[0] = gps.vel_m_s * std::cos(course_rad);
      vel_ned[1] = gps.vel_m_s * std::sin(course_rad);
      ekf.setGPSData(
        gps.lat_deg, gps.lon_deg, gps.alt_m, gps.vel_m_s, course_rad, vel_ned,
        gps.eph_m, gps.epv_m, 0.5f, 0.0f, 0.0f, imu_dt,
        static_cast<float>(gps.fix_type), 0.0f, true);
    }

    ExtNavSample nav;
    bool have_nav = bus.ext_nav.copy_latest(nav);
    if (have_nav && !vslam_aligned) {
      if (align_gps_compass) {
        if (ekf.is_initialized()) {
          const AttPosEKF * core = ekf.ekf_core();
          if (core != nullptr) {
            geometry_msgs::msg::Quaternion q_map;
            float q_ned[4] = {
              core->states[0], core->states[1], core->states[2], core->states[3]};
            ned_frd_quat_to_enu(q_ned, body_axes, q_map);
            const float yaw_map = yaw_from_quat_wxyz(
              static_cast<float>(q_map.w), static_cast<float>(q_map.x),
              static_cast<float>(q_map.y), static_cast<float>(q_map.z));
            const float yaw_v = yaw_from_quat_wxyz(
              nav.quat_wxyz[0], nav.quat_wxyz[1], nav.quat_wxyz[2], nav.quat_wxyz[3]);
            yaw_corr = yaw_map - yaw_v;
            const float c = std::cos(yaw_corr);
            const float s = std::sin(yaw_corr);
            t_corr_enu[0] = -(c * nav.pos_enu[0] - s * nav.pos_enu[1]);
            t_corr_enu[1] = -(s * nav.pos_enu[0] + c * nav.pos_enu[1]);
            t_corr_enu[2] = -nav.pos_enu[2];
            AttPosEKF * mutable_core = ekf.ekf_core();
            if (mutable_core != nullptr) {
              mutable_core->states[7] = 0.0f;
              mutable_core->states[8] = 0.0f;
              mutable_core->states[9] = 0.0f;
              mutable_core->posNE[0] = 0.0f;
              mutable_core->posNE[1] = 0.0f;
              mutable_core->hgtMea = 0.0f;
            }
            vslam_aligned = true;
            RCLCPP_INFO(
              logger_,
              "VSLAM aligned to compass/AHRS map at origin "
              "(yaw_corr=%.2f deg, vslam_xy=(%.2f,%.2f))",
              yaw_corr * 180.0f / static_cast<float>(M_PI),
              nav.pos_enu[0], nav.pos_enu[1]);
          }
        }
      } else if (align_lidar_icp) {
        IcpOriginSample icp;
        if (!bus.icp_origin.copy_latest(icp) || !icp.valid) {
          if (!logged_waiting_icp) {
            RCLCPP_WARN(
              logger_,
              "ext_nav_align=lidar_icp: waiting for PoseStamped on %s",
              cfg_.icp_origin_topic.c_str());
            logged_waiting_icp = true;
          }
        } else {
          const float yaw_map = yaw_from_quat_wxyz(
            icp.quat_wxyz[0], icp.quat_wxyz[1], icp.quat_wxyz[2], icp.quat_wxyz[3]);
          const float yaw_v = yaw_from_quat_wxyz(
            nav.quat_wxyz[0], nav.quat_wxyz[1], nav.quat_wxyz[2], nav.quat_wxyz[3]);
          yaw_corr = yaw_map - yaw_v;
          const float c = std::cos(yaw_corr);
          const float s = std::sin(yaw_corr);
          t_corr_enu[0] = icp.pos_enu[0] - (c * nav.pos_enu[0] - s * nav.pos_enu[1]);
          t_corr_enu[1] = icp.pos_enu[1] - (s * nav.pos_enu[0] + c * nav.pos_enu[1]);
          t_corr_enu[2] = icp.pos_enu[2] - nav.pos_enu[2];
          vslam_aligned = true;
          RCLCPP_INFO(
            logger_,
            "VSLAM aligned to ICP origin (yaw_corr=%.2f deg)",
            yaw_corr * 180.0f / static_cast<float>(M_PI));
        }
      }
    }

    if (have_nav && vslam_aligned) {
      align_ext_nav_enu(nav, yaw_corr, t_corr_enu);
      float pos_ned[3], vel_ned[3], q_ned[4];
      enu_position_to_ned(nav.pos_enu, pos_ned);
      enu_velocity_to_ned(nav.vel_enu, vel_ned);
      geometry_msgs::msg::Quaternion q;
      q.w = nav.quat_wxyz[0];
      q.x = nav.quat_wxyz[1];
      q.y = nav.quat_wxyz[2];
      q.z = nav.quat_wxyz[3];
      enu_quat_to_ned_frd(q, body_axes, q_ned);
      ekf.set_ext_nav_pose(pos_ned, vel_ned, q_ned, 0.1f, 0.1f, 0.2f, true, 0);
    }

    if (reset_requested_.exchange(false, std::memory_order_relaxed)) {
      filter_reset = true;
      vslam_aligned = false;
      yaw_corr = 0.0f;
      t_corr_enu[0] = t_corr_enu[1] = t_corr_enu[2] = 0.0f;
      logged_waiting_icp = false;
    }
    ekf.run_filter(filter_reset);
    filter_reset = false;

    if (!ekf.is_initialized()) {
      continue;
    }
    const AttPosEKF * core = ekf.ekf_core();
    if (core == nullptr) {
      continue;
    }

    const float * st = core->states;
    float pos_enu[3];
    ned_position_to_enu(&st[7], pos_enu);
    float q_wxyz[4] = {st[0], st[1], st[2], st[3]};
    geometry_msgs::msg::Quaternion q_enu;
    ned_frd_quat_to_enu(q_wxyz, body_axes, q_enu);
    float vel_frd[3], vel_ros[3];
    ned_velocity_to_frd_body(&st[4], q_wxyz, vel_frd);
    inertial_nav_ros2::frames::frd_body_vector_to_ros(vel_frd, body_axes, vel_ros);

    EkfNavSample nav_s;
    nav_s.stamp = imu.stamp;
    nav_s.pos_enu[0] = pos_enu[0];
    nav_s.pos_enu[1] = pos_enu[1];
    nav_s.pos_enu[2] = pos_enu[2];
    {
      const float qw = static_cast<float>(q_enu.w);
      const float qx = static_cast<float>(q_enu.x);
      const float qy = static_cast<float>(q_enu.y);
      const float qz = static_cast<float>(q_enu.z);
      nav_s.rpy[0] = std::atan2(
        2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));
      const float sinp = 2.0f * (qw * qy - qz * qx);
      nav_s.rpy[1] = (std::fabs(sinp) >= 1.0f) ?
        std::copysign(static_cast<float>(M_PI) / 2.0f, sinp) : std::asin(sinp);
      nav_s.rpy[2] = std::atan2(
        2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
    }
    nav_s.vel_body[0] = vel_ros[0];
    nav_s.vel_body[1] = vel_ros[1];
    nav_s.vel_body[2] = vel_ros[2];
    bus.ekf_nav.write(nav_s);

    ++imu_ticks_since_pub;
    if (imu_ticks_since_pub < pub_every) {
      continue;
    }
    imu_ticks_since_pub = 0;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = imu.stamp;
    odom.header.frame_id = cfg_.odom_frame;
    odom.child_frame_id = cfg_.base_frame;
    odom.pose.pose.position.x = pos_enu[0];
    odom.pose.pose.position.y = pos_enu[1];
    odom.pose.pose.position.z = pos_enu[2];
    odom.pose.pose.orientation = q_enu;
    odom.twist.twist.linear.x = vel_ros[0];
    odom.twist.twist.linear.y = vel_ros[1];
    odom.twist.twist.linear.z = vel_ros[2];
    fill_ekf_odom_covariance(core->P, q_wxyz, odom);
    if (odom_cb_) {
      odom_cb_(odom);
    }
  }
}

}  // namespace hound_core

#include "hound_core/hound_fcu_control_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <pthread.h>
#include <sched.h>

#include <mavlink/v2.0/common/common.hpp>

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

namespace hound_core
{
namespace
{

constexpr double kMilliRsToRadSec = 1.0e-3;
constexpr double kMilliGToMs2 = 9.80665 / 1000.0;
constexpr double kMilliTToTesla = 1000.0;  // match mavros APM convention
constexpr double kMilliBarToPa = 1.0e2;

inline void frd_to_flu(float x, float y, float z, float & xo, float & yo, float & zo)
{
  xo = x;
  yo = -y;
  zo = -z;
}

inline uint32_t stamp_ms(const rclcpp::Time & t)
{
  return static_cast<uint32_t>(t.nanoseconds() / 1000000ULL);
}

inline float isa_height_m(float pressure_pa, float sea_level_pa = 101325.0f)
{
  return 44330.0f * (1.0f - std::pow(pressure_pa / sea_level_pa, 0.190295f));
}

bool pin_current_thread(int cpu)
{
  if (cpu < 0) {
    return true;
  }
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  return pthread_setaffinity_np(pthread_self(), sizeof(set), &set) == 0;
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

/** Rotate ENU xy by yaw_corr and compose yaw into quat; translate by t_enu. */
void align_ext_nav_enu(
  ExtNavSample & nav, float yaw_corr, const float t_enu[3])
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

void fill_ekf_odom_covariance(const float P[22][22], const float q_ned_wxyz[4],
  nav_msgs::msg::Odometry & odom)
{
  odom.pose.covariance.fill(0.0);
  odom.twist.covariance.fill(0.0);

  const double pn = std::max(static_cast<double>(P[7][7]), 1e-9);
  const double pe = std::max(static_cast<double>(P[8][8]), 1e-9);
  const double pd = std::max(static_cast<double>(P[9][9]), 1e-9);
  const double pne = static_cast<double>(P[7][8]);
  const double pnd = static_cast<double>(P[7][9]);
  const double ped = static_cast<double>(P[8][9]);
  // NED -> ENU pose block
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
  // FRD -> ROS FLU
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

HoundFcuControlNode::HoundFcuControlNode(const rclcpp::NodeOptions & options)
: Node("hound_fcu_control", options)
{
  declare_params();
  for (const auto id : gcs_throttle_msgids_) {
    gcs_throttle_ids_.insert(static_cast<uint32_t>(id));
  }

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("~/imu", rclcpp::SensorDataQoS());
  mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("~/mag", rclcpp::SensorDataQoS());
  baro_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>("~/baro", rclcpp::SensorDataQoS());
  gps_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("~/gps/fix", 10);
  ekf_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(ekf_odom_topic_, 10);
  ap_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/ap/local_odometry", 10);
  mission_pub_ = create_publisher<mavros_msgs::msg::WaypointList>("~/mission/waypoints", 1);
  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/low_level_diagnostics", 1);
  limits_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/control_limits", 1);

  vision_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    vision_odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&HoundFcuControlNode::vision_cb, this, std::placeholders::_1));
  if (ext_nav_align_ == "lidar_icp") {
    icp_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      icp_origin_topic_, 10,
      std::bind(&HoundFcuControlNode::icp_origin_cb, this, std::placeholders::_1));
  }
  vesc_sub_ = create_subscription<vesc_msgs::msg::VescStateStamped>(
    "/sensors/core", rclcpp::SensorDataQoS(),
    std::bind(&HoundFcuControlNode::vesc_cb, this, std::placeholders::_1));
  auto_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    "hound/control", 10,
    std::bind(&HoundFcuControlNode::auto_control_cb, this, std::placeholders::_1));

  open_links();
  running_ = true;
  if (enable_ll_) {
    ll_thread_ = std::thread([this] { ll_worker(); });
  }
  if (enable_ekf_) {
    ekf_thread_ = std::thread([this] { ekf_worker(); });
  }

  const auto edge_period = std::chrono::duration<double>(1.0 / std::max(1.0, ros_publish_hz_));
  edge_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(edge_period),
    std::bind(&HoundFcuControlNode::ros_edge_timer, this));
  hb_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&HoundFcuControlNode::send_heartbeat, this));
  boot_timer_ = create_wall_timer(
    std::chrono::seconds(2),
    [this]() {
      if (fcu_seen_.load() && !boot_done_.exchange(true)) {
        boot_configure();
      }
    });

  RCLCPP_INFO(
    get_logger(),
    "hound_fcu_control up: fcu=%s gcs=%s ekf=%d ll=%d ros_hz=%.1f align=%s ekf_odom_hz=%d",
    fcu_url_.c_str(), gcs_url_.empty() ? "(none)" : gcs_url_.c_str(),
    static_cast<int>(enable_ekf_), static_cast<int>(enable_ll_), ros_publish_hz_,
    ext_nav_align_.c_str(), ekf_odom_hz_);
}

HoundFcuControlNode::~HoundFcuControlNode()
{
  running_ = false;
  bus_.imu.cv.notify_all();
  if (ll_thread_.joinable()) {
    ll_thread_.join();
  }
  if (ekf_thread_.joinable()) {
    ekf_thread_.join();
  }
  if (fcu_) {
    fcu_->close();
  }
  if (gcs_) {
    gcs_->close();
  }
}

void HoundFcuControlNode::declare_params()
{
  fcu_url_ = declare_parameter<std::string>("fcu_url", "/dev/ttyACM1:921600");
  gcs_url_ = declare_parameter<std::string>("gcs_url", "");
  ros_publish_hz_ = declare_parameter<double>("ros_publish_hz", 50.0);
  gcs_block_stream_requests_ = declare_parameter<bool>("gcs_block_stream_requests", true);
  gcs_throttle_hz_ = declare_parameter<double>("gcs_throttle_hz", 10.0);
  gcs_throttle_msgids_ = declare_parameter<std::vector<int64_t>>(
    "gcs_throttle_msgids", {27, 30, 31, 32, 33, 65});
  send_vision_to_fcu_ = declare_parameter<bool>("send_vision_to_fcu", true);
  enable_ekf_ = declare_parameter<bool>("enable_ekf", true);
  enable_ll_ = declare_parameter<bool>("enable_ll", true);
  enable_baro_ = declare_parameter<bool>("enable_baro", true);
  enable_mag_ = declare_parameter<bool>("enable_mag", true);
  enable_gps_ = declare_parameter<bool>("enable_gps", true);
  // When false (default), GPS is parsed/published but not fused — avoids indoor
  // junk fixes yanking the filter off the hardcoded origin before VSLAM aligns.
  fuse_gps_ = declare_parameter<bool>("fuse_gps", false);
  ekf_odom_hz_ = declare_parameter<int>("ekf_odom_hz", 50);
  {
    static constexpr int kAllowed[] = {200, 100, 50, 25, 10};
    bool ok = false;
    for (int a : kAllowed) {
      if (ekf_odom_hz_ == a) {
        ok = true;
        break;
      }
    }
    if (!ok) {
      RCLCPP_WARN(
        get_logger(),
        "ekf_odom_hz=%d not in {200,100,50,25,10}; using 50", ekf_odom_hz_);
      ekf_odom_hz_ = 50;
    }
  }
  mag_max_hz_ = declare_parameter<double>("mag_max_hz", 20.0);
  baro_max_hz_ = declare_parameter<double>("baro_max_hz", 20.0);
  ext_nav_align_ = declare_parameter<std::string>("ext_nav_align", "gps_compass");
  if (ext_nav_align_ != "gps_compass" && ext_nav_align_ != "lidar_icp") {
    RCLCPP_WARN(
      get_logger(), "ext_nav_align=%s unknown; using gps_compass",
      ext_nav_align_.c_str());
    ext_nav_align_ = "gps_compass";
  }
  icp_origin_topic_ = declare_parameter<std::string>(
    "icp_origin_topic", "/localization/icp_origin");
  vision_odom_topic_ = declare_parameter<std::string>(
    "vision_odom_topic", "/visual_slam/tracking/odometry");
  ekf_odom_topic_ = declare_parameter<std::string>("ekf_odom_topic", "ekf/odometry");
  ext_nav_origin_lat_ = declare_parameter<double>("ext_nav_origin.lat", 37.8715);
  ext_nav_origin_lon_ = declare_parameter<double>("ext_nav_origin.lon", -122.2730);
  ext_nav_origin_hgt_ = declare_parameter<double>("ext_nav_origin.hgt", 0.0);
  ekf_cpu_ = declare_parameter<int>("ekf_cpu", 2);
  ll_cpu_ = declare_parameter<int>("ll_cpu", 3);
  system_id_ = static_cast<uint8_t>(declare_parameter<int>("system_id", 255));
  component_id_ = static_cast<uint8_t>(declare_parameter<int>("component_id", 191));
  target_system_ = static_cast<uint8_t>(declare_parameter<int>("target_system", 1));
  target_component_ = static_cast<uint8_t>(declare_parameter<int>("target_component", 1));

  // Flat fcu_params as string-keyed doubles via nested declare isn't portable —
  // accept common SR0_* as explicit params + generic list.
  const int sr0_extra1 = declare_parameter<int>("fcu_params.SR0_EXTRA1", 200);
  const int sr0_raw = declare_parameter<int>("fcu_params.SR0_RAW_SENS", 200);
  fcu_params_["SR0_EXTRA1"] = static_cast<double>(sr0_extra1);
  fcu_params_["SR0_RAW_SENS"] = static_cast<double>(sr0_raw);

  LlController::Params lp;
  lp.erpm_gain = static_cast<float>(declare_parameter("ll.erpm_gain", 3166.6));
  lp.steering_max = static_cast<float>(declare_parameter("ll.steering_max", 0.488));
  lp.wheelbase = static_cast<float>(declare_parameter("ll.wheelbase", 0.29));
  lp.cg_height = static_cast<float>(declare_parameter("ll.cg_height", 0.125));
  lp.wheelspeed_max = static_cast<float>(declare_parameter("ll.wheelspeed_max", 17.0));
  lp.nominal_voltage = static_cast<float>(declare_parameter("ll.nominal_voltage", 14.8));
  lp.motor_kv = static_cast<float>(declare_parameter("ll.motor_kv", 3930.0));
  lp.speed_control_kp = static_cast<float>(declare_parameter("ll.speed_control_kp", 1.0));
  lp.speed_control_ki = static_cast<float>(declare_parameter("ll.speed_control_ki", 1.0));
  lp.safe_mode = declare_parameter("ll.safe_mode", true);
  lp.track_width = static_cast<float>(declare_parameter("ll.track_width", 0.25));
  lp.accel_gain = static_cast<float>(declare_parameter("ll.accel_gain", 1.0));
  lp.roll_gain = static_cast<float>(declare_parameter("ll.roll_gain", 0.33));
  lp.steer_slack = static_cast<float>(declare_parameter("ll.steer_slack", 0.4));
  lp.LPF_tau = static_cast<float>(declare_parameter("ll.LPF_tau", 0.2));
  lp.throttle_delta = static_cast<float>(declare_parameter("ll.throttle_delta", 0.02));
  lp.liftoff_oversteer = declare_parameter("ll.liftoff_oversteer", true);
  lp.control_dt = static_cast<float>(declare_parameter("ll.control_dt", 0.02));
  ll_.set_params(lp);
}

void HoundFcuControlNode::open_links()
{
  try {
    fcu_ = mavconn::MAVConnInterface::open_url(fcu_url_, system_id_, component_id_);
    fcu_->message_received_cb = [this](const mavlink::mavlink_message_t * msg,
        mavconn::Framing framing) {
        on_fcu_message(msg, framing);
      };
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "Failed to open fcu_url=%s: %s", fcu_url_.c_str(), e.what());
    throw;
  }

  if (!gcs_url_.empty()) {
    try {
      gcs_ = mavconn::MAVConnInterface::open_url(gcs_url_, system_id_, component_id_);
      gcs_->message_received_cb = [this](const mavlink::mavlink_message_t * msg,
          mavconn::Framing framing) {
          on_gcs_message(msg, framing);
        };
      RCLCPP_INFO(get_logger(), "GCS bridge open: %s", gcs_url_.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "GCS open failed (%s): %s", gcs_url_.c_str(), e.what());
      gcs_.reset();
    }
  }
}

void HoundFcuControlNode::send_to_fcu(const mavlink::Message & msg)
{
  if (!fcu_) {
    return;
  }
  try {
    fcu_->send_message_ignore_drop(msg);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "FCU send drop: %s", e.what());
  }
}

void HoundFcuControlNode::maybe_forward_to_gcs(const mavlink::mavlink_message_t & msg)
{
  if (!gcs_) {
    return;
  }
  const uint32_t msgid = msg.msgid;
  if (!gcs_throttle_ids_.empty() && gcs_throttle_ids_.count(msgid) && gcs_throttle_hz_ > 0.0) {
    const auto now = std::chrono::steady_clock::now();
    const auto min_dt = std::chrono::duration<double>(1.0 / gcs_throttle_hz_);
    std::lock_guard<std::mutex> lock(gcs_throttle_mu_);
    auto & last = gcs_last_fwd_[msgid];
    if (last.time_since_epoch().count() != 0 && (now - last) < min_dt) {
      return;
    }
    last = now;
  }
  try {
    gcs_->send_message_ignore_drop(&msg);
  } catch (...) {
  }
}

void HoundFcuControlNode::on_fcu_message(
  const mavlink::mavlink_message_t * msg, mavconn::Framing framing)
{
  if (framing != mavconn::Framing::ok || msg == nullptr) {
    return;
  }
  fcu_seen_ = true;
  maybe_forward_to_gcs(*msg);

  switch (msg->msgid) {
    case mavlink::minimal::msg::HEARTBEAT::MSG_ID:
      handle_heartbeat(*msg);
      break;
    case mavlink::common::msg::RAW_IMU::MSG_ID:
      handle_raw_imu(*msg);
      break;
    case mavlink::common::msg::SCALED_IMU::MSG_ID:
      handle_scaled_imu(*msg);
      break;
    case mavlink::common::msg::ATTITUDE_QUATERNION::MSG_ID:
      handle_attitude_quat(*msg);
      break;
    case mavlink::common::msg::SCALED_PRESSURE::MSG_ID:
      handle_scaled_pressure(*msg);
      break;
    case mavlink::common::msg::GPS_RAW_INT::MSG_ID:
      handle_gps_raw(*msg);
      break;
    case mavlink::common::msg::RC_CHANNELS::MSG_ID:
    case mavlink::common::msg::RC_CHANNELS_RAW::MSG_ID:
      handle_rc_channels(*msg);
      break;
    case mavlink::common::msg::LOCAL_POSITION_NED::MSG_ID:
      handle_local_position(*msg);
      break;
    case mavlink::common::msg::MISSION_COUNT::MSG_ID:
      handle_mission_count(*msg);
      break;
    case mavlink::common::msg::MISSION_ITEM_INT::MSG_ID:
      handle_mission_item(*msg);
      break;
    case mavlink::common::msg::PARAM_VALUE::MSG_ID:
      handle_param_value(*msg);
      break;
    default:
      break;
  }
}

void HoundFcuControlNode::on_gcs_message(
  const mavlink::mavlink_message_t * msg, mavconn::Framing framing)
{
  if (framing != mavconn::Framing::ok || msg == nullptr || !fcu_) {
    return;
  }
  if (gcs_block_stream_requests_ &&
    msg->msgid == mavlink::common::msg::REQUEST_DATA_STREAM::MSG_ID)
  {
    return;
  }
  try {
    fcu_->send_message_ignore_drop(msg);
  } catch (...) {
  }
}

void HoundFcuControlNode::handle_heartbeat(const mavlink::mavlink_message_t & msg)
{
  mavlink::minimal::msg::HEARTBEAT hb{};
  mavlink::MsgMap map(&msg);
  hb.deserialize(map);

  FcuStateSample st;
  st.stamp = now();
  st.connected = true;
  st.armed =
    (hb.base_mode &
    static_cast<uint8_t>(mavlink::minimal::MAV_MODE_FLAG::SAFETY_ARMED)) != 0;
  const uint32_t mode = hb.custom_mode;
  // Copter GUIDED=4; Rover/Plane GUIDED=15 — LL still needs RC switch.
  st.guided = st.armed && (mode == 4U || mode == 15U);
  st.system_status = static_cast<uint8_t>(hb.system_status);
  st.mode = std::to_string(mode);
  bus_.state.write(st);
  ll_.update_mode(st.armed, st.guided);
}

void HoundFcuControlNode::handle_raw_imu(const mavlink::mavlink_message_t & msg)
{
  mavlink::common::msg::RAW_IMU raw{};
  mavlink::MsgMap map(&msg);
  raw.deserialize(map);

  const float gx_frd = static_cast<float>(raw.xgyro * kMilliRsToRadSec);
  const float gy_frd = static_cast<float>(raw.ygyro * kMilliRsToRadSec);
  const float gz_frd = static_cast<float>(raw.zgyro * kMilliRsToRadSec);
  const float ax_frd = static_cast<float>(raw.xacc * kMilliGToMs2);
  const float ay_frd = static_cast<float>(raw.yacc * kMilliGToMs2);
  const float az_frd = static_cast<float>(raw.zacc * kMilliGToMs2);

  ImuSample s;
  s.stamp = now();
  frd_to_flu(gx_frd, gy_frd, gz_frd, s.gx, s.gy, s.gz);
  frd_to_flu(ax_frd, ay_frd, az_frd, s.ax, s.ay, s.az);
  {
    std::lock_guard<std::mutex> lock(att_mu_);
    if (att_valid_) {
      s.qw = att_qw_;
      s.qx = att_qx_;
      s.qy = att_qy_;
      s.qz = att_qz_;
      s.has_orientation = true;
    }
  }
  bus_.imu.write(s);

  MagSample m;
  m.stamp = s.stamp;
  float mx, my, mz;
  frd_to_flu(
    static_cast<float>(raw.xmag * kMilliTToTesla),
    static_cast<float>(raw.ymag * kMilliTToTesla),
    static_cast<float>(raw.zmag * kMilliTToTesla),
    mx, my, mz);
  m.mx = mx;
  m.my = my;
  m.mz = mz;
  bus_.mag.write(m);
}

void HoundFcuControlNode::handle_scaled_imu(const mavlink::mavlink_message_t & msg)
{
  // Same scaling path as RAW_IMU on APM
  mavlink::common::msg::SCALED_IMU raw{};
  mavlink::MsgMap map(&msg);
  raw.deserialize(map);

  ImuSample s;
  s.stamp = now();
  frd_to_flu(
    static_cast<float>(raw.xgyro * kMilliRsToRadSec),
    static_cast<float>(raw.ygyro * kMilliRsToRadSec),
    static_cast<float>(raw.zgyro * kMilliRsToRadSec),
    s.gx, s.gy, s.gz);
  frd_to_flu(
    static_cast<float>(raw.xacc * kMilliGToMs2),
    static_cast<float>(raw.yacc * kMilliGToMs2),
    static_cast<float>(raw.zacc * kMilliGToMs2),
    s.ax, s.ay, s.az);
  {
    std::lock_guard<std::mutex> lock(att_mu_);
    if (att_valid_) {
      s.qw = att_qw_;
      s.qx = att_qx_;
      s.qy = att_qy_;
      s.qz = att_qz_;
      s.has_orientation = true;
    }
  }
  bus_.imu.write(s);
}

void HoundFcuControlNode::handle_attitude_quat(const mavlink::mavlink_message_t & msg)
{
  mavlink::common::msg::ATTITUDE_QUATERNION q{};
  mavlink::MsgMap map(&msg);
  q.deserialize(map);
  // MAVLink aircraft FRD -> ROS FLU: same axis flip on vector part
  float qx, qy, qz;
  frd_to_flu(q.q2, q.q3, q.q4, qx, qy, qz);
  std::lock_guard<std::mutex> lock(att_mu_);
  att_qw_ = q.q1;
  att_qx_ = qx;
  att_qy_ = qy;
  att_qz_ = qz;
  att_valid_ = true;
}

void HoundFcuControlNode::handle_scaled_pressure(const mavlink::mavlink_message_t & msg)
{
  mavlink::common::msg::SCALED_PRESSURE p{};
  mavlink::MsgMap map(&msg);
  p.deserialize(map);
  BaroSample b;
  b.stamp = now();
  b.pressure_pa = static_cast<float>(p.press_abs * kMilliBarToPa);
  b.temperature_c = static_cast<float>(p.temperature) / 100.0f;
  bus_.baro.write(b);
}

void HoundFcuControlNode::handle_gps_raw(const mavlink::mavlink_message_t & msg)
{
  mavlink::common::msg::GPS_RAW_INT g{};
  mavlink::MsgMap map(&msg);
  g.deserialize(map);
  GpsSample s;
  s.stamp = now();
  s.lat_deg = g.lat * 1e-7;
  s.lon_deg = g.lon * 1e-7;
  s.alt_m = g.alt * 1e-3f;
  s.eph_m = g.eph * 1e-2f;
  s.epv_m = g.epv * 1e-2f;
  s.vel_m_s = g.vel * 1e-2f;
  s.cog_deg = g.cog * 1e-2f;
  s.fix_type = g.fix_type;
  s.satellites_visible = g.satellites_visible;
  bus_.gps.write(s);
}

void HoundFcuControlNode::handle_rc_channels(const mavlink::mavlink_message_t & msg)
{
  RcSample rc;
  rc.stamp = now();
  if (msg.msgid == mavlink::common::msg::RC_CHANNELS::MSG_ID) {
    mavlink::common::msg::RC_CHANNELS m{};
    mavlink::MsgMap map(&msg);
    m.deserialize(map);
    const uint16_t chans[] = {
      m.chan1_raw, m.chan2_raw, m.chan3_raw, m.chan4_raw, m.chan5_raw, m.chan6_raw,
      m.chan7_raw, m.chan8_raw, m.chan9_raw, m.chan10_raw, m.chan11_raw, m.chan12_raw,
      m.chan13_raw, m.chan14_raw, m.chan15_raw, m.chan16_raw, m.chan17_raw, m.chan18_raw};
    rc.nchan = std::min<uint8_t>(m.chancount, 18);
    for (uint8_t i = 0; i < rc.nchan; ++i) {
      rc.channels[i] = static_cast<float>(chans[i]);
    }
  } else {
    mavlink::common::msg::RC_CHANNELS_RAW m{};
    mavlink::MsgMap map(&msg);
    m.deserialize(map);
    const uint16_t chans[] = {
      m.chan1_raw, m.chan2_raw, m.chan3_raw, m.chan4_raw,
      m.chan5_raw, m.chan6_raw, m.chan7_raw, m.chan8_raw};
    rc.nchan = 8;
    for (uint8_t i = 0; i < 8; ++i) {
      rc.channels[i] = static_cast<float>(chans[i]);
    }
  }
  bus_.rc.write(rc);
  ll_.update_rc(rc.channels.data(), rc.nchan);

  ackermann_msgs::msg::AckermannDriveStamped limits;
  limits.header.stamp = rc.stamp;
  limits.drive.speed = ll_.auto_wheelspeed_limit();
  limits_pub_->publish(limits);
}

void HoundFcuControlNode::handle_local_position(const mavlink::mavlink_message_t & msg)
{
  mavlink::common::msg::LOCAL_POSITION_NED p{};
  mavlink::MsgMap map(&msg);
  p.deserialize(map);
  // NED -> ENU for ROS compare topic
  ApLocalPoseSample s;
  s.stamp = now();
  s.x = p.y;
  s.y = p.x;
  s.z = -p.z;
  s.vx = p.vy;
  s.vy = p.vx;
  s.vz = -p.vz;
  {
    std::lock_guard<std::mutex> lock(att_mu_);
    if (att_valid_) {
      s.qw = att_qw_;
      s.qx = att_qx_;
      s.qy = att_qy_;
      s.qz = att_qz_;
    }
  }
  bus_.ap_local.write(s);
}

void HoundFcuControlNode::handle_mission_count(const mavlink::mavlink_message_t & msg)
{
  mavlink::common::msg::MISSION_COUNT c{};
  mavlink::MsgMap map(&msg);
  c.deserialize(map);
  std::lock_guard<std::mutex> lock(mission_mu_);
  mission_count_ = c.count;
  mission_next_ = 0;
  mission_items_.clear();
  mission_items_.reserve(c.count);
  if (c.count == 0) {
    mission_dirty_ = true;
    return;
  }
  mavlink::common::msg::MISSION_REQUEST_INT req{};
  req.target_system = target_system_;
  req.target_component = target_component_;
  req.seq = 0;
  req.mission_type = 0;
  send_to_fcu(req);
}

void HoundFcuControlNode::handle_mission_item(const mavlink::mavlink_message_t & msg)
{
  mavlink::common::msg::MISSION_ITEM_INT it{};
  mavlink::MsgMap map(&msg);
  it.deserialize(map);

  mavros_msgs::msg::Waypoint wp;
  wp.frame = it.frame;
  wp.command = it.command;
  wp.is_current = it.current;
  wp.autocontinue = it.autocontinue;
  wp.param1 = it.param1;
  wp.param2 = it.param2;
  wp.param3 = it.param3;
  wp.param4 = it.param4;
  wp.x_lat = it.x * 1e-7;
  wp.y_long = it.y * 1e-7;
  wp.z_alt = it.z;

  std::lock_guard<std::mutex> lock(mission_mu_);
  if (it.seq >= mission_items_.size()) {
    mission_items_.resize(it.seq + 1);
  }
  mission_items_[it.seq] = wp;
  mission_next_ = static_cast<uint16_t>(it.seq + 1);
  if (mission_next_ < mission_count_) {
    mavlink::common::msg::MISSION_REQUEST_INT req{};
    req.target_system = target_system_;
    req.target_component = target_component_;
    req.seq = mission_next_;
    req.mission_type = 0;
    send_to_fcu(req);
  } else {
    mission_dirty_ = true;
  }
}

void HoundFcuControlNode::handle_param_value(const mavlink::mavlink_message_t & /*msg*/)
{
  // Ack path reserved for future param verify
}

void HoundFcuControlNode::boot_configure()
{
  RCLCPP_INFO(get_logger(), "FCU seen — pushing stream rates / params / mission pull");
  push_fcu_params();
  request_data_streams();
  request_mission_list();
}

void HoundFcuControlNode::push_fcu_params()
{
  for (const auto & kv : fcu_params_) {
    mavlink::common::msg::PARAM_SET ps{};
    ps.target_system = target_system_;
    ps.target_component = target_component_;
    mavlink::set_string(ps.param_id, kv.first);
    ps.param_type = static_cast<uint8_t>(mavlink::common::MAV_PARAM_TYPE::REAL32);
    ps.param_value = static_cast<float>(kv.second);
    send_to_fcu(ps);
  }
}

void HoundFcuControlNode::request_data_streams()
{
  auto set_interval = [this](uint32_t msgid, float hz) {
      mavlink::common::msg::COMMAND_LONG cmd{};
      cmd.target_system = target_system_;
      cmd.target_component = target_component_;
      cmd.command = static_cast<uint16_t>(mavlink::common::MAV_CMD::SET_MESSAGE_INTERVAL);
      cmd.param1 = static_cast<float>(msgid);
      cmd.param2 = (hz > 0.0f) ? (1.0e6f / hz) : -1.0f;
      send_to_fcu(cmd);
    };
  set_interval(mavlink::common::msg::RAW_IMU::MSG_ID, 200.0f);
  set_interval(mavlink::common::msg::ATTITUDE_QUATERNION::MSG_ID, 200.0f);
  set_interval(mavlink::common::msg::SCALED_PRESSURE::MSG_ID, 50.0f);
  set_interval(mavlink::common::msg::GPS_RAW_INT::MSG_ID, 10.0f);
  set_interval(mavlink::common::msg::RC_CHANNELS::MSG_ID, 50.0f);
  set_interval(mavlink::common::msg::LOCAL_POSITION_NED::MSG_ID, 50.0f);
}

void HoundFcuControlNode::request_mission_list()
{
  mavlink::common::msg::MISSION_REQUEST_LIST req{};
  req.target_system = target_system_;
  req.target_component = target_component_;
  req.mission_type = 0;
  send_to_fcu(req);
}

void HoundFcuControlNode::send_heartbeat()
{
  mavlink::minimal::msg::HEARTBEAT hb{};
  hb.type = static_cast<uint8_t>(mavlink::minimal::MAV_TYPE::ONBOARD_CONTROLLER);
  hb.autopilot = static_cast<uint8_t>(mavlink::minimal::MAV_AUTOPILOT::INVALID);
  hb.base_mode = 0;
  hb.custom_mode = 0;
  hb.system_status = static_cast<uint8_t>(mavlink::minimal::MAV_STATE::ACTIVE);
  send_to_fcu(hb);
}

void HoundFcuControlNode::send_vision_to_fcu(const ExtNavSample & nav)
{
  if (!send_vision_to_fcu_) {
    return;
  }
  // ENU -> NED for VISION_POSITION_ESTIMATE
  mavlink::common::msg::VISION_POSITION_ESTIMATE vp{};
  vp.usec = static_cast<uint64_t>(nav.stamp.nanoseconds() / 1000ULL);
  vp.x = nav.pos_enu[1];
  vp.y = nav.pos_enu[0];
  vp.z = -nav.pos_enu[2];
  // Yaw from quat (ENU) — approximate roll/pitch/yaw in NED via simple conversion
  const float w = nav.quat_wxyz[0], x = nav.quat_wxyz[1], y = nav.quat_wxyz[2],
    z = nav.quat_wxyz[3];
  const float siny_cosp = 2.0f * (w * z + x * y);
  const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
  const float yaw_enu = std::atan2(siny_cosp, cosy_cosp);
  vp.yaw = static_cast<float>(M_PI / 2.0) - yaw_enu;  // ENU yaw -> NED yaw
  vp.roll = 0.0f;
  vp.pitch = 0.0f;
  send_to_fcu(vp);
}

void HoundFcuControlNode::send_manual_control(const ManualControlCmd & cmd)
{
  mavlink::common::msg::MANUAL_CONTROL mc{};
  mc.target = target_system_;
  mc.x = static_cast<int16_t>(cmd.x);
  mc.y = static_cast<int16_t>(cmd.y);
  mc.z = static_cast<int16_t>(cmd.z);
  mc.r = static_cast<int16_t>(cmd.r);
  mc.buttons = cmd.buttons;
  send_to_fcu(mc);
}

void HoundFcuControlNode::vision_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  ExtNavSample s;
  s.stamp = msg->header.stamp;
  s.pos_enu[0] = static_cast<float>(msg->pose.pose.position.x);
  s.pos_enu[1] = static_cast<float>(msg->pose.pose.position.y);
  s.pos_enu[2] = static_cast<float>(msg->pose.pose.position.z);
  s.vel_enu[0] = static_cast<float>(msg->twist.twist.linear.x);
  s.vel_enu[1] = static_cast<float>(msg->twist.twist.linear.y);
  s.vel_enu[2] = static_cast<float>(msg->twist.twist.linear.z);
  s.has_vel = true;
  s.quat_wxyz[0] = static_cast<float>(msg->pose.pose.orientation.w);
  s.quat_wxyz[1] = static_cast<float>(msg->pose.pose.orientation.x);
  s.quat_wxyz[2] = static_cast<float>(msg->pose.pose.orientation.y);
  s.quat_wxyz[3] = static_cast<float>(msg->pose.pose.orientation.z);
  bus_.ext_nav.write(s);
  send_vision_to_fcu(s);
}

void HoundFcuControlNode::icp_origin_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  IcpOriginSample s;
  s.stamp = msg->header.stamp;
  s.pos_enu[0] = static_cast<float>(msg->pose.position.x);
  s.pos_enu[1] = static_cast<float>(msg->pose.position.y);
  s.pos_enu[2] = static_cast<float>(msg->pose.position.z);
  s.quat_wxyz[0] = static_cast<float>(msg->pose.orientation.w);
  s.quat_wxyz[1] = static_cast<float>(msg->pose.orientation.x);
  s.quat_wxyz[2] = static_cast<float>(msg->pose.orientation.y);
  s.quat_wxyz[3] = static_cast<float>(msg->pose.orientation.z);
  s.valid = true;
  bus_.icp_origin.write(s);
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "ICP origin sample (map ENU) received on %s", icp_origin_topic_.c_str());
}

void HoundFcuControlNode::vesc_cb(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
{
  ll_.update_vesc(
    static_cast<float>(msg->state.speed),
    static_cast<float>(msg->state.voltage_input),
    static_cast<float>(msg->state.duty_cycle),
    static_cast<float>(msg->state.current_input));
}

void HoundFcuControlNode::auto_control_cb(
  const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
  ll_.update_auto(
    static_cast<float>(msg->drive.steering_angle),
    static_cast<float>(msg->drive.speed));
}

void HoundFcuControlNode::ll_worker()
{
  pin_current_thread(ll_cpu_);
  uint64_t last_seq = 0;
  ImuSample imu;
  while (running_) {
    if (!bus_.imu.wait_new(last_seq, imu, running_)) {
      break;
    }
    float q[4] = {imu.qw, imu.qx, imu.qy, imu.qz};
    const auto cmd = ll_.tick_imu(
      imu.gx, imu.gy, imu.gz, imu.ax, imu.ay, imu.az,
      imu.has_orientation ? q : nullptr);
    if (!cmd.active) {
      continue;
    }
    ManualControlCmd mc;
    mc.x = 1000.0f;
    mc.y = -cmd.steering_norm * 1000.0f;
    mc.z = cmd.throttle_duty * 1000.0f;
    mc.r = 1000.0f;
    send_manual_control(mc);

    diagnostic_msgs::msg::DiagnosticArray dia;
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "LL_control";
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = "ok";
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "intervention";
    kv.value = std::to_string(cmd.intervention);
    st.values.push_back(kv);
    dia.status.push_back(st);
    diag_pub_->publish(dia);
  }
}

void HoundFcuControlNode::ekf_worker()
{
  pin_current_thread(ekf_cpu_);
  using inertial_nav_ros2::frames::BodyAxes;
  using inertial_nav_ros2::frames::enu_position_to_ned;
  using inertial_nav_ros2::frames::enu_quat_to_ned_frd;
  using inertial_nav_ros2::frames::enu_velocity_to_ned;
  using inertial_nav_ros2::frames::ned_frd_quat_to_enu;
  using inertial_nav_ros2::frames::ned_position_to_enu;
  using inertial_nav_ros2::frames::ned_velocity_to_frd_body;
  using inertial_nav_ros2::frames::ros_body_vector_to_frd;

  const bool align_gps_compass = (ext_nav_align_ == "gps_compass");
  const bool align_lidar_icp = (ext_nav_align_ == "lidar_icp");

  estimator_ekf ekf;
  ekf.set_ext_nav_origin(ext_nav_origin_lat_, ext_nav_origin_lon_,
    static_cast<float>(ext_nav_origin_hgt_));

  EkfSourceConfig cfg;
  // gps_compass: AHRS+mag sets earth yaw at hardcoded/GPS origin; VSLAM after yaw-align.
  // lidar_icp: wait for ICP pose, yaw-align VSLAM into that map, EXT_NAV init.
  cfg.init_ref = align_gps_compass ? InitReferenceSource::IMU_AHRS :
    InitReferenceSource::EXT_NAV;
  cfg.ext_nav[0].enabled = true;
  cfg.ext_nav[0].fuse_height = true;
  // After alignment, VSLAM yaw is in map frame — safe to fuse for both modes.
  cfg.ext_nav[0].fuse_yaw = true;
  cfg.ext_nav[0].fuse_vel = false;
  cfg.ext_nav[0].fuse_pos = true;
  cfg.ext_nav[0].pos_delay_ms = 100;
  cfg.ext_nav[0].yaw_delay_ms = 100;
  cfg.mag[0].enabled = enable_mag_;
  cfg.gps[0].enabled = enable_gps_ && fuse_gps_;
  cfg.baro_delay_ms = 50;
  ekf.set_ekf_config(cfg);
  ekf.set_baro_enabled(enable_baro_);
  ekf.set_ext_nav_enabled(0, true);
  ekf.set_mag_enabled(0, enable_mag_);

  RCLCPP_INFO(
    get_logger(),
    "EKF worker: align=%s init_ref=%s mag_max_hz=%.1f baro_max_hz=%.1f",
    ext_nav_align_.c_str(),
    align_gps_compass ? "imu_ahrs+compass" : "ext_nav+icp",
    mag_max_hz_, baro_max_hz_);

  uint64_t last_seq = 0;
  ImuSample imu;
  bool have_prev = false;
  uint32_t prev_ms = 0;
  bool filter_reset = true;
  const BodyAxes body_axes = BodyAxes::RosFlu;
  const int pub_every = std::max(1, 200 / std::max(1, ekf_odom_hz_));
  int imu_ticks_since_pub = 0;
  uint32_t last_mag_ms = 0;
  uint32_t last_baro_ms = 0;

  bool vslam_aligned = false;
  float yaw_corr = 0.0f;
  float t_corr_enu[3] = {0, 0, 0};
  bool logged_waiting_icp = false;

  while (running_) {
    if (!bus_.imu.wait_new(last_seq, imu, running_)) {
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
    ekf.setIMUData(accel, gyro, imu_dt, static_cast<float>(t_ms),
      static_cast<float>(static_cast<uint64_t>(t_ms) * 1000ULL));

    MagSample mag;
    if (enable_mag_ && bus_.mag.copy_latest(mag)) {
      const uint32_t m_ms = stamp_ms(mag.stamp);
      if (accept_rate_limited(m_ms, last_mag_ms, static_cast<float>(mag_max_hz_))) {
        float field_ut[3] = {
          mag.mx * 1.0e6f, mag.my * 1.0e6f, mag.mz * 1.0e6f};
        float field_frd[3];
        ros_body_vector_to_frd(field_ut, body_axes, field_frd);
        float zero[3] = {0, 0, 0};
        ekf.setMagData(field_frd, zero, true, 0);
      }
    }

    BaroSample baro;
    if (enable_baro_ && bus_.baro.copy_latest(baro)) {
      const uint32_t b_ms = stamp_ms(baro.stamp);
      if (accept_rate_limited(b_ms, last_baro_ms, static_cast<float>(baro_max_hz_))) {
        const float h = isa_height_m(baro.pressure_pa);
        ekf.setAirData(0.0f, h, 1.0f, imu_dt, true, false);
      }
    }

    GpsSample gps;
    if (enable_gps_ && cfg.gps[0].enabled && bus_.gps.copy_latest(gps) &&
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

    // --- Align VSLAM into map before feeding EXT_NAV ---
    ExtNavSample nav;
    bool have_nav = bus_.ext_nav.copy_latest(nav);
    if (have_nav && !vslam_aligned) {
      if (align_gps_compass) {
        // Need EKF already AHRS+mag initialized so map yaw is earth-fixed.
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
            // Pin current pose to map origin (0,0,0) — not "wherever GPS/baro left us".
            const float c = std::cos(yaw_corr);
            const float s = std::sin(yaw_corr);
            t_corr_enu[0] = -(c * nav.pos_enu[0] - s * nav.pos_enu[1]);
            t_corr_enu[1] = -(s * nav.pos_enu[0] + c * nav.pos_enu[1]);
            t_corr_enu[2] = -nav.pos_enu[2];
            // Snap filter position to origin so we do not fight a pre-align GPS yank.
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
              get_logger(),
              "VSLAM aligned to compass/AHRS map at origin "
              "(yaw_corr=%.2f deg, vslam_xy=(%.2f,%.2f))",
              yaw_corr * 180.0f / static_cast<float>(M_PI),
              nav.pos_enu[0], nav.pos_enu[1]);
          }
        }
      } else if (align_lidar_icp) {
        IcpOriginSample icp;
        if (!bus_.icp_origin.copy_latest(icp) || !icp.valid) {
          if (!logged_waiting_icp) {
            RCLCPP_WARN(
              get_logger(),
              "ext_nav_align=lidar_icp: waiting for PoseStamped on %s",
              icp_origin_topic_.c_str());
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
            get_logger(),
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

    ekf.run_filter(filter_reset);
    filter_reset = false;

    if (!ekf.is_initialized()) {
      continue;
    }
    const AttPosEKF * core = ekf.ekf_core();
    if (core == nullptr) {
      continue;
    }

    ++imu_ticks_since_pub;
    if (imu_ticks_since_pub < pub_every) {
      continue;
    }
    imu_ticks_since_pub = 0;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = imu.stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";
    const float * st = core->states;
    float pos_enu[3];
    ned_position_to_enu(&st[7], pos_enu);
    odom.pose.pose.position.x = pos_enu[0];
    odom.pose.pose.position.y = pos_enu[1];
    odom.pose.pose.position.z = pos_enu[2];
    float q_wxyz[4] = {st[0], st[1], st[2], st[3]};
    ned_frd_quat_to_enu(q_wxyz, body_axes, odom.pose.pose.orientation);
    float vel_frd[3], vel_ros[3];
    ned_velocity_to_frd_body(&st[4], q_wxyz, vel_frd);
    inertial_nav_ros2::frames::frd_body_vector_to_ros(vel_frd, body_axes, vel_ros);
    odom.twist.twist.linear.x = vel_ros[0];
    odom.twist.twist.linear.y = vel_ros[1];
    odom.twist.twist.linear.z = vel_ros[2];
    fill_ekf_odom_covariance(core->P, q_wxyz, odom);
    ekf_odom_pub_->publish(odom);
  }
}

void HoundFcuControlNode::ros_edge_timer()
{
  ImuSample imu;
  if (bus_.imu.copy_latest(imu)) {
    sensor_msgs::msg::Imu m;
    m.header.stamp = imu.stamp;
    m.header.frame_id = "base_link";
    m.angular_velocity.x = imu.gx;
    m.angular_velocity.y = imu.gy;
    m.angular_velocity.z = imu.gz;
    m.linear_acceleration.x = imu.ax;
    m.linear_acceleration.y = imu.ay;
    m.linear_acceleration.z = imu.az;
    if (imu.has_orientation) {
      m.orientation.w = imu.qw;
      m.orientation.x = imu.qx;
      m.orientation.y = imu.qy;
      m.orientation.z = imu.qz;
    } else {
      m.orientation_covariance[0] = -1.0;
    }
    imu_pub_->publish(m);
  }

  MagSample mag;
  if (bus_.mag.copy_latest(mag)) {
    sensor_msgs::msg::MagneticField m;
    m.header.stamp = mag.stamp;
    m.header.frame_id = "base_link";
    m.magnetic_field.x = mag.mx;
    m.magnetic_field.y = mag.my;
    m.magnetic_field.z = mag.mz;
    mag_pub_->publish(m);
  }

  BaroSample baro;
  if (bus_.baro.copy_latest(baro)) {
    sensor_msgs::msg::FluidPressure m;
    m.header.stamp = baro.stamp;
    m.fluid_pressure = baro.pressure_pa;
    baro_pub_->publish(m);
  }

  GpsSample gps;
  if (bus_.gps.copy_latest(gps)) {
    sensor_msgs::msg::NavSatFix m;
    m.header.stamp = gps.stamp;
    m.header.frame_id = "gps";
    m.latitude = gps.lat_deg;
    m.longitude = gps.lon_deg;
    m.altitude = gps.alt_m;
    m.status.status = (gps.fix_type >= 3) ?
      sensor_msgs::msg::NavSatStatus::STATUS_FIX :
      sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    m.position_covariance[0] = gps.eph_m * gps.eph_m;
    m.position_covariance[4] = gps.eph_m * gps.eph_m;
    m.position_covariance[8] = gps.epv_m * gps.epv_m;
    m.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    gps_pub_->publish(m);
  }

  ApLocalPoseSample ap;
  if (bus_.ap_local.copy_latest(ap)) {
    nav_msgs::msg::Odometry o;
    o.header.stamp = ap.stamp;
    o.header.frame_id = "map";
    o.child_frame_id = "base_link";
    o.pose.pose.position.x = ap.x;
    o.pose.pose.position.y = ap.y;
    o.pose.pose.position.z = ap.z;
    o.pose.pose.orientation.w = ap.qw;
    o.pose.pose.orientation.x = ap.qx;
    o.pose.pose.orientation.y = ap.qy;
    o.pose.pose.orientation.z = ap.qz;
    o.twist.twist.linear.x = ap.vx;
    o.twist.twist.linear.y = ap.vy;
    o.twist.twist.linear.z = ap.vz;
    ap_odom_pub_->publish(o);
  }

  {
    std::lock_guard<std::mutex> lock(mission_mu_);
    if (mission_dirty_) {
      mavros_msgs::msg::WaypointList list;
      list.waypoints = mission_items_;
      mission_pub_->publish(list);
      mission_dirty_ = false;
    }
  }
}

}  // namespace hound_core

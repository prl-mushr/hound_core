#include "hound_core/hound_fcu_control_modular_node.hpp"

#include <algorithm>
#include <utility>

namespace hound_core
{

HoundFcuControlModularNode::HoundFcuControlModularNode(const rclcpp::NodeOptions & options)
: Node("hound_fcu_control", options)
{
  declare_params();

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
    std::bind(&HoundFcuControlModularNode::vision_cb, this, std::placeholders::_1));
  if (ext_nav_align_ == "lidar_icp") {
    icp_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      icp_origin_topic_, 10,
      std::bind(&HoundFcuControlModularNode::icp_origin_cb, this, std::placeholders::_1));
  }
  vesc_sub_ = create_subscription<vesc_msgs::msg::VescStateStamped>(
    "/sensors/core", rclcpp::SensorDataQoS(),
    std::bind(&HoundFcuControlModularNode::vesc_cb, this, std::placeholders::_1));
  auto_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    "hound/control", 10,
    std::bind(&HoundFcuControlModularNode::auto_control_cb, this, std::placeholders::_1));

  MavlinkBridge::Config bridge_cfg;
  bridge_cfg.fcu_url = fcu_url_;
  bridge_cfg.gcs_url = gcs_url_;
  bridge_cfg.gcs_block_stream_requests = gcs_block_stream_requests_;
  bridge_cfg.gcs_throttle_hz = gcs_throttle_hz_;
  bridge_cfg.gcs_throttle_msgids = gcs_throttle_msgids_;
  bridge_cfg.send_vision_to_fcu = send_vision_to_fcu_;
  bridge_cfg.fcu_params = fcu_params_;
  bridge_cfg.system_id = system_id_;
  bridge_cfg.component_id = component_id_;
  bridge_cfg.target_system = target_system_;
  bridge_cfg.target_component = target_component_;

  bridge_ = std::make_unique<MavlinkBridge>(
    bridge_cfg, get_logger(), get_clock());
  bridge_->start(bus_);

  if (enable_ekf_) {
    EkfRunner::Config ekf_cfg;
    ekf_cfg.enable_baro = enable_baro_;
    ekf_cfg.enable_mag = enable_mag_;
    ekf_cfg.enable_gps = enable_gps_;
    ekf_cfg.fuse_gps = fuse_gps_;
    ekf_cfg.ekf_odom_hz = ekf_odom_hz_;
    ekf_cfg.mag_max_hz = mag_max_hz_;
    ekf_cfg.baro_max_hz = baro_max_hz_;
    ekf_cfg.ext_nav_align = ext_nav_align_;
    ekf_cfg.icp_origin_topic = icp_origin_topic_;
    ekf_cfg.ext_nav_origin_lat = ext_nav_origin_lat_;
    ekf_cfg.ext_nav_origin_lon = ext_nav_origin_lon_;
    ekf_cfg.ext_nav_origin_hgt = ext_nav_origin_hgt_;
    ekf_cfg.ekf_cpu = ekf_cpu_;

    ekf_runner_ = std::make_unique<EkfRunner>(get_logger());
    ekf_runner_->start(
      bus_, ekf_cfg,
      [this](const nav_msgs::msg::Odometry & odom) {
        ekf_odom_pub_->publish(odom);
      });
  }

  if (enable_ll_) {
    ll_runner_ = std::make_unique<LlRunner>(get_logger());
    ll_runner_->start(
      bus_, ll_,
      [this](const ManualControlCmd & cmd) {
        bridge_->send_manual_control(cmd);
      },
      ll_cpu_,
      [this](const diagnostic_msgs::msg::DiagnosticArray & dia) {
        diag_pub_->publish(dia);
      });
  }

  const auto edge_period = std::chrono::duration<double>(1.0 / std::max(1.0, ros_publish_hz_));
  edge_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(edge_period),
    std::bind(&HoundFcuControlModularNode::ros_edge_timer, this));
  hb_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    [this]() { bridge_->send_heartbeat(); });
  boot_timer_ = create_wall_timer(
    std::chrono::seconds(2),
    [this]() {
      if (bridge_->fcu_seen() && !bridge_->boot_done()) {
        bridge_->boot_configure();
      }
    });

  RCLCPP_INFO(
    get_logger(),
    "hound_fcu_control (modular) up: fcu=%s gcs=%s ekf=%d ll=%d ros_hz=%.1f "
    "align=%s ekf_odom_hz=%d",
    fcu_url_.c_str(), gcs_url_.empty() ? "(none)" : gcs_url_.c_str(),
    static_cast<int>(enable_ekf_), static_cast<int>(enable_ll_), ros_publish_hz_,
    ext_nav_align_.c_str(), ekf_odom_hz_);
}

HoundFcuControlModularNode::~HoundFcuControlModularNode()
{
  if (ll_runner_) {
    ll_runner_->stop();
  }
  if (ekf_runner_) {
    ekf_runner_->stop();
  }
  bus_.imu.cv.notify_all();
  if (bridge_) {
    bridge_->stop();
  }
}

void HoundFcuControlModularNode::declare_params()
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

void HoundFcuControlModularNode::vision_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
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
  bridge_->send_vision(s);
}

void HoundFcuControlModularNode::icp_origin_cb(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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

void HoundFcuControlModularNode::vesc_cb(
  const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
{
  ll_.update_vesc(
    static_cast<float>(msg->state.speed),
    static_cast<float>(msg->state.voltage_input),
    static_cast<float>(msg->state.duty_cycle),
    static_cast<float>(msg->state.current_input));
}

void HoundFcuControlModularNode::auto_control_cb(
  const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
  ll_.update_auto(
    static_cast<float>(msg->drive.steering_angle),
    static_cast<float>(msg->drive.speed));
}

void HoundFcuControlModularNode::ros_edge_timer()
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

  RcSample rc;
  if (bus_.rc.copy_latest(rc)) {
    ackermann_msgs::msg::AckermannDriveStamped limits;
    limits.header.stamp = rc.stamp;
    limits.drive.speed = ll_.auto_wheelspeed_limit();
    limits_pub_->publish(limits);
  }

  std::vector<mavros_msgs::msg::Waypoint> waypoints;
  if (bridge_->take_mission(waypoints)) {
    mavros_msgs::msg::WaypointList list;
    list.waypoints = std::move(waypoints);
    mission_pub_->publish(list);
  }
}

}  // namespace hound_core

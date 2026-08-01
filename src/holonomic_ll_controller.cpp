#include "hound_core/holonomic_ll_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "hound_core/ll_controller_registry.hpp"
#include "hound_core/mavlink_bridge.hpp"

namespace hound_core
{

HolonomicLlController::HolonomicLlController(MavlinkBridge & bridge, const Params & params)
: bridge_(bridge), p_(params)
{
  set_params(params);
}

void HolonomicLlController::set_params(const Params & params)
{
  std::lock_guard<std::mutex> lock(mu_);
  p_ = params;
  pid_x_.configure(p_.pid_vel_x);
  pid_y_.configure(p_.pid_vel_y);
  pid_yaw_.configure(p_.pid_yaw_rate);
}

HolonomicLlController::SwitchPos HolonomicLlController::switch_pos() const
{
  std::lock_guard<std::mutex> lock(mu_);
  return switch_pos_;
}

float HolonomicLlController::apply_deadband(float norm, float deadband)
{
  if (std::fabs(norm) < deadband) {
    return 0.0f;
  }
  return norm;
}

float HolonomicLlController::pwm_to_norm(float pwm)
{
  return std::clamp((pwm - 1500.0f) / 500.0f, -1.0f, 1.0f);
}

uint16_t HolonomicLlController::effort_to_pwm(float effort) const
{
  const float clamped = std::clamp(effort, -1.0f, 1.0f);
  return static_cast<uint16_t>(1500 + static_cast<int>(clamped * 500.0f));
}

void HolonomicLlController::update_rc(const RcSample & rc)
{
  if (rc.nchan == 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(mu_);
  rc_watchdog_.stamp(rc.stamp);
  channel_init_ = true;

  const auto ch = [&](int idx) -> float {
      if (idx < 0 || static_cast<std::size_t>(idx) >= rc.nchan) {
        return 1500.0f;
      }
      return rc.channels[static_cast<std::size_t>(idx)];
    };

  const float mode_pwm = ch(p_.input_ch_mode);
  // Opendubs thresholds: low=Manual, high=Auto; mid band = Off.
  SwitchPos new_pos = SwitchPos::Off;
  if (mode_pwm < p_.mode_pwm_manual_threshold) {
    new_pos = SwitchPos::Manual;
  } else if (mode_pwm > p_.mode_pwm_auto_threshold) {
    new_pos = SwitchPos::Auto;
  }

  if (new_pos != switch_pos_) {
    if (new_pos == SwitchPos::Auto || new_pos == SwitchPos::Manual) {
      pid_x_.reset();
      pid_y_.reset();
      pid_yaw_.reset();
    }
    switch_pos_ = new_pos;
  }

  semi_fwd_ = apply_deadband(pwm_to_norm(ch(p_.input_ch_fwd)), p_.input_deadband);
  semi_lat_ = apply_deadband(pwm_to_norm(ch(p_.input_ch_lat)), p_.input_deadband);
  semi_yaw_ = apply_deadband(pwm_to_norm(ch(p_.input_ch_yaw)), p_.input_deadband);
}

void HolonomicLlController::update_mode(const FcuStateSample & state)
{
  std::lock_guard<std::mutex> lock(mu_);
  mode_init_ = true;
  guided_ = state.guided && state.armed;
}

void HolonomicLlController::update_auto_cmd_vel(float vx, float vy, float wz)
{
  update_auto_cmd_vel(vx, vy, wz, rclcpp::Clock(RCL_ROS_TIME).now());
}

void HolonomicLlController::update_auto_cmd_vel(
  float vx, float vy, float wz, const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mu_);
  auto_init_ = true;
  auto_vx_ = vx;
  auto_vy_ = vy;
  auto_wz_ = wz;
  auto_cmd_watchdog_.stamp(stamp);
}

void HolonomicLlController::update_odom_velocity(float vx, float vy)
{
  std::lock_guard<std::mutex> lock(mu_);
  odom_vx_ = vx;
  odom_vy_ = vy;
  odom_init_ = true;
}

void HolonomicLlController::publish_override(float fwd, float lat, float yaw)
{
  RcOverrideCmd cmd = RcOverrideCmd::all_release();
  if (p_.output_ch_fwd < 18) {
    cmd.channels[static_cast<std::size_t>(p_.output_ch_fwd)] = effort_to_pwm(fwd);
  }
  if (p_.output_ch_lat < 18) {
    cmd.channels[static_cast<std::size_t>(p_.output_ch_lat)] = effort_to_pwm(lat);
  }
  if (p_.output_ch_turn < 18) {
    cmd.channels[static_cast<std::size_t>(p_.output_ch_turn)] = effort_to_pwm(yaw);
  }
  bridge_.send_rc_override(cmd);
}

void HolonomicLlController::publish_neutral_and_hold()
{
  bridge_.send_rc_override(
    RcOverrideCmd::neutral_on(
      static_cast<uint8_t>(p_.output_ch_fwd),
      static_cast<uint8_t>(p_.output_ch_lat),
      static_cast<uint8_t>(p_.output_ch_turn)));
  bridge_.request_mode(p_.mode_hold_custom_mode);
}

HolonomicLlController::HolonomicCommand HolonomicLlController::compute(
  const ImuSample & imu, const rclcpp::Time & now)
{
  HolonomicCommand out;

  SwitchPos switch_pos = SwitchPos::Off;
  bool guided = false;
  bool channel_init = false;
  bool mode_init = false;
  bool auto_init = false;
  float semi_fwd = 0.0f;
  float semi_lat = 0.0f;
  float semi_yaw = 0.0f;
  float auto_vx = 0.0f;
  float auto_vy = 0.0f;
  float auto_wz = 0.0f;
  float odom_vx = 0.0f;
  float odom_vy = 0.0f;
  bool use_odom = false;
  double cmd_timeout = 0.5;
  bool auto_stale = false;
  bool rc_stale = false;
  double dt = 0.02;

  {
    std::lock_guard<std::mutex> lock(mu_);
    switch_pos = switch_pos_;
    guided = guided_;
    channel_init = channel_init_;
    mode_init = mode_init_;
    auto_init = auto_init_;
    semi_fwd = semi_fwd_;
    semi_lat = semi_lat_;
    semi_yaw = semi_yaw_;
    auto_vx = auto_vx_;
    auto_vy = auto_vy_;
    auto_wz = auto_wz_;
    odom_vx = odom_vx_;
    odom_vy = odom_vy_;
    use_odom = p_.use_odom_feedback && odom_init_;
    cmd_timeout = p_.cmd_vel_timeout;
    auto_stale = auto_cmd_watchdog_.is_stale(now, cmd_timeout);
    rc_stale = rc_watchdog_.is_stale(now, cmd_timeout);
    if (last_tick_.nanoseconds() != 0) {
      dt = (now - last_tick_).seconds();
      if (dt <= 0.0 || dt > 1.0) {
        dt = 0.02;
      }
    }
    last_tick_ = now;
  }

  if (!channel_init || !mode_init || switch_pos == SwitchPos::Off) {
    return out;
  }

  // Staleness: RC always required; auto cmd required in Auto.
  if (rc_stale || (switch_pos == SwitchPos::Auto && (auto_stale || !auto_init))) {
    out.fault = true;
    out.requested_custom_mode = p_.mode_hold_custom_mode;
    out.fwd = 0.0f;
    out.lat = 0.0f;
    out.yaw = 0.0f;
    return out;
  }

  if (switch_pos == SwitchPos::Manual) {
    out.active = true;
    out.fwd = semi_fwd;
    out.lat = semi_lat;
    out.yaw = semi_yaw;
    return out;
  }

  // Auto: optionally gate on guided (mirrors Ackermann). Keep permissive if
  // the FCU is not yet reporting guided — still run PID when armed/guided.
  if (!guided) {
    return out;
  }

  const float meas_vx = use_odom ? odom_vx : 0.0f;  // feedforward-only when no odom
  const float meas_vy = use_odom ? odom_vy : 0.0f;
  const float meas_wz = imu.gz;

  out.active = true;
  out.fwd = static_cast<float>(pid_x_.calculate(auto_vx, meas_vx, dt));
  out.lat = static_cast<float>(pid_y_.calculate(auto_vy, meas_vy, dt));
  out.yaw = static_cast<float>(pid_yaw_.calculate(auto_wz, meas_wz, dt));
  return out;
}

LlStatus HolonomicLlController::tick_imu(const ImuSample & imu)
{
  const rclcpp::Time now = (imu.stamp.nanoseconds() != 0) ?
    imu.stamp : rclcpp::Clock(RCL_ROS_TIME).now();
  const HolonomicCommand cmd = compute(imu, now);

  LlStatus status;
  status.active = cmd.active;
  status.intervention = cmd.fault;

  if (cmd.fault) {
    publish_neutral_and_hold();
    status.diagnostics.push_back({"fault", "stale_input"});
    return status;
  }
  if (!cmd.active) {
    return status;
  }

  publish_override(cmd.fwd, cmd.lat, cmd.yaw);
  status.diagnostics.push_back({"fwd", std::to_string(cmd.fwd)});
  status.diagnostics.push_back({"lat", std::to_string(cmd.lat)});
  status.diagnostics.push_back({"yaw", std::to_string(cmd.yaw)});
  return status;
}

void HolonomicLlController::setup_subscriptions(rclcpp::Node & node)
{
  auto_sub_ = node.create_subscription<geometry_msgs::msg::Twist>(
    "hound/control", 10,
    [this, &node](const geometry_msgs::msg::Twist::SharedPtr msg) {
      update_auto_cmd_vel(
        static_cast<float>(msg->linear.x),
        static_cast<float>(msg->linear.y),
        static_cast<float>(msg->angular.z),
        node.get_clock()->now());
    });
}

namespace
{

PidConfig declare_pid(rclcpp::Node & node, const std::string & prefix, const PidConfig & def)
{
  PidConfig cfg;
  cfg.max_output = node.declare_parameter(prefix + ".max_output", def.max_output);
  cfg.k_p = node.declare_parameter(prefix + ".k_p", def.k_p);
  cfg.k_i = node.declare_parameter(prefix + ".k_i", def.k_i);
  cfg.k_d = node.declare_parameter(prefix + ".k_d", def.k_d);
  cfg.k_ff = node.declare_parameter(prefix + ".k_ff", def.k_ff);
  cfg.integrity_limit = node.declare_parameter(prefix + ".integrity_limit", def.integrity_limit);
  return cfg;
}

HolonomicLlController::Params params_from_node(rclcpp::Node & node)
{
  HolonomicLlController::Params p;
  PidConfig def_x;
  def_x.max_output = 1.0;
  def_x.k_ff = 1.0;
  PidConfig def_y = def_x;
  PidConfig def_yaw;
  def_yaw.max_output = 1.57;
  def_yaw.k_p = 0.15;
  def_yaw.k_ff = 0.7;

  p.pid_vel_x = declare_pid(node, "ll.pid_vel_x", def_x);
  p.pid_vel_y = declare_pid(node, "ll.pid_vel_y", def_y);
  p.pid_yaw_rate = declare_pid(node, "ll.pid_yaw_rate", def_yaw);

  p.input_ch_mode = static_cast<int>(node.declare_parameter("ll.input_ch_mode", 4));
  p.input_ch_fwd = static_cast<int>(node.declare_parameter("ll.input_ch_fwd", 1));
  p.input_ch_lat = static_cast<int>(node.declare_parameter("ll.input_ch_lat", 0));
  p.input_ch_yaw = static_cast<int>(node.declare_parameter("ll.input_ch_yaw", 3));
  p.mode_pwm_manual_threshold =
    static_cast<float>(node.declare_parameter("ll.mode_pwm_manual_threshold", 1300.0));
  p.mode_pwm_auto_threshold =
    static_cast<float>(node.declare_parameter("ll.mode_pwm_auto_threshold", 1700.0));
  p.input_deadband = static_cast<float>(node.declare_parameter("ll.input_deadband", 0.05));
  p.vel_max_linear = static_cast<float>(node.declare_parameter("ll.vel_max_linear", 1.0));
  p.vel_max_angular = static_cast<float>(node.declare_parameter("ll.vel_max_angular", 1.57));
  p.output_ch_fwd = static_cast<int>(node.declare_parameter("ll.output_ch_fwd", 8));
  p.output_ch_lat = static_cast<int>(node.declare_parameter("ll.output_ch_lat", 9));
  p.output_ch_turn = static_cast<int>(node.declare_parameter("ll.output_ch_turn", 10));
  p.cmd_vel_timeout = node.declare_parameter("ll.cmd_vel_timeout", 0.5);
  p.mode_hold_custom_mode =
    static_cast<uint32_t>(node.declare_parameter("ll.mode_hold_custom_mode", 4));
  p.use_odom_feedback = node.declare_parameter("ll.use_odom_feedback", false);
  return p;
}

LlControllerEntry make_holonomic(rclcpp::Node & node, MavlinkBridge & bridge)
{
  auto controller = std::make_unique<HolonomicLlController>(bridge, params_from_node(node));
  HolonomicLlController * raw = controller.get();
  LlControllerEntry entry;
  entry.controller = std::move(controller);
  entry.setup_subscriptions = [raw](rclcpp::Node & n) {raw->setup_subscriptions(n);};
  return entry;
}

}  // namespace

HOUND_REGISTER_LL_CONTROLLER("holonomic", make_holonomic)

}  // namespace hound_core

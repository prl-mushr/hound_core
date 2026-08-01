#include "hound_core/ackermann_ll_controller.hpp"

#include <algorithm>
#include <string>

#include "hound_core/ll_controller_registry.hpp"
#include "hound_core/mavlink_bridge.hpp"

namespace hound_core
{

AckermannLlController::AckermannLlController(MavlinkBridge & bridge, const Params & params)
: bridge_(bridge), p_(params), throttle_slew_(params.throttle_delta)
{
  max_rated_speed_ = 2.0f * 0.69f * p_.motor_kv * p_.nominal_voltage / std::fabs(p_.erpm_gain);
}

void AckermannLlController::set_params(const Params & params)
{
  std::lock_guard<std::mutex> lock(mu_);
  p_ = params;
  throttle_slew_.set_max_delta(p_.throttle_delta);
  max_rated_speed_ = 2.0f * 0.69f * p_.motor_kv * p_.nominal_voltage / std::fabs(p_.erpm_gain);
}

float AckermannLlController::auto_wheelspeed_limit() const
{
  std::lock_guard<std::mutex> lock(mu_);
  return auto_wheelspeed_limit_;
}

void AckermannLlController::lpf(const Vec3 & measurement, Vec3 & estimate)
{
  estimate.x = p_.LPF_tau * measurement.x + (1.0f - p_.LPF_tau) * estimate.x;
  estimate.y = p_.LPF_tau * measurement.y + (1.0f - p_.LPF_tau) * estimate.y;
  estimate.z = p_.LPF_tau * measurement.z + (1.0f - p_.LPF_tau) * estimate.z;
}

void AckermannLlController::rpy_from_quat_wxyz(float w, float x, float y, float z)
{
  const float q[4] = {x, y, z, w};
  rpy_.x = std::asin(2.0f * (q[0] * q[2] - q[3] * q[1]));
  rpy_.y = std::atan2(
    2.0f * (q[0] * q[3] + q[1] * q[2]),
    1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
  rpy_.z = std::atan2(
    2.0f * (q[0] * q[1] + q[2] * q[3]),
    1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
}

void AckermannLlController::update_vesc(
  float erpm, float voltage_input, float duty_cycle, float current_input)
{
  std::lock_guard<std::mutex> lock(mu_);
  vesc_init_ = true;
  wheelspeed_ = erpm / p_.erpm_gain;
  voltage_input_ = voltage_input;
  const float power_applied = std::fabs(duty_cycle * current_input);
  const float Kd_meas = power_applied / std::max(1.0f, wheelspeed_ * wheelspeed_);
  K_drag_ = std::min(1.0f, std::max(0.0f, 0.2f * Kd_meas + 0.8f * K_drag_));
}

void AckermannLlController::update_rc(const RcSample & rc)
{
  if (rc.nchan < 5) {
    return;
  }
  const float * channels = rc.channels.data();
  std::lock_guard<std::mutex> lock(mu_);
  semi_steering_ = p_.steering_max * ((channels[0] - 1500.0f) / 500.0f);
  semi_wheelspeed_ = p_.wheelspeed_max * ((channels[2] - 1000.0f) / 1000.0f);
  auto_wheelspeed_limit_ = p_.wheelspeed_max * ((channels[2] - 1000.0f) / 1000.0f);
  channel_init_ = true;

  const int mode_switch = static_cast<int>(channels[4]);
  if (mode_switch < 1200) {
    switch_pos_ = 0;
  } else if (mode_switch > 1200 && mode_switch < 1800) {
    switch_pos_ = 1;
  } else if (mode_switch > 1800) {
    switch_pos_ = 2;
  } else {
    switch_pos_ = 0;
  }

  if (limits_pub_) {
    ackermann_msgs::msg::AckermannDriveStamped limits;
    limits.header.stamp = rc.stamp;
    limits.drive.speed = auto_wheelspeed_limit_;
    limits_pub_->publish(limits);
  }
}

void AckermannLlController::update_mode(const FcuStateSample & state)
{
  std::lock_guard<std::mutex> lock(mu_);
  mode_init_ = true;
  guided_ = state.guided && state.armed;
}

void AckermannLlController::update_auto(float steering_rad, float speed_mps)
{
  std::lock_guard<std::mutex> lock(mu_);
  auto_init_ = true;
  auto_steering_ = steering_rad;
  auto_wheelspeed_ = speed_mps;
}

AckermannLlController::ManualCommand AckermannLlController::compute(const ImuSample & imu)
{
  ManualCommand out;
  bool channel_init = false;
  bool mode_init = false;
  bool vesc_init = false;
  int switch_pos = 0;
  bool guided = false;
  bool auto_init = false;
  float semi_steering = 0.0f;
  float semi_wheelspeed = 0.0f;
  float auto_steering = 0.0f;
  float auto_wheelspeed = 0.0f;
  float auto_wheelspeed_limit = 0.0f;
  bool safe_mode = true;
  float steering_max = 0.488f;

  {
    std::lock_guard<std::mutex> lock(mu_);
    imu_init_ = true;
    const Vec3 rot_bf_meas{imu.gx, imu.gy, imu.gz};
    const Vec3 acc_bf_meas{imu.ax, imu.ay, imu.az};
    lpf(acc_bf_meas, acc_bf_);
    rot_bf_ = rot_bf_meas;
    if (imu.has_orientation) {
      rpy_from_quat_wxyz(imu.qw, imu.qx, imu.qy, imu.qz);
    }

    channel_init = channel_init_;
    mode_init = mode_init_;
    vesc_init = vesc_init_;
    switch_pos = switch_pos_;
    guided = guided_;
    auto_init = auto_init_;
    semi_steering = semi_steering_;
    semi_wheelspeed = semi_wheelspeed_;
    auto_steering = auto_steering_;
    auto_wheelspeed = auto_wheelspeed_;
    auto_wheelspeed_limit = auto_wheelspeed_limit_;
    safe_mode = p_.safe_mode;
    steering_max = p_.steering_max;
  }

  if (!channel_init || !mode_init || !vesc_init || switch_pos == 0) {
    return out;
  }
  if ((switch_pos == 2 && guided) && !auto_init) {
    return out;
  }
  if (!(switch_pos >= 1 && guided)) {
    return out;
  }

  float wheelspeed_setpoint = 0.0f;
  float steering_setpoint = 0.0f;
  if (switch_pos == 1) {
    wheelspeed_setpoint = semi_wheelspeed;
    steering_setpoint = semi_steering;
  } else {
    wheelspeed_setpoint = std::min(auto_wheelspeed, auto_wheelspeed_limit);
    steering_setpoint = auto_steering;
  }

  bool intervention = false;
  if (safe_mode) {
    steering_setpoint = steering_limiter(steering_setpoint, intervention);
  }
  const float throttle_duty = speed_controller(wheelspeed_setpoint);

  out.active = true;
  out.intervention = intervention;
  out.steering_norm = steering_setpoint / steering_max;
  out.throttle_duty = throttle_duty;
  out.wheelspeed_setpoint = wheelspeed_setpoint;
  out.steering_setpoint = steering_setpoint;
  return out;
}

LlStatus AckermannLlController::tick_imu(const ImuSample & imu)
{
  const ManualCommand cmd = compute(imu);
  LlStatus status;
  status.active = cmd.active;
  status.intervention = cmd.intervention;
  if (!cmd.active) {
    return status;
  }

  ManualControlCmd mc;
  mc.x = 1000.0f;
  mc.y = -cmd.steering_norm * 1000.0f;
  mc.z = cmd.throttle_duty * 1000.0f;
  mc.r = 1000.0f;
  bridge_.send_manual_control(mc);

  status.diagnostics.push_back({"intervention", std::to_string(cmd.intervention)});
  status.diagnostics.push_back({"wheelspeed_setpoint", std::to_string(cmd.wheelspeed_setpoint)});
  status.diagnostics.push_back({"steering_setpoint", std::to_string(cmd.steering_setpoint)});
  return status;
}

float AckermannLlController::speed_controller(float wheelspeed_setpoint)
{
  std::lock_guard<std::mutex> lock(mu_);
  const float speed_error = (wheelspeed_setpoint - wheelspeed_) / max_rated_speed_;
  const float Kp_speed_error = p_.speed_control_kp * speed_error;
  const float Ki_speed_error_dt = p_.speed_control_ki * speed_error * p_.control_dt;

  speed_proportional_ = std::min(std::max(-0.05f, Kp_speed_error), 0.05f);
  speed_integral_ = std::min(
    std::max(-0.025f, Ki_speed_error_dt + speed_integral_), 0.025f);

  const float voltage_gain = p_.nominal_voltage / std::max(1.0f, voltage_input_);
  if (wheelspeed_setpoint < 0.1f * p_.wheelspeed_max) {
    speed_integral_ = 0.0f;
    speed_proportional_ = 0.0f;
  }

  float throttle_duty = voltage_gain * (
    (1.0f + K_drag_) * (wheelspeed_setpoint / max_rated_speed_) + speed_error +
    speed_integral_);
  throttle_duty = std::max(throttle_duty, 0.0f);
  throttle_duty = throttle_slew_.apply(throttle_duty);
  return throttle_duty;
}

float AckermannLlController::steering_limiter(float steering_setpoint, bool & intervention)
{
  std::lock_guard<std::mutex> lock(mu_);
  intervention = false;
  float whspd2 = std::max(1.0f, wheelspeed_);
  whspd2 *= whspd2;

  float Aylim = p_.track_width * 0.5f * std::max(1.0f, std::fabs(acc_bf_.z)) / p_.cg_height;
  if (p_.liftoff_oversteer) {
    Aylim = std::sqrt(std::max(Aylim * Aylim - acc_bf_.x * acc_bf_.x, 0.0f));
  }

  const float steering_input_temp = steering_setpoint;
  const float steering_limit_max =
    std::atan2(p_.wheelbase * (Aylim - 9.81f * std::sin(rpy_.x)), whspd2) +
    p_.steer_slack * p_.steering_max;
  const float steering_limit_min =
    -std::atan2(p_.wheelbase * (Aylim + 9.81f * std::sin(rpy_.x)), whspd2) -
    p_.steer_slack * p_.steering_max;

  steering_setpoint = std::min(
    steering_limit_max, std::max(steering_limit_min, steering_setpoint));
  if (std::fabs(steering_setpoint - steering_input_temp) > 0.01f) {
    intervention = true;
  }

  const float Ay = acc_bf_.y;
  if (std::fabs(Ay) > Aylim) {
    intervention = true;
    float Ay_error = 0.0f;
    float delta_steering = 0.0f;
    if (Ay >= 0.0f) {
      Ay_error = Aylim - Ay;
      delta_steering =
        (p_.accel_gain * Ay_error - p_.roll_gain * rot_bf_.x * std::fabs(acc_bf_.z)) *
        std::cos(steering_setpoint) * std::cos(steering_setpoint) * p_.wheelbase / whspd2;
      delta_steering = std::min(delta_steering, 0.0f);
    } else {
      Ay_error = -Aylim - Ay;
      delta_steering =
        (p_.accel_gain * Ay_error - p_.roll_gain * rot_bf_.x * std::fabs(acc_bf_.z)) *
        std::cos(steering_setpoint) * std::cos(steering_setpoint) * p_.wheelbase / whspd2;
      delta_steering = std::max(delta_steering, 0.0f);
    }
    steering_setpoint += delta_steering;
  }
  return steering_setpoint;
}

void AckermannLlController::setup_subscriptions(rclcpp::Node & node)
{
  vesc_sub_ = node.create_subscription<vesc_msgs::msg::VescStateStamped>(
    "/sensors/core", rclcpp::SensorDataQoS(),
    [this](const vesc_msgs::msg::VescStateStamped::SharedPtr msg) {
      update_vesc(
        static_cast<float>(msg->state.speed),
        static_cast<float>(msg->state.voltage_input),
        static_cast<float>(msg->state.duty_cycle),
        static_cast<float>(msg->state.current_input));
    });
  auto_sub_ = node.create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    "hound/control", 10,
    [this](const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
      update_auto(
        static_cast<float>(msg->drive.steering_angle),
        static_cast<float>(msg->drive.speed));
    });
  limits_pub_ = node.create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
    "/control_limits", 1);
}

namespace
{

AckermannLlController::Params params_from_node(rclcpp::Node & node)
{
  AckermannLlController::Params lp;
  lp.erpm_gain = static_cast<float>(node.declare_parameter("ll.erpm_gain", 3166.6));
  lp.steering_max = static_cast<float>(node.declare_parameter("ll.steering_max", 0.488));
  lp.wheelbase = static_cast<float>(node.declare_parameter("ll.wheelbase", 0.29));
  lp.cg_height = static_cast<float>(node.declare_parameter("ll.cg_height", 0.125));
  lp.wheelspeed_max = static_cast<float>(node.declare_parameter("ll.wheelspeed_max", 17.0));
  lp.nominal_voltage = static_cast<float>(node.declare_parameter("ll.nominal_voltage", 14.8));
  lp.motor_kv = static_cast<float>(node.declare_parameter("ll.motor_kv", 3930.0));
  lp.speed_control_kp = static_cast<float>(node.declare_parameter("ll.speed_control_kp", 1.0));
  lp.speed_control_ki = static_cast<float>(node.declare_parameter("ll.speed_control_ki", 1.0));
  lp.safe_mode = node.declare_parameter("ll.safe_mode", true);
  lp.track_width = static_cast<float>(node.declare_parameter("ll.track_width", 0.25));
  lp.accel_gain = static_cast<float>(node.declare_parameter("ll.accel_gain", 1.0));
  lp.roll_gain = static_cast<float>(node.declare_parameter("ll.roll_gain", 0.33));
  lp.steer_slack = static_cast<float>(node.declare_parameter("ll.steer_slack", 0.4));
  lp.LPF_tau = static_cast<float>(node.declare_parameter("ll.LPF_tau", 0.2));
  lp.throttle_delta = static_cast<float>(node.declare_parameter("ll.throttle_delta", 0.02));
  lp.liftoff_oversteer = node.declare_parameter("ll.liftoff_oversteer", true);
  lp.control_dt = static_cast<float>(node.declare_parameter("ll.control_dt", 0.02));
  return lp;
}

LlControllerEntry make_ackermann(rclcpp::Node & node, MavlinkBridge & bridge)
{
  auto controller = std::make_unique<AckermannLlController>(bridge, params_from_node(node));
  AckermannLlController * raw = controller.get();
  LlControllerEntry entry;
  entry.controller = std::move(controller);
  entry.setup_subscriptions = [raw](rclcpp::Node & n) {raw->setup_subscriptions(n);};
  return entry;
}

}  // namespace

HOUND_REGISTER_LL_CONTROLLER("ackermann", make_ackermann)

}  // namespace hound_core

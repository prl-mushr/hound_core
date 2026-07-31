#include "hound_core/ll_controller.hpp"

#include <algorithm>

namespace hound_core
{

LlController::LlController()
: LlController(Params{})
{
}

LlController::LlController(const Params & params)
: p_(params)
{
  max_rated_speed_ = 2.0f * 0.69f * p_.motor_kv * p_.nominal_voltage / std::fabs(p_.erpm_gain);
}

void LlController::set_params(const Params & params)
{
  std::lock_guard<std::mutex> lock(mu_);
  p_ = params;
  max_rated_speed_ = 2.0f * 0.69f * p_.motor_kv * p_.nominal_voltage / std::fabs(p_.erpm_gain);
}

float LlController::auto_wheelspeed_limit() const
{
  std::lock_guard<std::mutex> lock(mu_);
  return auto_wheelspeed_limit_;
}

void LlController::lpf(const Vec3 & measurement, Vec3 & estimate)
{
  estimate.x = p_.LPF_tau * measurement.x + (1.0f - p_.LPF_tau) * estimate.x;
  estimate.y = p_.LPF_tau * measurement.y + (1.0f - p_.LPF_tau) * estimate.y;
  estimate.z = p_.LPF_tau * measurement.z + (1.0f - p_.LPF_tau) * estimate.z;
}

void LlController::rpy_from_quat_wxyz(float w, float x, float y, float z)
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

void LlController::update_vesc(float erpm, float voltage_input, float duty_cycle, float current_input)
{
  std::lock_guard<std::mutex> lock(mu_);
  vesc_init_ = true;
  wheelspeed_ = erpm / p_.erpm_gain;
  voltage_input_ = voltage_input;
  const float power_applied = std::fabs(duty_cycle * current_input);
  const float Kd_meas = power_applied / std::max(1.0f, wheelspeed_ * wheelspeed_);
  K_drag_ = std::min(1.0f, std::max(0.0f, 0.2f * Kd_meas + 0.8f * K_drag_));
}

void LlController::update_rc(const float * channels, std::size_t n)
{
  if (channels == nullptr || n < 5) {
    return;
  }
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
}

void LlController::update_mode(bool armed, bool guided)
{
  std::lock_guard<std::mutex> lock(mu_);
  mode_init_ = true;
  guided_ = guided && armed;
}

void LlController::update_auto(float steering_rad, float speed_mps)
{
  std::lock_guard<std::mutex> lock(mu_);
  auto_init_ = true;
  auto_steering_ = steering_rad;
  auto_wheelspeed_ = speed_mps;
}

LlController::ManualCommand LlController::tick_imu(
  float gx, float gy, float gz, float ax, float ay, float az,
  const float * quat_wxyz)
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
    const Vec3 rot_bf_meas{gx, gy, gz};
    const Vec3 acc_bf_meas{ax, ay, az};
    lpf(acc_bf_meas, acc_bf_);
    rot_bf_ = rot_bf_meas;
    if (quat_wxyz != nullptr) {
      rpy_from_quat_wxyz(quat_wxyz[0], quat_wxyz[1], quat_wxyz[2], quat_wxyz[3]);
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

float LlController::speed_controller(float wheelspeed_setpoint)
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
  throttle_duty = std::min(
    std::max(last_throttle_ - p_.throttle_delta, throttle_duty),
    last_throttle_ + p_.throttle_delta);
  last_throttle_ = throttle_duty;
  return throttle_duty;
}

float LlController::steering_limiter(float steering_setpoint, bool & intervention)
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

}  // namespace hound_core

#include "hound_core/ll_controller_node.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace hound_core
{

LlControllerNode::LlControllerNode(const rclcpp::NodeOptions & options)
: Node("low_level_controller", options)
{
  declare_parameters();

  sub_vesc_ = create_subscription<vesc_msgs::msg::VescStateStamped>(
    "/sensors/core", rclcpp::SensorDataQoS(),
    std::bind(&LlControllerNode::vesc_cb, this, std::placeholders::_1));

  sub_channel_ = create_subscription<mavros_msgs::msg::RCIn>(
    "/mavros/rc/in", 10,
    std::bind(&LlControllerNode::channel_cb, this, std::placeholders::_1));

  sub_mode_ = create_subscription<mavros_msgs::msg::State>(
    "/mavros/state", 10,
    std::bind(&LlControllerNode::mode_cb, this, std::placeholders::_1));

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    "/mavros/imu/data_raw", rclcpp::SensorDataQoS(),
    std::bind(&LlControllerNode::imu_cb, this, std::placeholders::_1));

  sub_auto_control_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    "hound/control", 10,
    std::bind(&LlControllerNode::auto_control_cb, this, std::placeholders::_1));

  control_pub_ = create_publisher<mavros_msgs::msg::ManualControl>(
    "/mavros/manual_control/send", 10);
  diagnostic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/low_level_diagnostics", 1);
  limits_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
    "/control_limits", 1);

  max_rated_speed_ = 2.0f * 0.69f * motor_kv_ * nominal_voltage_ / std::fabs(erpm_gain_);

  RCLCPP_INFO(
    get_logger(),
    "Low-level controller ready (max_rated_speed=%.2f m/s, safe_mode=%s)",
    max_rated_speed_, safe_mode_ ? "on" : "off");
}

void LlControllerNode::declare_parameters()
{
  erpm_gain_ = declare_parameter("erpm_gain", 3500.0);
  steering_max_ = declare_parameter("steering_max", 0.488);
  wheelbase_ = declare_parameter("wheelbase", 0.29);
  cg_height_ = declare_parameter("cg_height", 0.136);
  wheelspeed_max_ = declare_parameter("wheelspeed_max", 17.0);
  nominal_voltage_ = declare_parameter("nominal_voltage", 14.8);
  motor_kv_ = declare_parameter("motor_kv", 3930.0);
  speed_control_kp_ = declare_parameter("speed_control_kp", 1.0);
  speed_control_ki_ = declare_parameter("speed_control_ki", 1.0);
  safe_mode_ = declare_parameter("safe_mode", true);
  track_width_ = declare_parameter("track_width", 0.25);
  accel_gain_ = declare_parameter("accel_gain", 1.0);
  roll_gain_ = declare_parameter("roll_gain", 0.33);
  steer_slack_ = declare_parameter("steer_slack", 0.4);
  LPF_tau_ = declare_parameter("LPF_tau", 0.2);
  throttle_delta_ = declare_parameter("throttle_delta", 0.02);
  liftoff_oversteer_ = declare_parameter("liftoff_oversteer", true);
  delta_t_ = declare_parameter("control_dt", 0.02);
  declare_parameter("executor_threads", 8);
}

void LlControllerNode::lpf(const Vec3 & measurement, Vec3 & estimate)
{
  estimate.x = LPF_tau_ * measurement.x + (1.0f - LPF_tau_) * estimate.x;
  estimate.y = LPF_tau_ * measurement.y + (1.0f - LPF_tau_) * estimate.y;
  estimate.z = LPF_tau_ * measurement.z + (1.0f - LPF_tau_) * estimate.z;
}

void LlControllerNode::rpy_from_quat(const geometry_msgs::msg::Quaternion & Q)
{
  const float q[4] = {
    static_cast<float>(Q.x),
    static_cast<float>(Q.y),
    static_cast<float>(Q.z),
    static_cast<float>(Q.w)};
  rpy_.x = std::asin(2.0f * (q[0] * q[2] - q[3] * q[1]));
  rpy_.y = std::atan2(
    2.0f * (q[0] * q[3] + q[1] * q[2]),
    1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
  rpy_.z = std::atan2(
    2.0f * (q[0] * q[1] + q[2] * q[3]),
    1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
}

void LlControllerNode::vesc_cb(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!vesc_init_) {
    vesc_init_ = true;
  }
  wheelspeed_ = static_cast<float>(msg->state.speed / erpm_gain_);
  voltage_input_ = static_cast<float>(msg->state.voltage_input);
  const float power_applied = std::fabs(
    static_cast<float>(msg->state.duty_cycle * msg->state.current_input));
  const float Kd_meas = power_applied / std::max(1.0f, wheelspeed_ * wheelspeed_);
  K_drag_ = std::min(1.0f, std::max(0.0f, 0.2f * Kd_meas + 0.8f * K_drag_));
}

void LlControllerNode::channel_cb(const mavros_msgs::msg::RCIn::SharedPtr msg)
{
  if (msg->channels.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);

  semi_steering_ = steering_max_ * ((msg->channels[0] - 1500.0f) / 500.0f);
  semi_wheelspeed_ = wheelspeed_max_ * ((msg->channels[2] - 1000.0f) / 1000.0f);

  manual_steering_ = steering_max_ * ((msg->channels[0] - 1500.0f) / 500.0f);
  manual_wheelspeed_ = wheelspeed_max_ * ((msg->channels[2] - 1000.0f) / 1000.0f);

  auto_wheelspeed_limit_ = wheelspeed_max_ * ((msg->channels[2] - 1000.0f) / 1000.0f);

  ackermann_msgs::msg::AckermannDriveStamped limits_msg;
  limits_msg.drive.speed = auto_wheelspeed_limit_;
  limits_pub_->publish(limits_msg);

  if (!channel_init_) {
    channel_init_ = true;
  }

  const int mode_switch = msg->channels[4];
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

void LlControllerNode::mode_cb(const mavros_msgs::msg::State::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!mode_init_) {
    mode_init_ = true;
  }
  guided_ = msg->guided && msg->armed;
}

void LlControllerNode::auto_control_cb(
  const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!auto_init_) {
    auto_init_ = true;
  }
  auto_steering_ = static_cast<float>(msg->drive.steering_angle);
  auto_wheelspeed_ = static_cast<float>(msg->drive.speed);
}

void LlControllerNode::imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
{
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

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!imu_init_) {
      imu_init_ = true;
    }

    const Vec3 rot_bf_meas{
      static_cast<float>(msg->angular_velocity.x),
      static_cast<float>(msg->angular_velocity.y),
      static_cast<float>(msg->angular_velocity.z)};
    const Vec3 acc_bf_meas{
      static_cast<float>(msg->linear_acceleration.x),
      static_cast<float>(msg->linear_acceleration.y),
      static_cast<float>(msg->linear_acceleration.z)};

    lpf(acc_bf_meas, acc_bf_);
    rot_bf_ = rot_bf_meas;
    rpy_from_quat(msg->orientation);

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
    safe_mode = safe_mode_;
  }

  if (!channel_init || !mode_init || !vesc_init || switch_pos == 0) {
    return;
  }
  if ((switch_pos == 2 && guided) && !auto_init) {
    return;
  }

  if (switch_pos >= 1 && guided) {
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
    publish_control(steering_setpoint / steering_max_, throttle_duty);
    publish_diagnostics(intervention, wheelspeed_setpoint, steering_setpoint);
  }
}

float LlControllerNode::speed_controller(float wheelspeed_setpoint)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  const float speed_error = (wheelspeed_setpoint - wheelspeed_) / max_rated_speed_;

  const float Kp_speed_error = speed_control_kp_ * speed_error;
  const float Ki_speed_error_dt = speed_control_ki_ * speed_error * delta_t_;

  speed_proportional_ = std::min(std::max(-0.05f, Kp_speed_error), 0.05f);
  speed_integral_ = std::min(
    std::max(-0.025f, Ki_speed_error_dt + speed_integral_), 0.025f);

  const float voltage_gain = nominal_voltage_ / voltage_input_;
  if (wheelspeed_setpoint < 0.1f * wheelspeed_max_) {
    speed_integral_ = 0.0f;
    speed_proportional_ = 0.0f;
  }

  float throttle_duty = voltage_gain * (
    (1.0f + K_drag_) * (wheelspeed_setpoint / max_rated_speed_) + speed_error +
    speed_integral_);
  throttle_duty = std::max(throttle_duty, 0.0f);

  throttle_duty = std::min(
    std::max(last_throttle_ - throttle_delta_, throttle_duty),
    last_throttle_ + throttle_delta_);
  last_throttle_ = throttle_duty;
  return throttle_duty;
}

float LlControllerNode::steering_limiter(float steering_setpoint, bool & intervention)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  intervention = false;
  float whspd2 = std::max(1.0f, wheelspeed_);
  whspd2 *= whspd2;

  float Aylim = track_width_ * 0.5f * std::max(1.0f, std::fabs(acc_bf_.z)) / cg_height_;
  if (liftoff_oversteer_) {
    Aylim = std::sqrt(std::max(Aylim * Aylim - acc_bf_.x * acc_bf_.x, 0.0f));
  }

  const float steering_input_temp = steering_setpoint;
  const float steering_limit_max =
    std::atan2(wheelbase_ * (Aylim - 9.81f * std::sin(rpy_.x)), whspd2) +
    steer_slack_ * steering_max_;
  const float steering_limit_min =
    -std::atan2(wheelbase_ * (Aylim + 9.81f * std::sin(rpy_.x)), whspd2) -
    steer_slack_ * steering_max_;

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
        (accel_gain_ * Ay_error - roll_gain_ * rot_bf_.x * std::fabs(acc_bf_.z)) *
        std::cos(steering_setpoint) * std::cos(steering_setpoint) * wheelbase_ / whspd2;
      delta_steering = std::min(delta_steering, 0.0f);
    } else {
      Ay_error = -Aylim - Ay;
      delta_steering =
        (accel_gain_ * Ay_error - roll_gain_ * rot_bf_.x * std::fabs(acc_bf_.z)) *
        std::cos(steering_setpoint) * std::cos(steering_setpoint) * wheelbase_ / whspd2;
      delta_steering = std::max(delta_steering, 0.0f);
    }
    steering_setpoint += delta_steering;
  }

  return steering_setpoint;
}

void LlControllerNode::publish_control(float steering_norm, float throttle_duty)
{
  mavros_msgs::msg::ManualControl manual_control_msg;
  manual_control_msg.header.stamp = now();
  manual_control_msg.x = 1000.0f;
  manual_control_msg.y = -steering_norm * 1000.0f;
  manual_control_msg.z = throttle_duty * 1000.0f;
  manual_control_msg.r = 1000.0f;
  manual_control_msg.buttons = 0;
  control_pub_->publish(manual_control_msg);
}

void LlControllerNode::publish_diagnostics(
  bool intervention, float wheelspeed_setpoint, float steering_setpoint)
{
  float wheelspeed = 0.0f;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    wheelspeed = wheelspeed_;
  }

  diagnostic_msgs::msg::DiagnosticArray dia_array;
  diagnostic_msgs::msg::DiagnosticStatus robot_status;
  robot_status.name = "LL_control";
  robot_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  robot_status.message = "intervention";

  diagnostic_msgs::msg::KeyValue steering;
  steering.key = "steering";
  steering.value = std::to_string(intervention);

  diagnostic_msgs::msg::KeyValue speed_error;
  speed_error.key = "speed_error";
  speed_error.value = std::to_string(wheelspeed_setpoint - wheelspeed);

  diagnostic_msgs::msg::KeyValue steering_input;
  steering_input.key = "steering_input";
  steering_input.value = std::to_string(steering_setpoint);

  diagnostic_msgs::msg::KeyValue wheelspeed_input;
  wheelspeed_input.key = "wheelspeed_input";
  wheelspeed_input.value = std::to_string(wheelspeed_setpoint);

  robot_status.values.push_back(steering);
  robot_status.values.push_back(speed_error);
  robot_status.values.push_back(steering_input);
  robot_status.values.push_back(wheelspeed_input);
  dia_array.status.push_back(robot_status);
  diagnostic_pub_->publish(dia_array);
}

}  // namespace hound_core

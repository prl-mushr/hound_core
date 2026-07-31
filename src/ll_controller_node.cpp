#include "hound_core/ll_controller_node.hpp"

#include <vector>

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

  RCLCPP_INFO(
    get_logger(),
    "Low-level controller ready (max_rated_speed=%.2f m/s)",
    controller_.max_rated_speed());
}

void LlControllerNode::declare_parameters()
{
  LlController::Params p;
  p.erpm_gain = static_cast<float>(declare_parameter("erpm_gain", 3500.0));
  p.steering_max = static_cast<float>(declare_parameter("steering_max", 0.488));
  p.wheelbase = static_cast<float>(declare_parameter("wheelbase", 0.29));
  p.cg_height = static_cast<float>(declare_parameter("cg_height", 0.136));
  p.wheelspeed_max = static_cast<float>(declare_parameter("wheelspeed_max", 17.0));
  p.nominal_voltage = static_cast<float>(declare_parameter("nominal_voltage", 14.8));
  p.motor_kv = static_cast<float>(declare_parameter("motor_kv", 3930.0));
  p.speed_control_kp = static_cast<float>(declare_parameter("speed_control_kp", 1.0));
  p.speed_control_ki = static_cast<float>(declare_parameter("speed_control_ki", 1.0));
  p.safe_mode = declare_parameter("safe_mode", true);
  p.track_width = static_cast<float>(declare_parameter("track_width", 0.25));
  p.accel_gain = static_cast<float>(declare_parameter("accel_gain", 1.0));
  p.roll_gain = static_cast<float>(declare_parameter("roll_gain", 0.33));
  p.steer_slack = static_cast<float>(declare_parameter("steer_slack", 0.4));
  p.LPF_tau = static_cast<float>(declare_parameter("LPF_tau", 0.2));
  p.throttle_delta = static_cast<float>(declare_parameter("throttle_delta", 0.02));
  p.liftoff_oversteer = declare_parameter("liftoff_oversteer", true);
  p.control_dt = static_cast<float>(declare_parameter("control_dt", 0.02));
  declare_parameter("executor_threads", 8);
  controller_.set_params(p);
}

void LlControllerNode::vesc_cb(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
{
  controller_.update_vesc(
    static_cast<float>(msg->state.speed),
    static_cast<float>(msg->state.voltage_input),
    static_cast<float>(msg->state.duty_cycle),
    static_cast<float>(msg->state.current_input));
}

void LlControllerNode::channel_cb(const mavros_msgs::msg::RCIn::SharedPtr msg)
{
  if (msg->channels.empty()) {
    return;
  }
  std::vector<float> ch(msg->channels.begin(), msg->channels.end());
  controller_.update_rc(ch.data(), ch.size());

  ackermann_msgs::msg::AckermannDriveStamped limits_msg;
  limits_msg.drive.speed = controller_.auto_wheelspeed_limit();
  limits_pub_->publish(limits_msg);
}

void LlControllerNode::mode_cb(const mavros_msgs::msg::State::SharedPtr msg)
{
  controller_.update_mode(msg->armed, msg->guided);
}

void LlControllerNode::auto_control_cb(
  const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
  controller_.update_auto(
    static_cast<float>(msg->drive.steering_angle),
    static_cast<float>(msg->drive.speed));
}

void LlControllerNode::imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  const float q[4] = {
    static_cast<float>(msg->orientation.w),
    static_cast<float>(msg->orientation.x),
    static_cast<float>(msg->orientation.y),
    static_cast<float>(msg->orientation.z)};
  const auto cmd = controller_.tick_imu(
    static_cast<float>(msg->angular_velocity.x),
    static_cast<float>(msg->angular_velocity.y),
    static_cast<float>(msg->angular_velocity.z),
    static_cast<float>(msg->linear_acceleration.x),
    static_cast<float>(msg->linear_acceleration.y),
    static_cast<float>(msg->linear_acceleration.z),
    q);
  if (!cmd.active) {
    return;
  }
  publish_control(cmd.steering_norm, cmd.throttle_duty);
  publish_diagnostics(cmd.intervention, cmd.wheelspeed_setpoint, cmd.steering_setpoint);
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
  speed_error.value = std::to_string(wheelspeed_setpoint);

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

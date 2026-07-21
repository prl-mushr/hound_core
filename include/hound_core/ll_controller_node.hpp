#ifndef HOUND_CORE__LL_CONTROLLER_NODE_HPP_
#define HOUND_CORE__LL_CONTROLLER_NODE_HPP_

#include <mutex>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "mavros_msgs/msg/manual_control.hpp"
#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"

namespace hound_core
{

struct Vec3
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
};

class LlControllerNode : public rclcpp::Node
{
public:
  explicit LlControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declare_parameters();
  void lpf(const Vec3 & measurement, Vec3 & estimate);
  void rpy_from_quat(const geometry_msgs::msg::Quaternion & q);
  float speed_controller(float wheelspeed_setpoint);
  float steering_limiter(float steering_setpoint, bool & intervention);
  void publish_control(float steering_norm, float throttle_duty);
  void publish_diagnostics(
    bool intervention, float wheelspeed_setpoint, float steering_setpoint);

  void vesc_cb(const vesc_msgs::msg::VescStateStamped::SharedPtr msg);
  void channel_cb(const mavros_msgs::msg::RCIn::SharedPtr msg);
  void mode_cb(const mavros_msgs::msg::State::SharedPtr msg);
  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg);
  void auto_control_cb(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr sub_vesc_;
  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr sub_channel_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr sub_mode_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr sub_auto_control_;

  rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr control_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr limits_pub_;

  mutable std::mutex state_mutex_;

  float wheelspeed_{0.0f};
  int switch_pos_{0};
  bool guided_{false};
  Vec3 rot_bf_;
  Vec3 acc_bf_;
  Vec3 rpy_;
  float semi_steering_{0.0f};
  float semi_wheelspeed_{0.0f};
  float auto_steering_{0.0f};
  float auto_wheelspeed_{0.0f};
  float auto_wheelspeed_limit_{0.0f};
  float manual_steering_{0.0f};
  float manual_wheelspeed_{0.0f};

  float erpm_gain_{3500.0f};
  float steering_max_{0.488f};
  float wheelspeed_max_{17.0f};
  float wheelbase_{0.29f};
  float cg_height_{0.136f};
  float track_width_{0.25f};
  bool channel_init_{false};
  bool vesc_init_{false};
  bool mode_init_{false};
  bool imu_init_{false};
  bool auto_init_{false};

  float motor_kv_{3930.0f};
  float nominal_voltage_{14.8f};
  float max_rated_speed_{0.0f};
  float voltage_input_{0.0f};
  float K_drag_{0.0f};

  float speed_integral_{0.0f};
  float speed_proportional_{0.0f};
  float delta_t_{0.02f};
  float last_throttle_{0.0f};
  float throttle_delta_{0.02f};
  float speed_control_kp_{1.0f};
  float speed_control_ki_{1.0f};

  bool safe_mode_{true};
  float accel_gain_{1.0f};
  float roll_gain_{0.33f};
  float steer_slack_{0.4f};
  float LPF_tau_{0.2f};
  bool liftoff_oversteer_{true};
};

}  // namespace hound_core

#endif  // HOUND_CORE__LL_CONTROLLER_NODE_HPP_

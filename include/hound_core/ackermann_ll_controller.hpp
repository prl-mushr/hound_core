#pragma once

#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/node.hpp>

#include "hound_core/fcu_slots.hpp"
#include "hound_core/low_level_controller.hpp"
#include "hound_core/safety.hpp"

namespace hound_core
{

class MavlinkBridge;

struct Vec3
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
};

/** Ackermann (RC car) low-level speed/steer controller. Self-actuates via MANUAL_CONTROL. */
class AckermannLlController : public LowLevelController
{
public:
  struct Params
  {
    float erpm_gain{3500.0f};
    float steering_max{0.488f};
    float wheelbase{0.29f};
    float cg_height{0.136f};
    float wheelspeed_max{17.0f};
    float nominal_voltage{14.8f};
    float motor_kv{3930.0f};
    float kv_correction_factor{0.69f};
    /** EMA weight on measured drag: K_drag = alpha*Kd_meas + (1-alpha)*K_drag. */
    float kd_alpha{0.2f};
    float speed_control_kp{1.0f};
    float speed_control_ki{1.0f};
    bool safe_mode{true};
    float track_width{0.25f};
    float accel_gain{1.0f};
    float roll_gain{0.33f};
    float steer_slack{0.4f};
    float LPF_tau{0.2f};
    float throttle_delta{0.02f};
    bool liftoff_oversteer{true};
    float control_dt{0.02f};
    /** RC ch2 PWM at 0 m/s (semi + auto speed limit). Typical stick-low = 1000. */
    float throttle_pwm_zero{1500.0f};
    /** RC ch2 PWM at wheelspeed_max. Typical stick-high = 2000. */
    float throttle_pwm_full{2000.0f};
  };

  struct ManualCommand
  {
    float steering_norm{0.0f};
    float throttle_duty{0.0f};
    bool active{false};
    bool intervention{false};
    float wheelspeed_setpoint{0.0f};
    float steering_setpoint{0.0f};
    float k_drag{0.0f};
    float wheelspeed_error{0.0f};
  };

  AckermannLlController(MavlinkBridge & bridge, const Params & params);

  void update_vesc(const VescSample & vesc) override;
  void update_rc(const RcSample & rc) override;
  void update_mode(const FcuStateSample & state) override;
  void update_auto(float steering_rad, float speed_mps);
  LlStatus tick_imu(const ImuSample & imu) override;
  bool plant_feedback(float & steering_rad, float & wheelspeed_mps) const override;

  /** Wire autonomy + /control_limits (VESC comes from FcuBus). */
  void setup_subscriptions(rclcpp::Node & node);

private:
  ManualCommand compute(const ImuSample & imu);
  void lpf(const Vec3 & measurement, Vec3 & estimate);
  void rpy_from_quat_wxyz(float w, float x, float y, float z);
  float speed_controller(float wheelspeed_setpoint);
  float steering_limiter(float steering_setpoint, bool & intervention);

  MavlinkBridge & bridge_;
  mutable std::mutex mu_;
  Params p_;
  SlewLimiter throttle_slew_;

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

  bool channel_init_{false};
  bool vesc_init_{false};
  bool mode_init_{false};
  bool imu_init_{false};
  bool auto_init_{false};

  float max_rated_speed_{0.0f};
  float voltage_input_{0.0f};
  float K_drag_{0.0f};
  float speed_integral_{0.0f};
  float speed_proportional_{0.0f};
  float last_steering_rad_{0.0f};

  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr auto_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr limits_pub_;
};

}  // namespace hound_core

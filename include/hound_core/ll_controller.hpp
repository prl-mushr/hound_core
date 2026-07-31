#pragma once

#include <cmath>
#include <mutex>
#include <string>

namespace hound_core
{

struct Vec3
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
};

/** Pure low-level speed/steer controller (no ROS). Thread-safe state updates. */
class LlController
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
  };

  struct ManualCommand
  {
    float steering_norm{0.0f};  // [-1, 1]
    float throttle_duty{0.0f};  // [0, ~1]
    bool active{false};
    bool intervention{false};
    float wheelspeed_setpoint{0.0f};
    float steering_setpoint{0.0f};
  };

  LlController();
  explicit LlController(const Params & params);

  void set_params(const Params & params);

  void update_vesc(float erpm, float voltage_input, float duty_cycle, float current_input);
  void update_rc(const float * channels, std::size_t n);
  void update_mode(bool armed, bool guided);
  void update_auto(float steering_rad, float speed_mps);
  /** IMU-paced tick. Orientation optional (w,x,y,z); if null, skip RPY update. */
  ManualCommand tick_imu(
    float gx, float gy, float gz, float ax, float ay, float az,
    const float * quat_wxyz = nullptr);

  float auto_wheelspeed_limit() const;
  float max_rated_speed() const { return max_rated_speed_; }

private:
  void lpf(const Vec3 & measurement, Vec3 & estimate);
  void rpy_from_quat_wxyz(float w, float x, float y, float z);
  float speed_controller(float wheelspeed_setpoint);
  float steering_limiter(float steering_setpoint, bool & intervention);

  mutable std::mutex mu_;
  Params p_;

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
  float last_throttle_{0.0f};
};

}  // namespace hound_core

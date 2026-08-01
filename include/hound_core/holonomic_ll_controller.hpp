#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/node.hpp>

#include "hound_core/fcu_slots.hpp"
#include "hound_core/low_level_controller.hpp"
#include "hound_core/pid.hpp"
#include "hound_core/safety.hpp"

namespace hound_core
{

class MavlinkBridge;

/**
 * Holonomic (mecanum) low-level controller. Decodes RC directly from FcuBus,
 * runs PID x/y/yaw, actuates via RC_CHANNELS_OVERRIDE + SET_MODE(HOLD) on fault.
 */
class HolonomicLlController : public LowLevelController
{
public:
  struct Params
  {
    PidConfig pid_vel_x;
    PidConfig pid_vel_y;
    PidConfig pid_yaw_rate;

    int input_ch_mode{4};
    int input_ch_fwd{1};
    int input_ch_lat{0};
    int input_ch_yaw{3};
    float mode_pwm_manual_threshold{1300.0f};
    float mode_pwm_auto_threshold{1700.0f};
    float input_deadband{0.05f};
    float vel_max_linear{1.0f};
    float vel_max_angular{1.57f};

    int output_ch_fwd{8};
    int output_ch_lat{9};
    int output_ch_turn{10};

    double cmd_vel_timeout{0.5};
    /** ArduPilot Rover HOLD custom_mode (default 4). */
    uint32_t mode_hold_custom_mode{4};
    /**
     * When false (default), linear x/y use feedforward only with measured
     * velocity treated as zero — matches current opendubs no-odom behavior.
     */
    bool use_odom_feedback{false};
  };

  enum class SwitchPos : int
  {
    Off = 0,
    Manual = 1,
    Auto = 2,
  };

  struct HolonomicCommand
  {
    bool active{false};
    bool fault{false};
    float fwd{0.0f};
    float lat{0.0f};
    float yaw{0.0f};
    std::optional<uint32_t> requested_custom_mode;
  };

  HolonomicLlController(MavlinkBridge & bridge, const Params & params);

  void set_params(const Params & params);
  void update_rc(const RcSample & rc) override;
  void update_mode(const FcuStateSample & state) override;
  void update_auto_cmd_vel(float vx, float vy, float wz);
  void update_auto_cmd_vel(float vx, float vy, float wz, const rclcpp::Time & stamp);
  /** Optional body-frame linear velocity feedback for closed-loop x/y. */
  void update_odom_velocity(float vx, float vy);

  /** Pure compute path (unit-testable without requiring a live FCU link). */
  HolonomicCommand compute(const ImuSample & imu, const rclcpp::Time & now);
  LlStatus tick_imu(const ImuSample & imu) override;

  void setup_subscriptions(rclcpp::Node & node);

  SwitchPos switch_pos() const;

private:
  static float apply_deadband(float norm, float deadband);
  static float pwm_to_norm(float pwm);
  uint16_t effort_to_pwm(float effort) const;
  void publish_override(float fwd, float lat, float yaw);
  void publish_neutral_and_hold();

  MavlinkBridge & bridge_;
  mutable std::mutex mu_;
  Params p_;
  Pid pid_x_;
  Pid pid_y_;
  Pid pid_yaw_;
  Watchdog auto_cmd_watchdog_;
  Watchdog rc_watchdog_;

  SwitchPos switch_pos_{SwitchPos::Off};
  bool guided_{false};
  bool mode_init_{false};
  bool channel_init_{false};
  bool auto_init_{false};

  float semi_fwd_{0.0f};
  float semi_lat_{0.0f};
  float semi_yaw_{0.0f};
  float auto_vx_{0.0f};
  float auto_vy_{0.0f};
  float auto_wz_{0.0f};
  float odom_vx_{0.0f};
  float odom_vy_{0.0f};
  bool odom_init_{false};

  rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr auto_sub_;
};

}  // namespace hound_core

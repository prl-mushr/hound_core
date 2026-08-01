#pragma once

#include <algorithm>
#include <cmath>

namespace hound_core
{

struct PidConfig
{
  double k_p{0.0};
  double k_i{0.0};
  double k_d{0.0};
  double k_ff{0.0};
  /** Physical velocity scale (m/s or rad/s). Normalizes FF: (target/max)*k_ff. */
  double max_output{1.0};
  /** Anti-windup clamp on the integral state. */
  double integrity_limit{1.0};
};

/**
 * Generic P/I/D/FF controller returning normalized effort in [-1, 1].
 * Ported from opendubs-core PidController with ROS publisher coupling removed.
 */
class Pid
{
public:
  void configure(const PidConfig & config) {config_ = config;}
  void reset()
  {
    integral_ = 0.0;
    prev_error_ = 0.0;
  }

  /**
   * @param target   Desired value in physical units.
   * @param feedback Measured value in the same units.
   * @param dt       Seconds since last update.
   * @return Normalized control effort clamped to [-1, 1].
   */
  double calculate(double target, double feedback, double dt)
  {
    const double capped_target =
      std::clamp(target, -config_.max_output, config_.max_output);
    const double error = capped_target - feedback;

    integral_ += error * dt;
    integral_ = std::clamp(integral_, -config_.integrity_limit, config_.integrity_limit);

    double derivative = 0.0;
    if (dt > 1e-6) {
      derivative = (error - prev_error_) / dt;
    }
    prev_error_ = error;

    double output =
      (config_.k_p * error) + (config_.k_i * integral_) + (config_.k_d * derivative);
    if (config_.max_output > 1e-9) {
      output += (capped_target / config_.max_output) * config_.k_ff;
    }
    return std::clamp(output, -1.0, 1.0);
  }

  const PidConfig & config() const {return config_;}

private:
  PidConfig config_;
  double integral_{0.0};
  double prev_error_{0.0};
};

}  // namespace hound_core

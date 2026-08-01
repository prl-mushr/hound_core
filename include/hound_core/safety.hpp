#pragma once

#include <algorithm>
#include <cmath>

#include <rclcpp/time.hpp>

namespace hound_core
{

/** Tracks last-update time; reports staleness against a threshold. */
class Watchdog
{
public:
  void stamp(const rclcpp::Time & now) {last_ = now; has_stamp_ = true;}

  bool is_stale(const rclcpp::Time & now, double threshold_s) const
  {
    if (!has_stamp_) {
      return true;
    }
    return (now - last_).seconds() > threshold_s;
  }

  bool has_stamp() const {return has_stamp_;}
  rclcpp::Time last() const {return last_;}

private:
  rclcpp::Time last_{0, 0, RCL_ROS_TIME};
  bool has_stamp_{false};
};

/** Max-delta-per-tick clamp (slew-rate limiter). */
class SlewLimiter
{
public:
  explicit SlewLimiter(float max_delta = 0.02f)
  : max_delta_(max_delta) {}

  void set_max_delta(float max_delta) {max_delta_ = max_delta;}
  void reset(float value = 0.0f) {last_ = value;}

  float apply(float desired)
  {
    const float lo = last_ - max_delta_;
    const float hi = last_ + max_delta_;
    last_ = std::clamp(desired, lo, hi);
    return last_;
  }

  float value() const {return last_;}

private:
  float max_delta_{0.02f};
  float last_{0.0f};
};

}  // namespace hound_core

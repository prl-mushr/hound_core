#include <gtest/gtest.h>

#include <rclcpp/time.hpp>

#include "hound_core/pid.hpp"
#include "hound_core/safety.hpp"

using hound_core::Pid;
using hound_core::PidConfig;
using hound_core::SlewLimiter;
using hound_core::Watchdog;

TEST(PidTest, FeedforwardOnly)
{
  Pid pid;
  PidConfig cfg;
  cfg.k_ff = 1.0;
  cfg.max_output = 1.0;
  pid.configure(cfg);
  EXPECT_NEAR(pid.calculate(1.0, 0.0, 0.02), 1.0, 1e-6);
  EXPECT_NEAR(pid.calculate(0.5, 0.0, 0.02), 0.5, 1e-6);
  EXPECT_NEAR(pid.calculate(-1.0, 0.0, 0.02), -1.0, 1e-6);
}

TEST(PidTest, ProportionalAndClamp)
{
  Pid pid;
  PidConfig cfg;
  cfg.k_p = 2.0;
  cfg.max_output = 10.0;
  pid.configure(cfg);
  // error=1 → raw 2.0 → clamp to 1
  EXPECT_NEAR(pid.calculate(1.0, 0.0, 0.02), 1.0, 1e-6);
}

TEST(PidTest, AntiWindup)
{
  Pid pid;
  PidConfig cfg;
  cfg.k_i = 10.0;
  cfg.integrity_limit = 0.1;
  cfg.max_output = 10.0;
  pid.configure(cfg);
  for (int i = 0; i < 50; ++i) {
    pid.calculate(1.0, 0.0, 0.02);
  }
  // Integral capped; output still within [-1, 1]
  const double out = pid.calculate(1.0, 0.0, 0.02);
  EXPECT_LE(out, 1.0);
  EXPECT_GE(out, -1.0);
}

TEST(WatchdogTest, StaleUntilStamped)
{
  Watchdog wd;
  const rclcpp::Time t0(1, 0, RCL_ROS_TIME);
  EXPECT_TRUE(wd.is_stale(t0, 0.5));
  wd.stamp(t0);
  EXPECT_FALSE(wd.is_stale(t0, 0.5));
  const rclcpp::Time t1(1, 600000000, RCL_ROS_TIME);  // +0.6s
  EXPECT_TRUE(wd.is_stale(t1, 0.5));
}

TEST(SlewLimiterTest, LimitsDelta)
{
  SlewLimiter slew(0.1f);
  EXPECT_NEAR(slew.apply(1.0f), 0.1f, 1e-5);
  EXPECT_NEAR(slew.apply(1.0f), 0.2f, 1e-5);
  EXPECT_NEAR(slew.apply(0.0f), 0.1f, 1e-5);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

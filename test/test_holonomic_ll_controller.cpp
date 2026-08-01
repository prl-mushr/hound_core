#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "hound_core/holonomic_ll_controller.hpp"
#include "hound_core/mavlink_bridge.hpp"

using hound_core::FcuStateSample;
using hound_core::HolonomicLlController;
using hound_core::ImuSample;
using hound_core::MavlinkBridge;
using hound_core::RcSample;

namespace
{

HolonomicLlController::Params test_params()
{
  HolonomicLlController::Params p;
  p.pid_vel_x.k_ff = 1.0;
  p.pid_vel_x.max_output = 1.0;
  p.pid_vel_y.k_ff = 1.0;
  p.pid_vel_y.max_output = 1.0;
  p.pid_yaw_rate.k_p = 0.0;
  p.pid_yaw_rate.k_ff = 1.0;
  p.pid_yaw_rate.max_output = 1.57;
  p.cmd_vel_timeout = 0.5;
  p.use_odom_feedback = false;
  return p;
}

RcSample make_rc(float mode_pwm, float fwd = 1500.0f, float lat = 1500.0f, float yaw = 1500.0f)
{
  RcSample rc;
  rc.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
  rc.nchan = 8;
  rc.channels.fill(1500.0f);
  rc.channels[0] = lat;
  rc.channels[1] = fwd;
  rc.channels[3] = yaw;
  rc.channels[4] = mode_pwm;
  return rc;
}

}  // namespace

class HolonomicLlControllerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    bridge_ = std::make_unique<MavlinkBridge>(
      MavlinkBridge::Config{}, rclcpp::get_logger("test"),
      std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    ctrl_ = std::make_unique<HolonomicLlController>(*bridge_, test_params());
    FcuStateSample st;
    st.armed = true;
    st.guided = true;
    ctrl_->update_mode(st);
  }

  void TearDown() override
  {
    ctrl_.reset();
    bridge_.reset();
    rclcpp::shutdown();
  }

  std::unique_ptr<MavlinkBridge> bridge_;
  std::unique_ptr<HolonomicLlController> ctrl_;
};

TEST_F(HolonomicLlControllerTest, ModeSwitchDecode)
{
  ctrl_->update_rc(make_rc(1000.0f));
  EXPECT_EQ(ctrl_->switch_pos(), HolonomicLlController::SwitchPos::Manual);
  ctrl_->update_rc(make_rc(1500.0f));
  EXPECT_EQ(ctrl_->switch_pos(), HolonomicLlController::SwitchPos::Off);
  ctrl_->update_rc(make_rc(1900.0f));
  EXPECT_EQ(ctrl_->switch_pos(), HolonomicLlController::SwitchPos::Auto);
}

TEST_F(HolonomicLlControllerTest, ManualPassthrough)
{
  // fwd stick high → ~+1 effort
  ctrl_->update_rc(make_rc(1000.0f, 2000.0f, 1500.0f, 1500.0f));
  ImuSample imu;
  imu.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
  const auto cmd = ctrl_->compute(imu, imu.stamp);
  EXPECT_TRUE(cmd.active);
  EXPECT_FALSE(cmd.fault);
  EXPECT_NEAR(cmd.fwd, 1.0f, 1e-3);
  EXPECT_NEAR(cmd.lat, 0.0f, 1e-3);
}

TEST_F(HolonomicLlControllerTest, AutoPidFeedforward)
{
  ctrl_->update_rc(make_rc(1900.0f));
  ctrl_->update_auto_cmd_vel(1.0f, 0.0f, 0.0f, rclcpp::Time(1, 0, RCL_ROS_TIME));
  ImuSample imu;
  imu.stamp = rclcpp::Time(1, 10000000, RCL_ROS_TIME);  // +10ms
  const auto cmd = ctrl_->compute(imu, imu.stamp);
  EXPECT_TRUE(cmd.active);
  EXPECT_NEAR(cmd.fwd, 1.0f, 1e-3);
}

TEST_F(HolonomicLlControllerTest, StaleAutoTriggersFault)
{
  ctrl_->update_rc(make_rc(1900.0f));
  ctrl_->update_auto_cmd_vel(1.0f, 0.0f, 0.0f, rclcpp::Time(1, 0, RCL_ROS_TIME));
  ImuSample imu;
  // RC also needs a fresh stamp — restamp RC at same time as auto, then advance past timeout
  auto rc = make_rc(1900.0f);
  rc.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
  ctrl_->update_rc(rc);
  imu.stamp = rclcpp::Time(2, 0, RCL_ROS_TIME);  // +1s > 0.5s timeout
  // Keep RC fresh so only auto is stale
  rc.stamp = imu.stamp;
  ctrl_->update_rc(rc);
  const auto cmd = ctrl_->compute(imu, imu.stamp);
  EXPECT_TRUE(cmd.fault);
  EXPECT_TRUE(cmd.requested_custom_mode.has_value());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

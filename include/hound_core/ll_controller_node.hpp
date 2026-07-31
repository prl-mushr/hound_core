#ifndef HOUND_CORE__LL_CONTROLLER_NODE_HPP_
#define HOUND_CORE__LL_CONTROLLER_NODE_HPP_

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hound_core/ll_controller.hpp"
#include "mavros_msgs/msg/manual_control.hpp"
#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"

namespace hound_core
{

class LlControllerNode : public rclcpp::Node
{
public:
  explicit LlControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declare_parameters();
  void publish_control(float steering_norm, float throttle_duty);
  void publish_diagnostics(
    bool intervention, float wheelspeed_setpoint, float steering_setpoint);

  void vesc_cb(const vesc_msgs::msg::VescStateStamped::SharedPtr msg);
  void channel_cb(const mavros_msgs::msg::RCIn::SharedPtr msg);
  void mode_cb(const mavros_msgs::msg::State::SharedPtr msg);
  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg);
  void auto_control_cb(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

  LlController controller_;

  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr sub_vesc_;
  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr sub_channel_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr sub_mode_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr sub_auto_control_;

  rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr control_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr limits_pub_;
};

}  // namespace hound_core

#endif  // HOUND_CORE__LL_CONTROLLER_NODE_HPP_

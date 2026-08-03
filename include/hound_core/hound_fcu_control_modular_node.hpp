#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include "hound_core/ekf_runner.hpp"
#include "hound_core/fcu_slots.hpp"
#include "hound_core/ll_controller_registry.hpp"
#include "hound_core/ll_runner.hpp"
#include "hound_core/low_level_controller.hpp"
#include "hound_core/mavlink_bridge.hpp"
#include "hound_core/vesc_runner.hpp"

namespace hound_core
{

/**
 * Thin ROS wire-up for the modular FCU path: params, edge pubs/subs, and
 * MavlinkBridge + EkfRunner + pluggable LlRunner over a shared FcuBus.
 *
 * High-rate ROS edge (imu/mag/baro/gps fix/ap odom) runs on a wall timer.
 * Low-rate aux (RC, armed, GPS sats/h_acc, mission Path, play_tune RX) runs
 * on a dedicated thread so it never stalls the high-rate path.
 */
class HoundFcuControlModularNode : public rclcpp::Node
{
public:
  explicit HoundFcuControlModularNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~HoundFcuControlModularNode() override;

private:
  void declare_params();
  void ros_edge_timer();
  void aux_loop();
  void play_tune_cb(const std_msgs::msg::String::SharedPtr msg);
  void ekf_reset_cb(const std_msgs::msg::Empty::SharedPtr msg);

  void vision_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void icp_origin_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  nav_msgs::msg::Path mission_to_path(const std::vector<MissionItem> & items) const;

  std::string fcu_url_;
  std::string gcs_url_;
  double ros_publish_hz_{50.0};
  double aux_publish_hz_{10.0};
  bool gcs_block_stream_requests_{true};
  double gcs_throttle_hz_{10.0};
  std::vector<int64_t> gcs_throttle_msgids_;
  std::map<std::string, double> fcu_params_;
  bool send_vision_to_fcu_{true};
  bool enable_ekf_{true};
  bool enable_ll_{true};
  bool enable_baro_{true};
  bool enable_mag_{true};
  bool enable_gps_{true};
  bool fuse_gps_{false};
  int ekf_odom_hz_{50};
  double mag_max_hz_{20.0};
  double baro_max_hz_{20.0};
  std::string ext_nav_align_{"gps_compass"};
  std::string icp_origin_topic_{"/localization/icp_origin"};
  std::string vision_odom_topic_;
  std::string ekf_odom_topic_;
  std::string ekf_reset_topic_{"~/ekf_reset"};
  double ext_nav_origin_lat_{0.0};
  double ext_nav_origin_lon_{0.0};
  double ext_nav_origin_hgt_{0.0};
  int ekf_cpu_{2};
  int ll_cpu_{3};
  uint8_t system_id_{255};
  uint8_t component_id_{191};
  uint8_t target_system_{1};
  uint8_t target_component_{1};
  std::string ll_controller_name_{"ackermann"};
  bool vesc_enabled_{false};
  std::string vesc_port_{"/dev/ttyACM0"};
  double vesc_telemetry_hz_{200.0};

  FcuBus bus_;
  std::unique_ptr<MavlinkBridge> bridge_;
  std::unique_ptr<EkfRunner> ekf_runner_;
  std::unique_ptr<LlRunner> ll_runner_;
  std::unique_ptr<LowLevelController> ll_controller_;
  std::unique_ptr<VescRunner> vesc_runner_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr baro_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ap_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mission_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr rc_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr gps_sats_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr gps_h_acc_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr control_state_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vision_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr icp_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr play_tune_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr ekf_reset_sub_;
  rclcpp::TimerBase::SharedPtr edge_timer_;
  rclcpp::TimerBase::SharedPtr hb_timer_;
  rclcpp::TimerBase::SharedPtr boot_timer_;

  std::atomic<bool> aux_running_{false};
  std::thread aux_thread_;
};

}  // namespace hound_core

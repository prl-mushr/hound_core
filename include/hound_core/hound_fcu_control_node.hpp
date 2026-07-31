#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavconn/interface.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include "hound_core/fcu_slots.hpp"
#include "hound_core/ll_controller.hpp"

namespace hound_core
{

/**
 * Single-process FCU owner: MAVLink R/W + GCS bridge + EKF + LL controller.
 * High-rate sensors stay in-process (LatestSlot); ROS edge ~50 Hz.
 */
class HoundFcuControlNode : public rclcpp::Node
{
public:
  explicit HoundFcuControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~HoundFcuControlNode() override;

private:
  void declare_params();
  void open_links();
  void on_fcu_message(const mavlink::mavlink_message_t * msg, mavconn::Framing framing);
  void on_gcs_message(const mavlink::mavlink_message_t * msg, mavconn::Framing framing);
  void send_to_fcu(const mavlink::Message & msg);
  void maybe_forward_to_gcs(const mavlink::mavlink_message_t & msg);

  void handle_heartbeat(const mavlink::mavlink_message_t & msg);
  void handle_raw_imu(const mavlink::mavlink_message_t & msg);
  void handle_scaled_imu(const mavlink::mavlink_message_t & msg);
  void handle_attitude_quat(const mavlink::mavlink_message_t & msg);
  void handle_scaled_pressure(const mavlink::mavlink_message_t & msg);
  void handle_gps_raw(const mavlink::mavlink_message_t & msg);
  void handle_rc_channels(const mavlink::mavlink_message_t & msg);
  void handle_local_position(const mavlink::mavlink_message_t & msg);
  void handle_mission_count(const mavlink::mavlink_message_t & msg);
  void handle_mission_item(const mavlink::mavlink_message_t & msg);
  void handle_param_value(const mavlink::mavlink_message_t & msg);

  void boot_configure();
  void push_fcu_params();
  void request_data_streams();
  void request_mission_list();
  void send_heartbeat();
  void send_vision_to_fcu(const ExtNavSample & nav);
  void send_manual_control(const ManualControlCmd & cmd);

  void ll_worker();
  void ekf_worker();
  void ros_edge_timer();

  void vision_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void icp_origin_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void vesc_cb(const vesc_msgs::msg::VescStateStamped::SharedPtr msg);
  void auto_control_cb(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

  // Params
  std::string fcu_url_;
  std::string gcs_url_;
  double ros_publish_hz_{50.0};
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
  /** gps_compass | lidar_icp — how map/ext-nav frame is oriented at boot. */
  std::string ext_nav_align_{"gps_compass"};
  std::string icp_origin_topic_{"/localization/icp_origin"};
  std::string vision_odom_topic_;
  std::string ekf_odom_topic_;
  double ext_nav_origin_lat_{0.0};
  double ext_nav_origin_lon_{0.0};
  double ext_nav_origin_hgt_{0.0};
  int ekf_cpu_{2};
  int ll_cpu_{3};
  uint8_t system_id_{255};
  uint8_t component_id_{191};
  uint8_t target_system_{1};
  uint8_t target_component_{1};

  FcuBus bus_;
  LlController ll_;
  std::atomic<bool> running_{false};
  std::atomic<bool> fcu_seen_{false};
  std::atomic<bool> boot_done_{false};

  mavconn::MAVConnInterface::Ptr fcu_;
  mavconn::MAVConnInterface::Ptr gcs_;

  std::thread ll_thread_;
  std::thread ekf_thread_;

  // Attitude from ATTITUDE_QUATERNION (FLU) for imu/data-style orientation
  std::mutex att_mu_;
  float att_qw_{1}, att_qx_{0}, att_qy_{0}, att_qz_{0};
  bool att_valid_{false};

  // Mission download
  std::mutex mission_mu_;
  uint16_t mission_count_{0};
  uint16_t mission_next_{0};
  std::vector<mavros_msgs::msg::Waypoint> mission_items_;
  bool mission_dirty_{false};

  // GCS throttle
  std::mutex gcs_throttle_mu_;
  std::unordered_map<uint32_t, std::chrono::steady_clock::time_point> gcs_last_fwd_;
  std::unordered_set<uint32_t> gcs_throttle_ids_;

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr baro_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ap_odom_pub_;
  rclcpp::Publisher<mavros_msgs::msg::WaypointList>::SharedPtr mission_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr limits_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vision_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr icp_sub_;
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr auto_sub_;
  rclcpp::TimerBase::SharedPtr edge_timer_;
  rclcpp::TimerBase::SharedPtr hb_timer_;
  rclcpp::TimerBase::SharedPtr boot_timer_;
};

}  // namespace hound_core

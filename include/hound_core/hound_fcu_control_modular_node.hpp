#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "hound_core/ekf_runner.hpp"
#include "hound_core/fcu_slots.hpp"
#include "hound_core/ll_controller_registry.hpp"
#include "hound_core/ll_runner.hpp"
#include "hound_core/low_level_controller.hpp"
#include "hound_core/mavlink_bridge.hpp"

namespace hound_core
{

/**
 * Thin ROS wire-up for the modular FCU path: params, edge pubs/subs, and
 * MavlinkBridge + EkfRunner + pluggable LlRunner over a shared FcuBus.
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

  void vision_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void icp_origin_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

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
  std::string ll_controller_name_{"ackermann"};

  FcuBus bus_;
  std::unique_ptr<MavlinkBridge> bridge_;
  std::unique_ptr<EkfRunner> ekf_runner_;
  std::unique_ptr<LlRunner> ll_runner_;
  std::unique_ptr<LowLevelController> ll_controller_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr baro_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ap_odom_pub_;
  rclcpp::Publisher<mavros_msgs::msg::WaypointList>::SharedPtr mission_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vision_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr icp_sub_;
  rclcpp::TimerBase::SharedPtr edge_timer_;
  rclcpp::TimerBase::SharedPtr hb_timer_;
  rclcpp::TimerBase::SharedPtr boot_timer_;
};

}  // namespace hound_core

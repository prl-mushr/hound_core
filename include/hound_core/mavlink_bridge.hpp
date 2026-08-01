#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <mavconn/interface.hpp>
#include <mavros_msgs/msg/waypoint.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

#include "hound_core/fcu_slots.hpp"
#include "hound_core/sensor_board.hpp"

namespace hound_core
{

/**
 * Owns FCU/GCS libmavconn links and MAVLink SensorBoard decode into FcuBus.
 * Also handles TX (heartbeat, boot params/streams, vision, manual control)
 * and optional GCS forward.
 */
class MavlinkBridge : public SensorBoard
{
public:
  struct Config
  {
    std::string fcu_url{"/dev/ttyACM1:921600"};
    std::string gcs_url;
    bool gcs_block_stream_requests{true};
    double gcs_throttle_hz{10.0};
    std::vector<int64_t> gcs_throttle_msgids{27, 30, 31, 32, 33, 65};
    bool send_vision_to_fcu{true};
    std::map<std::string, double> fcu_params;
    uint8_t system_id{255};
    uint8_t component_id{191};
    uint8_t target_system{1};
    uint8_t target_component{1};
  };

  MavlinkBridge(
    const Config & config, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);
  ~MavlinkBridge() override;

  MavlinkBridge(const MavlinkBridge &) = delete;
  MavlinkBridge & operator=(const MavlinkBridge &) = delete;

  void start(FcuBus & bus) override;
  void stop() override;

  bool fcu_seen() const { return fcu_seen_.load(); }
  bool boot_done() const { return boot_done_.load(); }

  void boot_configure();
  void send_heartbeat();
  void send_vision(const ExtNavSample & nav);
  void send_manual_control(const ManualControlCmd & cmd);
  void send_rc_override(const RcOverrideCmd & cmd);
  /** Rate-limited (1 Hz) SET_MODE with custom_mode (e.g. Rover HOLD = 4). */
  void request_mode(uint32_t custom_mode);

  /** If a full mission download finished, copies waypoints and clears dirty. */
  bool take_mission(std::vector<mavros_msgs::msg::Waypoint> & out);

private:
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

  void push_fcu_params();
  void request_data_streams();
  void request_mission_list();

  rclcpp::Time now() const { return clock_->now(); }

  Config cfg_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  FcuBus * bus_{nullptr};
  std::atomic<bool> started_{false};
  std::atomic<bool> fcu_seen_{false};
  std::atomic<bool> boot_done_{false};

  mavconn::MAVConnInterface::Ptr fcu_;
  mavconn::MAVConnInterface::Ptr gcs_;

  std::mutex att_mu_;
  float att_qw_{1}, att_qx_{0}, att_qy_{0}, att_qz_{0};
  bool att_valid_{false};

  std::mutex mission_mu_;
  uint16_t mission_count_{0};
  uint16_t mission_next_{0};
  std::vector<mavros_msgs::msg::Waypoint> mission_items_;
  bool mission_dirty_{false};

  std::mutex gcs_throttle_mu_;
  std::unordered_map<uint32_t, std::chrono::steady_clock::time_point> gcs_last_fwd_;
  std::unordered_set<uint32_t> gcs_throttle_ids_;

  std::mutex mode_mu_;
  std::chrono::steady_clock::time_point last_mode_request_{};
};

}  // namespace hound_core

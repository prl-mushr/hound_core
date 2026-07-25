#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <librealsense2/rs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "cuvslam/cuvslam2.h"
#include "hound_core/rs_pipeline.hpp"

namespace hound_core {

/**
 * Single-process RealSense D455 owner: IR stereo → cuVSLAM (VO+SLAM),
 * optional color + depth publish at capped FPS. IR never goes to DDS.
 *
 * Stereo (IR ± depth) and RGB use separate sensors/queues so Track is not
 * blocked by color frameset sync. Align (if enabled) runs on the depth worker.
 */
class RealsenseCuvslamNode : public rclcpp::Node
{
public:
  explicit RealsenseCuvslamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RealsenseCuvslamNode() override;

private:
  struct LatestSlot {
    std::mutex mu;
    std::condition_variable cv_pose;
    std::condition_variable cv_color;
    std::condition_variable cv_depth;

    bool has_pose{false};
    uint64_t pose_seq{0};
    // ROS-time stamp for odom/TF (nvblox TF lookups + delay stats).
    rclcpp::Time ros_stamp{0, 0, RCL_ROS_TIME};
    cuvslam::Pose pose{};

    // Latest stereo frameset (IR ± depth) for depth worker.
    bool has_stereo{false};
    rs2::frameset stereo_fs;
    rclcpp::Time stereo_stamp{0, 0, RCL_ROS_TIME};

    // Latest color frame (independent sensor); seq bumps only when kept for publish.
    bool has_color{false};
    uint64_t color_seq{0};
    rs2::frame color_frame;
    rclcpp::Time color_stamp{0, 0, RCL_ROS_TIME};
  };

  void declare_params();
  void start_pipeline();
  void publish_static_tfs();
  void capture_loop();
  void odom_worker();
  void color_worker();
  void depth_worker();
  void publish_odom_tf(const rclcpp::Time & stamp, const cuvslam::Pose & pose);

  // Params (startup-only)
  std::string serial_number_;
  std::string camera_name_;
  int infra_width_{640};
  int infra_height_{360};
  int infra_fps_{60};
  bool enable_color_{true};
  int color_width_{640};
  int color_height_{360};
  int color_fps_{30};
  double color_publish_fps_{15.0};
  bool enable_depth_{false};
  bool align_depth_{true};
  int depth_width_{640};
  int depth_height_{360};
  int depth_fps_{30};
  double depth_publish_fps_{15.0};
  int emitter_enabled_{0};
  int visual_preset_{3};  // rs400: 3 = high_accuracy
  double clip_distance_{0.0};
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool async_sba_{true};
  bool slam_sync_mode_{false};
  int warmup_frames_{60};

  // ROS pubs
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  sensor_msgs::msg::CameraInfo color_camera_info_;
  sensor_msgs::msg::CameraInfo depth_camera_info_;
  std::string color_optical_frame_;
  std::string depth_optical_frame_;

  // RealSense / cuVSLAM (decoupled sensors)
  RsSensorStreams rs_{};
  std::unique_ptr<rs2::align> align_to_color_;
  rs2::syncer depth_color_sync_{4};
  std::unique_ptr<cuvslam::Odometry> odometry_;
  std::unique_ptr<cuvslam::Slam> slam_;

  LatestSlot slot_;
  std::atomic<bool> running_{false};
  std::thread capture_thread_;
  std::thread odom_thread_;
  std::thread color_thread_;
  std::thread depth_thread_;

  // Finite-diff twist state (odom worker)
  std::array<double, 3> prev_t_flu_{{0, 0, 0}};
  rclcpp::Time prev_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace hound_core

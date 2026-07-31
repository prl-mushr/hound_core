#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <librealsense2/rs.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "hound_core/camera_device.hpp"
#include "hound_core/rs_pipeline.hpp"

namespace hound_core {

/**
 * CameraDevice over decoupled RealSense stereo (+ optional color/depth)
 * using rs_pipeline.hpp helpers. Same ownership model as the legacy node:
 * IR Track is not gated on color frameset sync.
 */
class RealsenseCameraDevice : public CameraDevice
{
public:
  RealsenseCameraDevice() = default;
  ~RealsenseCameraDevice() override;

  void open(const CameraOpenConfig & config) override;
  StreamCaps caps() const override;
  StereoCalibration stereo_calibration() const override;

  bool wait_stereo(StereoFrame & out, int timeout_ms) override;
  bool poll_color(ColorFrame & out) override;
  bool poll_depth(DepthFrame & out) override;
  void stop() override;

  /** Cached CameraInfo for ROS publish (stamp filled by caller). */
  const sensor_msgs::msg::CameraInfo & color_camera_info() const;
  const sensor_msgs::msg::CameraInfo & depth_camera_info() const;

  /** Static camera_link / optical TFs (realsense2_camera naming). */
  std::vector<geometry_msgs::msg::TransformStamped> static_camera_tfs(
    const std::string & camera_name) const;

  const RsStreamProfiles & profiles() const { return rs_.profiles; }

private:
  CameraIntrinsics intrinsics_from_rs(const rs2_intrinsics & intr) const;
  int64_t stereo_timestamp_ns(const rs2::video_frame & left, int frame_id) const;

  RsSensorStreams rs_{};
  StreamCaps caps_{};
  StereoCalibration stereo_cal_{};
  CameraOpenConfig config_{};

  std::unique_ptr<rs2::align> align_to_color_;
  rs2::syncer depth_color_sync_{4};

  sensor_msgs::msg::CameraInfo color_camera_info_;
  sensor_msgs::msg::CameraInfo depth_camera_info_;
  std::string color_optical_frame_;
  std::string depth_optical_frame_;

  mutable std::mutex mu_;
  rs2::frameset latest_stereo_;
  rs2::frame left_frame_;   // keeps wait_stereo left pixels alive
  rs2::frame right_frame_;  // keeps wait_stereo right pixels alive
  bool has_stereo_{false};
  rs2::frame latest_color_;
  bool has_color_{false};
  rs2::frame latest_depth_frame_;  // keeps poll_depth pixels alive
  int stereo_frame_id_{0};
};

}  // namespace hound_core

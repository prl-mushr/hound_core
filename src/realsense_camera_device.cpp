#include "hound_core/realsense_camera_device.hpp"

#include <algorithm>
#include <stdexcept>

#include "hound_core/camera_info_utils.hpp"

namespace hound_core {

RealsenseCameraDevice::~RealsenseCameraDevice()
{
  stop();
}

CameraIntrinsics RealsenseCameraDevice::intrinsics_from_rs(
  const rs2_intrinsics & intr) const
{
  CameraIntrinsics out;
  out.width = intr.width;
  out.height = intr.height;
  out.fx = intr.fx;
  out.fy = intr.fy;
  out.cx = intr.ppx;
  out.cy = intr.ppy;
  for (int i = 0; i < 5; ++i) {
    out.coeffs[static_cast<size_t>(i)] = intr.coeffs[i];
  }
  return out;
}

void RealsenseCameraDevice::open(const CameraOpenConfig & config)
{
  stop();
  config_ = config;

  bool enable_color = config.enable_color;
  if (config.enable_depth && config.align_depth && !enable_color) {
    enable_color = true;
  }

  int depth_w = config.depth_width;
  int depth_h = config.depth_height;
  int depth_fps = config.depth_fps;
  if (config.enable_depth) {
    if (depth_w != config.infra_width || depth_h != config.infra_height ||
      depth_fps != config.infra_fps)
    {
      depth_w = config.infra_width;
      depth_h = config.infra_height;
      depth_fps = config.infra_fps;
    }
  }

  try {
    rs_ = open_rs_sensors(
      config.serial,
      config.infra_width, config.infra_height, config.infra_fps,
      enable_color, config.color_width, config.color_height, config.color_fps,
      config.enable_depth, depth_w, depth_h, depth_fps,
      config.visual_preset, config.emitter_enabled, config.clip_distance_m);
  } catch (const rs2::error & e) {
    throw std::runtime_error(
      std::string("RealSense start failed (is another process using the camera?): ") +
      e.what());
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("RealSense start failed: ") + e.what());
  }

  const auto & profiles = rs_.profiles;
  if (!profiles.infra1 || !profiles.infra2) {
    throw std::runtime_error("IR stereo streams missing from RealSense profile");
  }

  caps_ = {};
  caps_.stereo = true;
  caps_.color = profiles.has_color;
  caps_.depth = profiles.has_depth;
  caps_.aligned_depth = profiles.has_depth && config.align_depth && profiles.has_color;

  const auto left_intr = profiles.infra1.get_intrinsics();
  const auto right_intr = profiles.infra2.get_intrinsics();
  const auto right_to_left = profiles.infra2.get_extrinsics_to(profiles.infra1);

  stereo_cal_.left = intrinsics_from_rs(left_intr);
  stereo_cal_.right = intrinsics_from_rs(right_intr);
  for (int i = 0; i < 9; ++i) {
    stereo_cal_.right_rotation_colmajor[static_cast<size_t>(i)] =
      right_to_left.rotation[i];
  }
  for (int i = 0; i < 3; ++i) {
    stereo_cal_.right_translation[static_cast<size_t>(i)] =
      right_to_left.translation[i];
  }

  // Frame ids filled by the ROS node via static_camera_tfs / CameraInfo setters
  // below use placeholder optical frames; node overwrites CameraInfo.frame_id.
  color_optical_frame_ = "color_optical_frame";
  depth_optical_frame_ = "depth_optical_frame";

  if (caps_.color) {
    color_camera_info_ = camera_info_from_intrinsics(
      profiles.color.get_intrinsics(), color_optical_frame_);
  }
  if (caps_.depth) {
    if (caps_.aligned_depth) {
      depth_camera_info_ = camera_info_from_intrinsics(
        profiles.color.get_intrinsics(), depth_optical_frame_);
      align_to_color_ = std::make_unique<rs2::align>(RS2_STREAM_COLOR);
    } else {
      depth_camera_info_ = camera_info_from_intrinsics(
        profiles.depth.get_intrinsics(), depth_optical_frame_);
    }
  }

  stereo_frame_id_ = 0;
  has_stereo_ = false;
  has_color_ = false;
}

StreamCaps RealsenseCameraDevice::caps() const
{
  return caps_;
}

StereoCalibration RealsenseCameraDevice::stereo_calibration() const
{
  return stereo_cal_;
}

const sensor_msgs::msg::CameraInfo & RealsenseCameraDevice::color_camera_info() const
{
  return color_camera_info_;
}

const sensor_msgs::msg::CameraInfo & RealsenseCameraDevice::depth_camera_info() const
{
  return depth_camera_info_;
}

std::vector<geometry_msgs::msg::TransformStamped>
RealsenseCameraDevice::static_camera_tfs(const std::string & camera_name) const
{
  return build_static_camera_tfs(camera_name, rs_.profiles);
}

int64_t RealsenseCameraDevice::stereo_timestamp_ns(
  const rs2::video_frame & left, int frame_id) const
{
  if (left.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP)) {
    return left.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP) * 1000LL;
  }
  if (left.supports_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP)) {
    return left.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP) * 1000LL;
  }
  return static_cast<int64_t>(frame_id) *
    (1000000000LL / std::max(1, config_.infra_fps));
}

bool RealsenseCameraDevice::wait_stereo(StereoFrame & out, int timeout_ms)
{
  rs2::frameset frames;
  if (!rs_.ir_sync.try_wait_for_frames(&frames, timeout_ms)) {
    return false;
  }
  if (!frames) {
    return false;
  }

  auto left = frames.get_infrared_frame(1);
  auto right = frames.get_infrared_frame(2);
  if (!left || !right) {
    return false;
  }

  auto left_v = left.as<rs2::video_frame>();
  auto right_v = right.as<rs2::video_frame>();
  ++stereo_frame_id_;
  const int64_t ts = stereo_timestamp_ns(left_v, stereo_frame_id_);

  {
    std::lock_guard<std::mutex> lock(mu_);
    latest_stereo_ = frames;
    left_frame_ = left;
    right_frame_ = right;
    has_stereo_ = true;

    out.left_pixels = static_cast<const uint8_t *>(
      left_frame_.as<rs2::video_frame>().get_data());
    out.right_pixels = static_cast<const uint8_t *>(
      right_frame_.as<rs2::video_frame>().get_data());
    out.width = left_v.get_width();
    out.height = left_v.get_height();
    out.stride = left_v.get_stride_in_bytes();
    out.timestamp_ns = ts;
    out.arrival_host_ns = rs_frame_time_of_arrival_ns(left_v);
  }
  return true;
}

bool RealsenseCameraDevice::poll_color(ColorFrame & out)
{
  if (!caps_.color) {
    return false;
  }

  rs2::frame color_f;
  if (!rs_.color_queue.poll_for_frame(&color_f)) {
    return false;
  }
  if (!color_f) {
    return false;
  }

  auto cf = color_f.as<rs2::video_frame>();
  {
    std::lock_guard<std::mutex> lock(mu_);
    latest_color_ = color_f;
    has_color_ = true;
  }

  out.pixels = static_cast<const uint8_t *>(cf.get_data());
  out.width = cf.get_width();
  out.height = cf.get_height();
  out.stride = cf.get_stride_in_bytes();
  out.timestamp_ns = 0;  // ROS stamp applied by node
  return true;
}

bool RealsenseCameraDevice::poll_depth(DepthFrame & out)
{
  if (!caps_.depth) {
    return false;
  }

  rs2::frameset stereo_fs;
  rs2::frame color_f;
  bool have_color = false;
  {
    std::lock_guard<std::mutex> lock(mu_);
    if (!has_stereo_) {
      return false;
    }
    stereo_fs = latest_stereo_;
    if (caps_.aligned_depth && has_color_) {
      color_f = latest_color_;
      have_color = true;
    }
  }

  rs2::frameset depth_fs = stereo_fs;
  if (caps_.aligned_depth && align_to_color_) {
    if (!have_color) {
      return false;
    }
    try {
      depth_color_sync_(stereo_fs);
      depth_color_sync_(color_f);
      rs2::frameset synced = depth_color_sync_.wait_for_frames(50);
      if (!synced) {
        return false;
      }
      depth_fs = align_to_color_->process(synced);
    } catch (const rs2::error &) {
      return false;
    }
  }

  auto depth_frame = depth_fs.get_depth_frame();
  if (!depth_frame) {
    return false;
  }

  auto df = depth_frame.as<rs2::depth_frame>();
  {
    std::lock_guard<std::mutex> lock(mu_);
    latest_depth_frame_ = depth_frame;
  }

  out.pixels = static_cast<const uint8_t *>(df.get_data());
  out.width = df.get_width();
  out.height = df.get_height();
  out.stride = df.get_stride_in_bytes();
  out.timestamp_ns = 0;
  return true;
}

void RealsenseCameraDevice::stop()
{
  {
    std::lock_guard<std::mutex> lock(mu_);
    has_stereo_ = false;
    has_color_ = false;
    latest_stereo_ = rs2::frameset();
    left_frame_ = rs2::frame();
    right_frame_ = rs2::frame();
    latest_color_ = rs2::frame();
    latest_depth_frame_ = rs2::frame();
  }
  align_to_color_.reset();
  rs_.stop();
  caps_ = {};
}

}  // namespace hound_core

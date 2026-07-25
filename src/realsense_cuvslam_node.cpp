#include "hound_core/realsense_cuvslam_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>

#include <rclcpp/qos.hpp>

#include "hound_core/camera_info_utils.hpp"
#include "hound_core/cuvslam_tracker.hpp"
#include "hound_core/frame_conventions.hpp"

namespace hound_core {
namespace {

rclcpp::QoS sensor_qos(size_t depth = 5)
{
  return rclcpp::SensorDataQoS().keep_last(depth);
}

}  // namespace

RealsenseCuvslamNode::RealsenseCuvslamNode(const rclcpp::NodeOptions & options)
: Node("realsense_cuvslam_node", options)
{
  declare_params();

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

  rclcpp::QoS odom_qos(10);
  odom_qos.reliable();
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, odom_qos);

  color_optical_frame_ = camera_name_ + "_color_optical_frame";
  depth_optical_frame_ = camera_name_ + "_depth_optical_frame";

  if (enable_color_) {
    const std::string base = "/" + camera_name_ + "/color";
    color_pub_ = create_publisher<sensor_msgs::msg::Image>(base + "/image_raw", sensor_qos());
    color_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      base + "/camera_info", sensor_qos());
  }
  if (enable_depth_) {
    if (align_depth_ && !enable_color_) {
      RCLCPP_WARN(
        get_logger(),
        "align_depth=true requires color; enabling color streams");
      enable_color_ = true;
      const std::string cbase = "/" + camera_name_ + "/color";
      color_pub_ = create_publisher<sensor_msgs::msg::Image>(
        cbase + "/image_raw", sensor_qos());
      color_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        cbase + "/camera_info", sensor_qos());
    }
    const std::string base = align_depth_
      ? ("/" + camera_name_ + "/aligned_depth_to_color")
      : ("/" + camera_name_ + "/depth");
    const std::string image_name = align_depth_ ? "/image_raw" : "/image_rect_raw";
    if (align_depth_) {
      depth_optical_frame_ = color_optical_frame_;
    }
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(base + image_name, sensor_qos());
    depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      base + "/camera_info", sensor_qos());
  }

  start_pipeline();
  publish_static_tfs();

  running_ = true;
  capture_thread_ = std::thread([this] { capture_loop(); });
  odom_thread_ = std::thread([this] { odom_worker(); });
  if (enable_color_) {
    color_thread_ = std::thread([this] { color_worker(); });
  }
  if (enable_depth_) {
    depth_thread_ = std::thread([this] { depth_worker(); });
  }

  RCLCPP_INFO(
    get_logger(),
    "realsense_cuvslam: serial=%s IR %dx%d@%d color=%s@%dHz(pub %.1f) "
    "depth=%s%s (decoupled sensors) odom=%s",
    serial_number_.c_str(), infra_width_, infra_height_, infra_fps_,
    enable_color_ ? "on" : "off", color_fps_, color_publish_fps_,
    enable_depth_ ? "on" : "off",
    enable_depth_ ? (align_depth_ ? "(aligned_to_color)" : "(native)") : "",
    odom_topic_.c_str());
}

RealsenseCuvslamNode::~RealsenseCuvslamNode()
{
  running_ = false;
  slot_.cv_pose.notify_all();
  slot_.cv_color.notify_all();
  slot_.cv_depth.notify_all();
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  if (odom_thread_.joinable()) {
    odom_thread_.join();
  }
  if (color_thread_.joinable()) {
    color_thread_.join();
  }
  if (depth_thread_.joinable()) {
    depth_thread_.join();
  }
  rs_.stop();
}

void RealsenseCuvslamNode::declare_params()
{
  serial_number_ = declare_parameter<std::string>("serial_number", "030422250301");
  camera_name_ = declare_parameter<std::string>("camera_name", "camera");
  infra_width_ = declare_parameter<int>("infra_width", 640);
  infra_height_ = declare_parameter<int>("infra_height", 360);
  infra_fps_ = declare_parameter<int>("infra_fps", 60);
  enable_color_ = declare_parameter<bool>("enable_color", true);
  color_width_ = declare_parameter<int>("color_width", 640);
  color_height_ = declare_parameter<int>("color_height", 360);
  color_fps_ = declare_parameter<int>("color_fps", 30);
  color_publish_fps_ = declare_parameter<double>("color_publish_fps", 15.0);
  enable_depth_ = declare_parameter<bool>("enable_depth", false);
  align_depth_ = declare_parameter<bool>("align_depth", true);
  depth_width_ = declare_parameter<int>("depth_width", 640);
  depth_height_ = declare_parameter<int>("depth_height", 360);
  depth_fps_ = declare_parameter<int>("depth_fps", 30);
  depth_publish_fps_ = declare_parameter<double>("depth_publish_fps", 15.0);
  emitter_enabled_ = declare_parameter<int>("emitter_enabled", 0);
  visual_preset_ = declare_parameter<int>("visual_preset", 3);
  clip_distance_ = declare_parameter<double>("clip_distance", 0.0);
  odom_topic_ = declare_parameter<std::string>(
    "odom_topic", "/visual_slam/tracking/odometry");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "camera_link");
  async_sba_ = declare_parameter<bool>("async_sba", true);
  slam_sync_mode_ = declare_parameter<bool>("slam_sync_mode", false);
  warmup_frames_ = declare_parameter<int>("warmup_frames", 60);

  if (enable_depth_ && !enable_color_ && align_depth_) {
    RCLCPP_WARN(
      get_logger(),
      "enable_depth+align_depth requires color; enabling color streams");
    enable_color_ = true;
  }
  if (enable_depth_) {
    if (depth_width_ != infra_width_ || depth_height_ != infra_height_ ||
      depth_fps_ != infra_fps_)
    {
      RCLCPP_WARN(
        get_logger(),
        "depth %dx%d@%d forced to match infra %dx%d@%d (shared depth module)",
        depth_width_, depth_height_, depth_fps_,
        infra_width_, infra_height_, infra_fps_);
      depth_width_ = infra_width_;
      depth_height_ = infra_height_;
      depth_fps_ = infra_fps_;
    }
  }
}

void RealsenseCuvslamNode::start_pipeline()
{
  try {
    rs_ = open_rs_sensors(
      serial_number_,
      infra_width_, infra_height_, infra_fps_,
      enable_color_, color_width_, color_height_, color_fps_,
      enable_depth_, depth_width_, depth_height_, depth_fps_,
      visual_preset_, emitter_enabled_, static_cast<float>(clip_distance_));
  } catch (const rs2::error & e) {
    throw std::runtime_error(
      std::string("RealSense start failed (is another process using the camera?): ") +
      e.what());
  } catch (const std::exception & e) {
    throw std::runtime_error(
      std::string("RealSense start failed: ") + e.what());
  }

  const auto & profiles = rs_.profiles;
  if (!profiles.infra1 || !profiles.infra2) {
    throw std::runtime_error("IR stereo streams missing from RealSense profile");
  }

  const auto left_intr = profiles.infra1.get_intrinsics();
  const auto right_intr = profiles.infra2.get_intrinsics();
  const auto right_to_left = profiles.infra2.get_extrinsics_to(profiles.infra1);
  auto rig = build_ir_stereo_rig(left_intr, right_intr, right_to_left);

  if (enable_color_) {
    color_camera_info_ = camera_info_from_intrinsics(
      profiles.color.get_intrinsics(), color_optical_frame_);
  }
  if (enable_depth_) {
    if (align_depth_) {
      depth_camera_info_ = camera_info_from_intrinsics(
        profiles.color.get_intrinsics(), depth_optical_frame_);
      align_to_color_ = std::make_unique<rs2::align>(RS2_STREAM_COLOR);
    } else {
      depth_camera_info_ = camera_info_from_intrinsics(
        profiles.depth.get_intrinsics(), depth_optical_frame_);
    }
  }

  cuvslam::WarmUpGPU();

  cuvslam::Odometry::Config odom_cfg = cuvslam::Odometry::GetDefaultConfig();
  odom_cfg.odometry_mode = cuvslam::Odometry::OdometryMode::Multicamera;
  odom_cfg.async_sba = async_sba_;
  odom_cfg.rectified_stereo_camera = true;
  odom_cfg.enable_observations_export = true;
  odom_cfg.enable_landmarks_export = true;
  odom_cfg.enable_final_landmarks_export = false;
  odometry_ = std::make_unique<cuvslam::Odometry>(rig, odom_cfg);

  cuvslam::Slam::Config slam_cfg = cuvslam::Slam::GetDefaultConfig();
  slam_cfg.sync_mode = slam_sync_mode_;
  slam_cfg.use_gpu = true;
  slam_ = std::make_unique<cuvslam::Slam>(
    rig, odometry_->GetPrimaryCameras(), slam_cfg);

  RCLCPP_INFO(
    get_logger(),
    "cuVSLAM ready (async_sba=%s slam_sync=%s; IR queue decoupled from color)",
    async_sba_ ? "true" : "false",
    slam_sync_mode_ ? "true" : "false");
}

void RealsenseCuvslamNode::publish_static_tfs()
{
  auto tfs = build_static_camera_tfs(camera_name_, rs_.profiles);
  const auto now = get_clock()->now();
  for (auto & tf : tfs) {
    tf.header.stamp = now;
  }
  static_tf_broadcaster_->sendTransform(tfs);
  RCLCPP_INFO(get_logger(), "published %zu static camera TFs", tfs.size());
}

void RealsenseCuvslamNode::capture_loop()
{
  using clock = std::chrono::steady_clock;
  int frame_id = 0;
  int64_t prev_ts = -1;
  int64_t last_track_ts = -1;
  constexpr int64_t kJitterWarnNs = 75LL * 1000000LL;

  double sum_vo_ms = 0.0;
  double sum_slam_ms = 0.0;
  double sum_iter_ms = 0.0;
  double max_vo_ms = 0.0;
  double max_slam_ms = 0.0;
  double max_iter_ms = 0.0;
  int n_track = 0;
  int n_skip_ts = 0;
  int n_ir_frames = 0;
  int color_raw_count = 0;
  // Publish every Nth sensor frame: e.g. 30 Hz raw / 15 Hz pub → stride 2.
  const int color_stride = (enable_color_ && color_publish_fps_ > 0.0)
    ? std::max(
      1,
      static_cast<int>(std::lround(
        static_cast<double>(color_fps_) / color_publish_fps_)))
    : 1;
  auto stats_t0 = clock::now();

  if (enable_color_) {
    RCLCPP_INFO(
      get_logger(),
      "color publish: every %d sensor frame(s) (%d Hz raw → ~%.1f Hz)",
      color_stride, color_fps_,
      static_cast<double>(color_fps_) / static_cast<double>(color_stride));
  }

  while (running_ && rclcpp::ok()) {
    const auto iter_t0 = clock::now();

    // Drain color queue; keep every Nth frame for publish (does not gate Track).
    if (enable_color_) {
      rs2::frame color_f;
      while (rs_.color_queue.poll_for_frame(&color_f)) {
        ++color_raw_count;
        if ((color_raw_count % color_stride) != 0) {
          continue;
        }
        const auto stamp = this->get_clock()->now();
        {
          std::lock_guard<std::mutex> lock(slot_.mu);
          slot_.color_frame = color_f;
          slot_.color_stamp = stamp;
          slot_.has_color = true;
          ++slot_.color_seq;
        }
        slot_.cv_color.notify_one();
      }
    }

    rs2::frameset frames;
    if (!rs_.ir_sync.try_wait_for_frames(&frames, 1000)) {
      continue;
    }
    if (!frames) {
      continue;
    }

    auto left = frames.get_infrared_frame(1);
    auto right = frames.get_infrared_frame(2);
    if (!left || !right) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IR syncer frameset missing left/right (size=%zu)", frames.size());
      continue;
    }

    ++frame_id;
    ++n_ir_frames;

    int64_t timestamp_ns = 0;
    if (left.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP)) {
      timestamp_ns =
        left.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP) * 1000LL;
    } else if (left.supports_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP)) {
      timestamp_ns =
        left.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP) * 1000LL;
    } else {
      timestamp_ns = static_cast<int64_t>(frame_id) *
        (1000000000LL / std::max(1, infra_fps_));
    }

    if (prev_ts >= 0 && (timestamp_ns - prev_ts) > kJitterWarnNs) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IR timestamp gap %.1f ms", (timestamp_ns - prev_ts) * 1e-6);
    }
    prev_ts = timestamp_ns;

    if (frame_id <= warmup_frames_) {
      continue;
    }

    if (timestamp_ns <= last_track_ts) {
      ++n_skip_ts;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "skipping non-increasing IR timestamp (%ld <= %ld)",
        static_cast<long>(timestamp_ns), static_cast<long>(last_track_ts));
      continue;
    }

    cuvslam::Odometry::ImageSet images(2);
    fill_mono_image(
      images[0], left.get_data(), left.get_width(), left.get_height(),
      timestamp_ns, 0);
    fill_mono_image(
      images[1], right.get_data(), right.get_width(), right.get_height(),
      timestamp_ns, 1);

    cuvslam::PoseEstimate estimate;
    const auto vo_t0 = clock::now();
    try {
      estimate = odometry_->Track(images);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Odometry::Track failed: %s", e.what());
      continue;
    }
    const double vo_ms =
      std::chrono::duration<double, std::milli>(clock::now() - vo_t0).count();
    last_track_ts = timestamp_ns;

    if (!estimate.world_from_rig.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "pose tracking not valid");
      continue;
    }

    cuvslam::Pose pose = estimate.world_from_rig->pose;
    double slam_ms = 0.0;
    try {
      cuvslam::Odometry::State state;
      odometry_->GetState(state);
      const auto slam_t0 = clock::now();
      pose = slam_->Track(state);
      slam_ms =
        std::chrono::duration<double, std::milli>(clock::now() - slam_t0).count();
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Slam::Track failed, using VO pose: %s", e.what());
    }

    const double iter_ms =
      std::chrono::duration<double, std::milli>(clock::now() - iter_t0).count();
    sum_vo_ms += vo_ms;
    sum_slam_ms += slam_ms;
    sum_iter_ms += iter_ms;
    max_vo_ms = std::max(max_vo_ms, vo_ms);
    max_slam_ms = std::max(max_slam_ms, slam_ms);
    max_iter_ms = std::max(max_iter_ms, iter_ms);
    ++n_track;

    const auto stats_now = clock::now();
    if (stats_now - stats_t0 >= std::chrono::seconds(2)) {
      const double dt =
        std::chrono::duration<double>(stats_now - stats_t0).count();
      const double inv_n = n_track > 0 ? 1.0 / static_cast<double>(n_track) : 0.0;
      RCLCPP_INFO(
        get_logger(),
        "cuVSLAM timing (%.1fs): vo mean/max=%.2f/%.2f ms  "
        "slam mean/max=%.2f/%.2f ms  iter mean/max=%.2f/%.2f ms  "
        "track=%.1f Hz  ir_frames=%.1f Hz  skip_ts=%d/%d",
        dt,
        sum_vo_ms * inv_n, max_vo_ms,
        sum_slam_ms * inv_n, max_slam_ms,
        sum_iter_ms * inv_n, max_iter_ms,
        n_track / dt, n_ir_frames / dt, n_skip_ts, n_ir_frames);
      sum_vo_ms = sum_slam_ms = sum_iter_ms = 0.0;
      max_vo_ms = max_slam_ms = max_iter_ms = 0.0;
      n_track = n_skip_ts = n_ir_frames = 0;
      stats_t0 = stats_now;
    }

    {
      std::lock_guard<std::mutex> lock(slot_.mu);
      slot_.pose = pose;
      slot_.ros_stamp = this->get_clock()->now();
      slot_.has_pose = true;
      ++slot_.pose_seq;
      if (enable_depth_) {
        slot_.stereo_fs = frames;
        slot_.stereo_stamp = slot_.ros_stamp;
        slot_.has_stereo = true;
      }
    }
    slot_.cv_pose.notify_one();
    if (enable_depth_) {
      slot_.cv_depth.notify_one();
    }
  }
}

void RealsenseCuvslamNode::publish_odom_tf(
  const rclcpp::Time & stamp, const cuvslam::Pose & pose)
{
  std::array<double, 3> t{};
  std::array<double, 4> q{};
  optical_pose_to_ros_flu(pose, t, q);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = t[0];
  odom.pose.pose.position.y = t[1];
  odom.pose.pose.position.z = t[2];
  odom.pose.pose.orientation.x = q[0];
  odom.pose.pose.orientation.y = q[1];
  odom.pose.pose.orientation.z = q[2];
  odom.pose.pose.orientation.w = q[3];

  if (prev_stamp_.nanoseconds() > 0) {
    const double dt = (stamp - prev_stamp_).seconds();
    if (dt > 1e-6) {
      odom.twist.twist.linear.x = (t[0] - prev_t_flu_[0]) / dt;
      odom.twist.twist.linear.y = (t[1] - prev_t_flu_[1]) / dt;
      odom.twist.twist.linear.z = (t[2] - prev_t_flu_[2]) / dt;
    }
  }
  prev_t_flu_ = t;
  prev_stamp_ = stamp;
  odom_pub_->publish(odom);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = odom_frame_;
  tf.child_frame_id = base_frame_;
  tf.transform.translation.x = t[0];
  tf.transform.translation.y = t[1];
  tf.transform.translation.z = t[2];
  tf.transform.rotation.x = q[0];
  tf.transform.rotation.y = q[1];
  tf.transform.rotation.z = q[2];
  tf.transform.rotation.w = q[3];
  tf_broadcaster_->sendTransform(tf);
}

void RealsenseCuvslamNode::odom_worker()
{
  uint64_t last_seq = 0;
  while (running_ && rclcpp::ok()) {
    cuvslam::Pose pose;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    {
      std::unique_lock<std::mutex> lock(slot_.mu);
      slot_.cv_pose.wait_for(lock, std::chrono::milliseconds(100), [&] {
        return !running_ || (slot_.has_pose && slot_.pose_seq != last_seq);
      });
      if (!running_) {
        break;
      }
      if (!slot_.has_pose || slot_.pose_seq == last_seq) {
        continue;
      }
      last_seq = slot_.pose_seq;
      pose = slot_.pose;
      stamp = slot_.ros_stamp;
    }
    publish_odom_tf(stamp, pose);
  }
}

void RealsenseCuvslamNode::color_worker()
{
  if (color_publish_fps_ <= 0.0) {
    return;
  }
  uint64_t last_seq = 0;

  while (running_ && rclcpp::ok()) {
    rs2::frame color_f;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    {
      std::unique_lock<std::mutex> lock(slot_.mu);
      slot_.cv_color.wait_for(lock, std::chrono::milliseconds(100), [&] {
        return !running_ || (slot_.has_color && slot_.color_seq != last_seq);
      });
      if (!running_) {
        break;
      }
      if (!slot_.has_color || slot_.color_seq == last_seq) {
        continue;
      }
      last_seq = slot_.color_seq;
      color_f = slot_.color_frame;
      stamp = slot_.color_stamp;
    }

    if (!color_f) {
      continue;
    }
    auto cf = color_f.as<rs2::video_frame>();
    sensor_msgs::msg::Image img;
    img.header.stamp = stamp;
    img.header.frame_id = color_optical_frame_;
    img.height = static_cast<uint32_t>(cf.get_height());
    img.width = static_cast<uint32_t>(cf.get_width());
    img.encoding = "rgb8";
    img.is_bigendian = 0;
    img.step = static_cast<uint32_t>(cf.get_width() * 3);
    const size_t nbytes = static_cast<size_t>(img.step) * img.height;
    img.data.resize(nbytes);
    std::memcpy(img.data.data(), cf.get_data(), nbytes);

    auto info = color_camera_info_;
    info.header = img.header;
    color_pub_->publish(img);
    color_info_pub_->publish(info);
  }
}

void RealsenseCuvslamNode::depth_worker()
{
  using clock = std::chrono::steady_clock;
  if (depth_publish_fps_ <= 0.0) {
    return;
  }
  const auto period = std::chrono::duration_cast<clock::duration>(
    std::chrono::duration<double>(1.0 / depth_publish_fps_));
  auto next = clock::now();

  while (running_ && rclcpp::ok()) {
    {
      std::unique_lock<std::mutex> lock(slot_.mu);
      slot_.cv_depth.wait_until(lock, next, [&] {
        return !running_ || (slot_.has_stereo && clock::now() >= next);
      });
      if (!running_) {
        break;
      }
    }

    auto now = clock::now();
    if (now < next) {
      continue;
    }

    rs2::frameset stereo_fs;
    rs2::frame color_f;
    bool have_color = false;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    {
      std::lock_guard<std::mutex> lock(slot_.mu);
      if (!slot_.has_stereo) {
        continue;
      }
      stereo_fs = slot_.stereo_fs;
      stamp = slot_.stereo_stamp;
      if (align_depth_ && slot_.has_color) {
        color_f = slot_.color_frame;
        have_color = true;
      }
    }

    rs2::frameset depth_fs = stereo_fs;
    if (align_depth_ && align_to_color_) {
      if (!have_color) {
        do {
          next += period;
        } while (next <= clock::now());
        continue;
      }
      try {
        // Syncer matches depth+color by timestamp without gating the IR Track path.
        depth_color_sync_(stereo_fs);
        depth_color_sync_(color_f);
        rs2::frameset synced = depth_color_sync_.wait_for_frames(50);
        if (synced) {
          depth_fs = align_to_color_->process(synced);
        } else {
          do {
            next += period;
          } while (next <= clock::now());
          continue;
        }
      } catch (const rs2::error & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "depth align failed: %s", e.what());
        do {
          next += period;
        } while (next <= clock::now());
        continue;
      }
    }

    auto depth_frame = depth_fs.get_depth_frame();
    if (depth_frame) {
      auto df = depth_frame.as<rs2::depth_frame>();
      sensor_msgs::msg::Image img;
      img.header.stamp = stamp;
      img.header.frame_id = depth_optical_frame_;
      img.height = static_cast<uint32_t>(df.get_height());
      img.width = static_cast<uint32_t>(df.get_width());
      img.encoding = "16UC1";
      img.is_bigendian = 0;
      img.step = static_cast<uint32_t>(df.get_width() * 2);
      const size_t nbytes = static_cast<size_t>(img.step) * img.height;
      img.data.resize(nbytes);
      std::memcpy(img.data.data(), df.get_data(), nbytes);

      auto info = depth_camera_info_;
      info.header = img.header;
      depth_pub_->publish(img);
      depth_info_pub_->publish(info);
    }

    now = clock::now();
    do {
      next += period;
    } while (next <= now);
  }
}

}  // namespace hound_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<hound_core::RealsenseCuvslamNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "[realsense_cuvslam_node] fatal: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

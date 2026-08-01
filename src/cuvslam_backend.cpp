#include "hound_core/cuvslam_backend.hpp"

#include "hound_core/cuvslam_tracker.hpp"

namespace hound_core {

cuvslam::Rig CuvslamBackend::build_rig(const StereoCalibration & cal) const
{
  cuvslam::Rig rig;

  auto make_cam = [](const CameraIntrinsics & intr, const cuvslam::Pose * extr) {
    cuvslam::Camera cam;
    cam.size = {intr.width, intr.height};
    cam.focal = {intr.fx, intr.fy};
    cam.principal = {intr.cx, intr.cy};
    cam.distortion.model = cuvslam::Distortion::Model::Pinhole;
    cam.distortion.parameters.clear();
    if (extr) {
      cam.rig_from_camera = *extr;
    } else {
      cam.rig_from_camera.rotation = {0.f, 0.f, 0.f, 1.f};
      cam.rig_from_camera.translation = {0.f, 0.f, 0.f};
    }
    return cam;
  };

  rs2_extrinsics right_ex{};
  for (int i = 0; i < 9; ++i) {
    right_ex.rotation[i] = cal.right_rotation_colmajor[static_cast<size_t>(i)];
  }
  for (int i = 0; i < 3; ++i) {
    right_ex.translation[i] = cal.right_translation[static_cast<size_t>(i)];
  }
  const cuvslam::Pose right_pose = rs_extrinsics_to_pose(right_ex);

  rig.cameras.push_back(make_cam(cal.left, nullptr));
  rig.cameras.push_back(make_cam(cal.right, &right_pose));
  return rig;
}

void CuvslamBackend::configure(
  const StereoCalibration & calibration, const VslamConfig & config)
{
  auto rig = build_rig(calibration);

  cuvslam::WarmUpGPU();

  cuvslam::Odometry::Config odom_cfg = cuvslam::Odometry::GetDefaultConfig();
  odom_cfg.odometry_mode = cuvslam::Odometry::OdometryMode::Multicamera;
  odom_cfg.async_sba = config.async_sba;
  odom_cfg.rectified_stereo_camera = true;
  odom_cfg.enable_observations_export = true;
  odom_cfg.enable_landmarks_export = true;
  odom_cfg.enable_final_landmarks_export = false;
  odometry_ = std::make_unique<cuvslam::Odometry>(rig, odom_cfg);

  cuvslam::Slam::Config slam_cfg = cuvslam::Slam::GetDefaultConfig();
  slam_cfg.sync_mode = config.slam_sync_mode;
  slam_cfg.use_gpu = true;
  slam_ = std::make_unique<cuvslam::Slam>(
    rig, odometry_->GetPrimaryCameras(), slam_cfg);
}

std::optional<PoseOptical> CuvslamBackend::track(const StereoFrame & stereo)
{
  if (!odometry_ || !slam_) {
    return std::nullopt;
  }
  if (!stereo.left_pixels || !stereo.right_pixels ||
    stereo.width <= 0 || stereo.height <= 0)
  {
    return std::nullopt;
  }

  cuvslam::Odometry::ImageSet images(2);
  fill_mono_image(
    images[0], stereo.left_pixels, stereo.width, stereo.height,
    stereo.timestamp_ns, 0);
  // fill_mono_image sets pitch = width; override if stride differs.
  images[0].pitch = stereo.stride > 0 ? stereo.stride : stereo.width;
  fill_mono_image(
    images[1], stereo.right_pixels, stereo.width, stereo.height,
    stereo.timestamp_ns, 1);
  images[1].pitch = stereo.stride > 0 ? stereo.stride : stereo.width;

  cuvslam::PoseEstimate estimate;
  try {
    estimate = odometry_->Track(images);
  } catch (const std::exception &) {
    return std::nullopt;
  }

  if (!estimate.world_from_rig.has_value()) {
    return std::nullopt;
  }

  cuvslam::Pose pose = estimate.world_from_rig->pose;
  try {
    cuvslam::Odometry::State state;
    odometry_->GetState(state);
    slam_->Track(state);
    pose = slam_->GetPose();
  } catch (const std::exception &) {
    // Fall back to VO pose (same as legacy).
  }

  PoseOptical out;
  out.translation = {
    pose.translation[0], pose.translation[1], pose.translation[2]};
  out.rotation = {
    pose.rotation[0], pose.rotation[1], pose.rotation[2], pose.rotation[3]};
  return out;
}

}  // namespace hound_core

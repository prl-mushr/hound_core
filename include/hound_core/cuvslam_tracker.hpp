#pragma once

#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <librealsense2/rs.hpp>

#include "cuvslam/cuvslam2.h"
#include "hound_core/frame_conventions.hpp"

namespace hound_core {

/** Column-major rs2 rotation + translation → cuvslam Pose (rig_from_camera). */
inline cuvslam::Pose rs_extrinsics_to_pose(const rs2_extrinsics & ex)
{
  // Librealsense rotation is column-major; convert to row-major for quat helper.
  std::array<double, 9> R = {
    ex.rotation[0], ex.rotation[3], ex.rotation[6],
    ex.rotation[1], ex.rotation[4], ex.rotation[7],
    ex.rotation[2], ex.rotation[5], ex.rotation[8]};
  std::array<double, 4> q{};
  rotation_matrix_to_quat_xyzw(R, q);

  cuvslam::Pose pose;
  pose.rotation = {
    static_cast<float>(q[0]), static_cast<float>(q[1]),
    static_cast<float>(q[2]), static_cast<float>(q[3])};
  pose.translation = {ex.translation[0], ex.translation[1], ex.translation[2]};
  return pose;
}

inline cuvslam::Camera make_cuvslam_camera(
  const rs2_intrinsics & intr, const rs2_extrinsics * extrinsics_to_left = nullptr)
{
  cuvslam::Camera cam;
  cam.size = {intr.width, intr.height};
  cam.focal = {intr.fx, intr.fy};
  cam.principal = {intr.ppx, intr.ppy};
  // IR stereo on D455 is effectively rectified; use Pinhole like the Python path.
  cam.distortion.model = cuvslam::Distortion::Model::Pinhole;
  cam.distortion.parameters.clear();
  if (extrinsics_to_left) {
    cam.rig_from_camera = rs_extrinsics_to_pose(*extrinsics_to_left);
  } else {
    cam.rig_from_camera.rotation = {0.f, 0.f, 0.f, 1.f};
    cam.rig_from_camera.translation = {0.f, 0.f, 0.f};
  }
  return cam;
}

inline cuvslam::Rig build_ir_stereo_rig(
  const rs2_intrinsics & left_intr,
  const rs2_intrinsics & right_intr,
  const rs2_extrinsics & right_to_left)
{
  cuvslam::Rig rig;
  rig.cameras.push_back(make_cuvslam_camera(left_intr, nullptr));
  rig.cameras.push_back(make_cuvslam_camera(right_intr, &right_to_left));
  return rig;
}

inline void fill_mono_image(
  cuvslam::Image & image,
  const void * pixels,
  int width,
  int height,
  int64_t timestamp_ns,
  uint32_t camera_index)
{
  image.pixels = pixels;
  image.width = width;
  image.height = height;
  image.pitch = width;  // contiguous Y8
  image.encoding = cuvslam::ImageData::Encoding::MONO;
  image.data_type = cuvslam::ImageData::DataType::UINT8;
  image.is_gpu_mem = false;
  image.timestamp_ns = timestamp_ns;
  image.camera_index = camera_index;
}

}  // namespace hound_core

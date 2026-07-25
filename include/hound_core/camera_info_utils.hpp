#pragma once

#include <string>

#include <librealsense2/rs.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace hound_core {

/**
 * Pack sensor_msgs/CameraInfo from rs2_intrinsics the same way realsense2_camera
 * does (cached once; only stamp changes per frame).
 */
inline sensor_msgs::msg::CameraInfo camera_info_from_intrinsics(
  const rs2_intrinsics & intr,
  const std::string & frame_id,
  double tx = 0.0,
  double ty = 0.0)
{
  sensor_msgs::msg::CameraInfo info;
  info.width = static_cast<uint32_t>(intr.width);
  info.height = static_cast<uint32_t>(intr.height);
  info.header.frame_id = frame_id;

  switch (intr.model) {
    case RS2_DISTORTION_BROWN_CONRADY:
      info.distortion_model = "plumb_bob";
      break;
    case RS2_DISTORTION_INVERSE_BROWN_CONRADY:
      info.distortion_model = "inverse_plumb_bob";
      break;
    case RS2_DISTORTION_FTHETA:
      info.distortion_model = "ftheta";
      break;
    case RS2_DISTORTION_KANNALA_BRANDT4:
      info.distortion_model = "kannala_brandt4";
      break;
    case RS2_DISTORTION_NONE:
    default:
      info.distortion_model = "plumb_bob";
      break;
  }

  info.d.assign(intr.coeffs, intr.coeffs + 5);

  info.k = {
    intr.fx, 0.0, intr.ppx,
    0.0, intr.fy, intr.ppy,
    0.0, 0.0, 1.0};

  info.r = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};

  // P = [fx 0 ppx Tx; 0 fy ppy Ty; 0 0 1 0]. Tx/Ty in meters * fx for stereo.
  info.p = {
    intr.fx, 0.0, intr.ppx, tx,
    0.0, intr.fy, intr.ppy, ty,
    0.0, 0.0, 1.0, 0.0};

  return info;
}

}  // namespace hound_core

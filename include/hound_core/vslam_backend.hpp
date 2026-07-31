#pragma once

#include <array>
#include <optional>

#include "hound_core/camera_device.hpp"

namespace hound_core {

/** cuVSLAM / optical-frame pose: X right, Y down, Z forward. */
struct PoseOptical {
  std::array<float, 3> translation{0.f, 0.f, 0.f};
  std::array<float, 4> rotation{0.f, 0.f, 0.f, 1.f};  // xyzw
};

struct VslamConfig {
  bool async_sba{true};
  bool slam_sync_mode{false};
};

/**
 * Visual odometry (+ optional SLAM) backend. configure() once after camera
 * open; track() consumes StereoFrame views (must stay valid for the call).
 */
class VslamBackend
{
public:
  virtual ~VslamBackend() = default;

  virtual void configure(
    const StereoCalibration & calibration, const VslamConfig & config) = 0;

  /** Returns nullopt when tracking is not yet valid. */
  virtual std::optional<PoseOptical> track(const StereoFrame & stereo) = 0;
};

}  // namespace hound_core

#pragma once

#include <memory>
#include <optional>

#include "cuvslam/cuvslam2.h"
#include "hound_core/vslam_backend.hpp"

namespace hound_core {

/** VslamBackend wrapping cuvslam::Odometry + cuvslam::Slam (VO then SLAM). */
class CuvslamBackend : public VslamBackend
{
public:
  CuvslamBackend() = default;
  ~CuvslamBackend() override = default;

  void configure(
    const StereoCalibration & calibration, const VslamConfig & config) override;

  std::optional<PoseOptical> track(const StereoFrame & stereo) override;

private:
  cuvslam::Rig build_rig(const StereoCalibration & cal) const;

  std::unique_ptr<cuvslam::Odometry> odometry_;
  std::unique_ptr<cuvslam::Slam> slam_;
};

}  // namespace hound_core

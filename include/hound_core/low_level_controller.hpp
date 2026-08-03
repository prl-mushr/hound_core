#pragma once

#include <string>
#include <utility>
#include <vector>

#include "hound_core/fcu_slots.hpp"

namespace hound_core
{

struct LlStatus
{
  bool active{false};
  bool intervention{false};
  /** Generic k/v pairs published on /low_level_diagnostics. */
  std::vector<std::pair<std::string, std::string>> diagnostics;
};

/**
 * Pluggable low-level controller. Controllers self-actuate via their own
 * MavlinkBridge reference inside tick_imu — the runner never sees command types.
 */
class LowLevelController
{
public:
  virtual ~LowLevelController() = default;
  virtual void update_rc(const RcSample & rc) {(void)rc;}
  virtual void update_mode(const FcuStateSample & state) {(void)state;}
  virtual void update_vesc(const VescSample & vesc) {(void)vesc;}
  /**
   * Measured plant feedback for control_state packing.
   * @return false if not yet valid (e.g. no VESC).
   */
  virtual bool plant_feedback(float & steering_rad, float & wheelspeed_mps) const
  {
    (void)steering_rad;
    (void)wheelspeed_mps;
    return false;
  }
  virtual LlStatus tick_imu(const ImuSample & imu) = 0;
};

}  // namespace hound_core

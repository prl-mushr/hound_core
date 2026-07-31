#pragma once

#include "hound_core/fcu_slots.hpp"

namespace hound_core
{

/**
 * Proprioception seam: writes IMU / mag / baro / GPS / RC / state / AP local
 * (and any future board sensors) into FcuBus. First adapter is MAVLink FCU.
 */
class SensorBoard
{
public:
  virtual ~SensorBoard() = default;

  /** Begin producing samples into `bus`. Safe to call once. */
  virtual void start(FcuBus & bus) = 0;

  /** Stop I/O and release hardware. Idempotent. */
  virtual void stop() = 0;
};

}  // namespace hound_core

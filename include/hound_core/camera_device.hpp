#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace hound_core {

struct StreamCaps {
  bool stereo{false};
  bool color{false};
  bool depth{false};
  bool aligned_depth{false};
};

/** Pinhole-ish intrinsics (transport-agnostic). */
struct CameraIntrinsics {
  int width{0};
  int height{0};
  float fx{0.f};
  float fy{0.f};
  float cx{0.f};
  float cy{0.f};
  std::array<float, 5> coeffs{};
};

/**
 * Stereo IR (or equivalent) calibration.
 * right_* describe the right camera pose in the left optical frame
 * (Librealsense-style column-major rotation + translation in meters).
 */
struct StereoCalibration {
  CameraIntrinsics left{};
  CameraIntrinsics right{};
  std::array<float, 9> right_rotation_colmajor{
    1.f, 0.f, 0.f,
    0.f, 1.f, 0.f,
    0.f, 0.f, 1.f};
  std::array<float, 3> right_translation{0.f, 0.f, 0.f};
};

/**
 * Contiguous or strided mono8 stereo pair. Pixel pointers remain valid until
 * the next wait_stereo() (or stop()) on the owning device.
 */
struct StereoFrame {
  const uint8_t * left_pixels{nullptr};
  const uint8_t * right_pixels{nullptr};
  int width{0};
  int height{0};
  int stride{0};  // bytes per row (Y8: typically == width)
  /** Device/sensor timestamp used by cuVSLAM (ns). */
  int64_t timestamp_ns{0};
  /**
   * Host system-clock ns when librealsense received the left IR frame
   * (RS2_FRAME_METADATA_TIME_OF_ARRIVAL). 0 if unavailable.
   */
  int64_t arrival_host_ns{0};
};

/**
 * RGB8 color frame. Pointers valid until the next poll_color()/stop().
 */
struct ColorFrame {
  const uint8_t * pixels{nullptr};
  int width{0};
  int height{0};
  int stride{0};  // bytes per row (rgb8: typically width * 3)
  int64_t timestamp_ns{0};
};

/**
 * 16-bit depth (Z16) as bytes. Pointers valid until the next poll_depth()/stop().
 */
struct DepthFrame {
  const uint8_t * pixels{nullptr};
  int width{0};
  int height{0};
  int stride{0};  // bytes per row (Z16: typically width * 2)
  int64_t timestamp_ns{0};
};

struct CameraOpenConfig {
  std::string serial;
  int infra_width{640};
  int infra_height{360};
  int infra_fps{60};
  bool enable_color{true};
  int color_width{640};
  int color_height{360};
  int color_fps{30};
  bool enable_depth{false};
  bool align_depth{true};
  int depth_width{640};
  int depth_height{360};
  int depth_fps{30};
  int visual_preset{3};
  int emitter_enabled{0};
  float clip_distance_m{0.f};
};

/**
 * Transport-agnostic camera source. Implementations own hardware buffers;
 * returned frame pointers are views into that storage.
 */
class CameraDevice
{
public:
  virtual ~CameraDevice() = default;

  virtual void open(const CameraOpenConfig & config) = 0;
  virtual StreamCaps caps() const = 0;
  virtual StereoCalibration stereo_calibration() const = 0;

  /** Block up to timeout_ms for the next stereo pair. Returns false on timeout. */
  virtual bool wait_stereo(StereoFrame & out, int timeout_ms) = 0;

  /** Non-blocking; returns false if no new color frame. */
  virtual bool poll_color(ColorFrame & out) = 0;

  /**
   * Non-blocking depth from the latest stereo frameset (optionally aligned to
   * color when caps().aligned_depth). Returns false if unavailable.
   */
  virtual bool poll_depth(DepthFrame & out) = 0;

  virtual void stop() = 0;
};

}  // namespace hound_core

#include "hound_core/mavlink_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include <mavlink/v2.0/common/common.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hound_core
{
namespace
{

constexpr double kMilliRsToRadSec = 1.0e-3;
constexpr double kMilliGToMs2 = 9.80665 / 1000.0;
constexpr double kMilliTToTesla = 1000.0;  // match mavros APM convention
constexpr double kMilliBarToPa = 1.0e2;

inline void frd_to_flu(float x, float y, float z, float & xo, float & yo, float & zo)
{
  xo = x;
  yo = -y;
  zo = -z;
}

}  // namespace

MavlinkBridge::MavlinkBridge(
  const Config & config, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
: cfg_(config), logger_(logger), clock_(std::move(clock))
{
  for (const auto id : cfg_.gcs_throttle_msgids) {
    gcs_throttle_ids_.insert(static_cast<uint32_t>(id));
  }
}

MavlinkBridge::~MavlinkBridge()
{
  stop();
}

void MavlinkBridge::start(FcuBus & bus)
{
  if (started_.exchange(true)) {
    return;
  }
  bus_ = &bus;
  open_links();
}

void MavlinkBridge::stop()
{
  if (!started_.exchange(false) && !fcu_ && !gcs_) {
    return;
  }
  if (fcu_) {
    fcu_->close();
    fcu_.reset();
  }
  if (gcs_) {
    gcs_->close();
    gcs_.reset();
  }
  bus_ = nullptr;
}

void MavlinkBridge::open_links()
{
  try {
    fcu_ = mavconn::MAVConnInterface::open_url(
      cfg_.fcu_url, cfg_.system_id, cfg_.component_id);
    fcu_->message_received_cb = [this](const mavlink::mavlink_message_t * msg,
        mavconn::Framing framing) {
        on_fcu_message(msg, framing);
      };
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      logger_, "Failed to open fcu_url=%s: %s", cfg_.fcu_url.c_str(), e.what());
    throw;
  }

  if (!cfg_.gcs_url.empty()) {
    try {
      gcs_ = mavconn::MAVConnInterface::open_url(
        cfg_.gcs_url, cfg_.system_id, cfg_.component_id);
      gcs_->message_received_cb = [this](const mavlink::mavlink_message_t * msg,
          mavconn::Framing framing) {
          on_gcs_message(msg, framing);
        };
      RCLCPP_INFO(logger_, "GCS bridge open: %s", cfg_.gcs_url.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        logger_, "GCS open failed (%s): %s", cfg_.gcs_url.c_str(), e.what());
      gcs_.reset();
    }
  }
}

void MavlinkBridge::send_to_fcu(const mavlink::Message & msg)
{
  if (!fcu_) {
    return;
  }
  try {
    fcu_->send_message_ignore_drop(msg);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000, "FCU send drop: %s", e.what());
  }
}

void MavlinkBridge::maybe_forward_to_gcs(const mavlink::mavlink_message_t & msg)
{
  if (!gcs_) {
    return;
  }
  const uint32_t msgid = msg.msgid;
  if (!gcs_throttle_ids_.empty() && gcs_throttle_ids_.count(msgid) &&
    cfg_.gcs_throttle_hz > 0.0)
  {
    const auto now_tp = std::chrono::steady_clock::now();
    const auto min_dt = std::chrono::duration<double>(1.0 / cfg_.gcs_throttle_hz);
    std::lock_guard<std::mutex> lock(gcs_throttle_mu_);
    auto & last = gcs_last_fwd_[msgid];
    if (last.time_since_epoch().count() != 0 && (now_tp - last) < min_dt) {
      return;
    }
    last = now_tp;
  }
  try {
    gcs_->send_message_ignore_drop(&msg);
  } catch (...) {
  }
}

void MavlinkBridge::on_fcu_message(
  const mavlink::mavlink_message_t * msg, mavconn::Framing framing)
{
  if (framing != mavconn::Framing::ok || msg == nullptr || bus_ == nullptr) {
    return;
  }
  fcu_seen_ = true;
  maybe_forward_to_gcs(*msg);

  switch (msg->msgid) {
    case mavlink::minimal::msg::HEARTBEAT::MSG_ID:
      handle_heartbeat(*msg);
      break;
    case mavlink::common::msg::SCALED_IMU::MSG_ID:
      handle_scaled_imu(*msg);
      break;
    case mavlink::common::msg::TIMESYNC::MSG_ID:
      handle_timesync(*msg);
      break;
    case mavlink::common::msg::ATTITUDE_QUATERNION::MSG_ID:
      handle_attitude_quat(*msg);
      break;
    case mavlink::common::msg::SCALED_PRESSURE::MSG_ID:
      handle_scaled_pressure(*msg);
      break;
    case mavlink::common::msg::GPS_RAW_INT::MSG_ID:
      handle_gps_raw(*msg);
      break;
    case mavlink::common::msg::RC_CHANNELS::MSG_ID:
    case mavlink::common::msg::RC_CHANNELS_RAW::MSG_ID:
      handle_rc_channels(*msg);
      break;
    case mavlink::common::msg::LOCAL_POSITION_NED::MSG_ID:
      handle_local_position(*msg);
      break;
    case mavlink::common::msg::MISSION_COUNT::MSG_ID:
      handle_mission_count(*msg);
      break;
    case mavlink::common::msg::MISSION_ITEM_INT::MSG_ID:
      handle_mission_item(*msg);
      break;
    case mavlink::common::msg::PARAM_VALUE::MSG_ID:
      handle_param_value(*msg);
      break;
    default:
      break;
  }
}

void MavlinkBridge::on_gcs_message(
  const mavlink::mavlink_message_t * msg, mavconn::Framing framing)
{
  if (framing != mavconn::Framing::ok || msg == nullptr || !fcu_) {
    return;
  }
  if (cfg_.gcs_block_stream_requests &&
    msg->msgid == mavlink::common::msg::REQUEST_DATA_STREAM::MSG_ID)
  {
    return;
  }
  try {
    fcu_->send_message_ignore_drop(msg);
  } catch (...) {
  }
}

void MavlinkBridge::handle_heartbeat(const mavlink::mavlink_message_t & msg)
{
  mavlink::minimal::msg::HEARTBEAT hb{};
  mavlink::MsgMap map(&msg);
  hb.deserialize(map);

  FcuStateSample st;
  st.stamp = now();
  st.connected = true;
  st.armed =
    (hb.base_mode &
    static_cast<uint8_t>(mavlink::minimal::MAV_MODE_FLAG::SAFETY_ARMED)) != 0;
  const uint32_t mode = hb.custom_mode;
  // Copter GUIDED=4; Rover/Plane GUIDED=15 — LL still needs RC switch.
  st.guided = st.armed && (mode == 4U || mode == 15U);
  st.system_status = static_cast<uint8_t>(hb.system_status);
  st.mode = std::to_string(mode);
  bus_->state.write(st);
}

void MavlinkBridge::handle_scaled_imu(const mavlink::mavlink_message_t & msg)
{
  mavlink::common::msg::SCALED_IMU raw{};
  mavlink::MsgMap map(&msg);
  raw.deserialize(map);

  ImuSample s;
  s.time_boot_ms = raw.time_boot_ms;
  s.stamp = stamp_from_fcu_boot_ms(raw.time_boot_ms);
  frd_to_flu(
    static_cast<float>(raw.xgyro * kMilliRsToRadSec),
    static_cast<float>(raw.ygyro * kMilliRsToRadSec),
    static_cast<float>(raw.zgyro * kMilliRsToRadSec),
    s.gx, s.gy, s.gz);
  frd_to_flu(
    static_cast<float>(raw.xacc * kMilliGToMs2),
    static_cast<float>(raw.yacc * kMilliGToMs2),
    static_cast<float>(raw.zacc * kMilliGToMs2),
    s.ax, s.ay, s.az);
  {
    std::lock_guard<std::mutex> lock(att_mu_);
    if (att_valid_) {
      s.qw = att_qw_;
      s.qx = att_qx_;
      s.qy = att_qy_;
      s.qz = att_qz_;
      s.has_orientation = true;
    }
  }
  bus_->imu.write(s);

  MagSample m;
  m.stamp = s.stamp;
  float mx, my, mz;
  frd_to_flu(
    static_cast<float>(raw.xmag * kMilliTToTesla),
    static_cast<float>(raw.ymag * kMilliTToTesla),
    static_cast<float>(raw.zmag * kMilliTToTesla),
    mx, my, mz);
  m.mx = mx;
  m.my = my;
  m.mz = mz;
  bus_->mag.write(m);
}

void MavlinkBridge::handle_attitude_quat(const mavlink::mavlink_message_t & msg)
{
  mavlink::common::msg::ATTITUDE_QUATERNION q{};
  mavlink::MsgMap map(&msg);
  q.deserialize(map);
  float qx, qy, qz;
  frd_to_flu(q.q2, q.q3, q.q4, qx, qy, qz);
  std::lock_guard<std::mutex> lock(att_mu_);
  att_qw_ = q.q1;
  att_qx_ = qx;
  att_qy_ = qy;
  att_qz_ = qz;
  att_valid_ = true;
}

void MavlinkBridge::handle_scaled_pressure(const mavlink::mavlink_message_t & msg)
{
  mavlink::common::msg::SCALED_PRESSURE p{};
  mavlink::MsgMap map(&msg);
  p.deserialize(map);
  BaroSample b;
  b.stamp = now();
  b.pressure_pa = static_cast<float>(p.press_abs * kMilliBarToPa);
  b.temperature_c = static_cast<float>(p.temperature) / 100.0f;
  bus_->baro.write(b);
}

void MavlinkBridge::handle_gps_raw(const mavlink::mavlink_message_t & msg)
{
  mavlink::common::msg::GPS_RAW_INT g{};
  mavlink::MsgMap map(&msg);
  g.deserialize(map);
  // Stream may be 200 Hz; only publish a new fix when AP's fix time advances.
  if (g.time_usec != 0 && g.time_usec == last_gps_time_usec_) {
    return;
  }
  last_gps_time_usec_ = g.time_usec;
  GpsSample s;
  s.stamp = now();
  s.time_usec = g.time_usec;
  s.lat_deg = g.lat * 1e-7;
  s.lon_deg = g.lon * 1e-7;
  s.alt_m = g.alt * 1e-3f;
  s.eph_m = g.eph * 1e-2f;
  s.epv_m = g.epv * 1e-2f;
  s.vel_m_s = g.vel * 1e-2f;
  s.cog_deg = g.cog * 1e-2f;
  s.fix_type = g.fix_type;
  s.satellites_visible = g.satellites_visible;
  bus_->gps.write(s);
}

void MavlinkBridge::handle_rc_channels(const mavlink::mavlink_message_t & msg)
{
  RcSample rc;
  rc.stamp = now();
  if (msg.msgid == mavlink::common::msg::RC_CHANNELS::MSG_ID) {
    mavlink::common::msg::RC_CHANNELS m{};
    mavlink::MsgMap map(&msg);
    m.deserialize(map);
    const uint16_t chans[] = {
      m.chan1_raw, m.chan2_raw, m.chan3_raw, m.chan4_raw, m.chan5_raw, m.chan6_raw,
      m.chan7_raw, m.chan8_raw, m.chan9_raw, m.chan10_raw, m.chan11_raw, m.chan12_raw,
      m.chan13_raw, m.chan14_raw, m.chan15_raw, m.chan16_raw, m.chan17_raw, m.chan18_raw};
    // chancount is 0 on some stacks; still copy all 18 raw fields so HAL
    // can index a configured switch (silent empty ~/rc/in otherwise).
    rc.nchan = 18;
    for (uint8_t i = 0; i < 18; ++i) {
      rc.channels[i] = static_cast<float>(chans[i]);
    }
    (void)m.chancount;
  } else {
    mavlink::common::msg::RC_CHANNELS_RAW m{};
    mavlink::MsgMap map(&msg);
    m.deserialize(map);
    const uint16_t chans[] = {
      m.chan1_raw, m.chan2_raw, m.chan3_raw, m.chan4_raw,
      m.chan5_raw, m.chan6_raw, m.chan7_raw, m.chan8_raw};
    rc.nchan = 8;
    for (uint8_t i = 0; i < 8; ++i) {
      rc.channels[i] = static_cast<float>(chans[i]);
    }
  }
  bus_->rc.write(rc);
}

void MavlinkBridge::handle_local_position(const mavlink::mavlink_message_t & msg)
{
  mavlink::common::msg::LOCAL_POSITION_NED p{};
  mavlink::MsgMap map(&msg);
  p.deserialize(map);
  ApLocalPoseSample s;
  s.stamp = now();
  s.x = p.y;
  s.y = p.x;
  s.z = -p.z;
  s.vx = p.vy;
  s.vy = p.vx;
  s.vz = -p.vz;
  {
    std::lock_guard<std::mutex> lock(att_mu_);
    if (att_valid_) {
      s.qw = att_qw_;
      s.qx = att_qx_;
      s.qy = att_qy_;
      s.qz = att_qz_;
    }
  }
  bus_->ap_local.write(s);
}

void MavlinkBridge::handle_mission_count(const mavlink::mavlink_message_t & msg)
{
  mavlink::common::msg::MISSION_COUNT c{};
  mavlink::MsgMap map(&msg);
  c.deserialize(map);
  std::lock_guard<std::mutex> lock(mission_mu_);
  mission_count_ = c.count;
  mission_next_ = 0;
  mission_items_.clear();
  mission_items_.reserve(c.count);
  if (c.count == 0) {
    mission_dirty_ = true;
    return;
  }
  mavlink::common::msg::MISSION_REQUEST_INT req{};
  req.target_system = cfg_.target_system;
  req.target_component = cfg_.target_component;
  req.seq = 0;
  req.mission_type = 0;
  send_to_fcu(req);
}

void MavlinkBridge::handle_mission_item(const mavlink::mavlink_message_t & msg)
{
  mavlink::common::msg::MISSION_ITEM_INT it{};
  mavlink::MsgMap map(&msg);
  it.deserialize(map);

  MissionItem wp;
  wp.seq = it.seq;
  wp.frame = it.frame;
  wp.command = it.command;
  wp.is_current = it.current != 0;
  wp.autocontinue = it.autocontinue != 0;
  wp.param1 = it.param1;
  wp.param2 = it.param2;
  wp.param3 = it.param3;
  wp.param4 = it.param4;
  wp.x_lat = it.x * 1e-7;
  wp.y_long = it.y * 1e-7;
  wp.z_alt = it.z;

  std::lock_guard<std::mutex> lock(mission_mu_);
  if (it.seq >= mission_items_.size()) {
    mission_items_.resize(it.seq + 1);
  }
  mission_items_[it.seq] = wp;
  mission_next_ = static_cast<uint16_t>(it.seq + 1);
  if (mission_next_ < mission_count_) {
    mavlink::common::msg::MISSION_REQUEST_INT req{};
    req.target_system = cfg_.target_system;
    req.target_component = cfg_.target_component;
    req.seq = mission_next_;
    req.mission_type = 0;
    send_to_fcu(req);
  } else {
    mission_dirty_ = true;
  }
}

void MavlinkBridge::handle_param_value(const mavlink::mavlink_message_t & /*msg*/)
{
  // Ack path reserved for future param verify
}

void MavlinkBridge::handle_timesync(const mavlink::mavlink_message_t & msg)
{
  mavlink::common::msg::TIMESYNC t{};
  mavlink::MsgMap map(&msg);
  t.deserialize(map);

  if (t.tc1 == 0) {
    mavlink::common::msg::TIMESYNC reply{};
    reply.tc1 = static_cast<int64_t>(now().nanoseconds());
    reply.ts1 = t.ts1;
    send_to_fcu(reply);
    return;
  }

  const int64_t now_ns = static_cast<int64_t>(now().nanoseconds());
  const int64_t rtt_ns = now_ns - t.ts1;
  if (rtt_ns < 0 || rtt_ns > 50000000) {
    return;
  }
  // host_ns - fcu_boot_ns; ArduPilot tc1 is AP_HAL::micros64()*1000 (boot ns).
  const int64_t offset_ns = (t.ts1 + now_ns) / 2 - t.tc1;
  if (!tsync_valid_.load(std::memory_order_relaxed)) {
    tsync_offset_ns_.store(offset_ns, std::memory_order_relaxed);
    tsync_valid_.store(true, std::memory_order_release);
    RCLCPP_INFO(
      logger_, "FCU TIMESYNC locked: offset=%.3f ms RTT=%.3f ms",
      static_cast<double>(offset_ns) * 1.0e-6,
      static_cast<double>(rtt_ns) * 1.0e-6);
    return;
  }
  const int64_t prev = tsync_offset_ns_.load(std::memory_order_relaxed);
  tsync_offset_ns_.store(prev + (offset_ns - prev) / 8, std::memory_order_relaxed);
}

void MavlinkBridge::send_timesync()
{
  mavlink::common::msg::TIMESYNC t{};
  t.tc1 = 0;
  t.ts1 = static_cast<int64_t>(now().nanoseconds());
  send_to_fcu(t);
}

rclcpp::Time MavlinkBridge::stamp_from_fcu_boot_ms(uint32_t time_boot_ms) const
{
  if (time_boot_ms == 0U || !tsync_valid_.load(std::memory_order_acquire)) {
    return now();
  }
  const int64_t ns =
    static_cast<int64_t>(time_boot_ms) * 1000000LL +
    tsync_offset_ns_.load(std::memory_order_relaxed);
  if (ns <= 0) {
    return now();
  }
  return rclcpp::Time(static_cast<uint64_t>(ns), RCL_ROS_TIME);
}

void MavlinkBridge::boot_configure()
{
  if (boot_done_.exchange(true)) {
    return;
  }
  RCLCPP_INFO(logger_, "FCU seen — pushing stream rates / params / mission pull");
  push_fcu_params();
  request_data_streams();
  request_mission_list();
  send_timesync();
}

void MavlinkBridge::push_fcu_params()
{
  for (const auto & kv : cfg_.fcu_params) {
    mavlink::common::msg::PARAM_SET ps{};
    ps.target_system = cfg_.target_system;
    ps.target_component = cfg_.target_component;
    mavlink::set_string(ps.param_id, kv.first);
    ps.param_type = static_cast<uint8_t>(mavlink::common::MAV_PARAM_TYPE::REAL32);
    ps.param_value = static_cast<float>(kv.second);
    send_to_fcu(ps);
  }
}

void MavlinkBridge::request_data_streams()
{
  auto set_interval = [this](uint32_t msgid, float hz) {
      mavlink::common::msg::COMMAND_LONG cmd{};
      cmd.target_system = cfg_.target_system;
      cmd.target_component = cfg_.target_component;
      cmd.command = static_cast<uint16_t>(mavlink::common::MAV_CMD::SET_MESSAGE_INTERVAL);
      cmd.param1 = static_cast<float>(msgid);
      cmd.param2 = (hz > 0.0f) ? (1.0e6f / hz) : -1.0f;
      send_to_fcu(cmd);
    };
  set_interval(mavlink::common::msg::SCALED_IMU::MSG_ID, 200.0f);
  set_interval(mavlink::common::msg::RAW_IMU::MSG_ID, 0.0f);
  set_interval(mavlink::common::msg::ATTITUDE_QUATERNION::MSG_ID, 200.0f);
  set_interval(mavlink::common::msg::SCALED_PRESSURE::MSG_ID, 50.0f);
  set_interval(mavlink::common::msg::GPS_RAW_INT::MSG_ID, 200.0f);
  set_interval(mavlink::common::msg::RC_CHANNELS::MSG_ID, 50.0f);
  set_interval(mavlink::common::msg::LOCAL_POSITION_NED::MSG_ID, 50.0f);
}

void MavlinkBridge::request_mission_list()
{
  mavlink::common::msg::MISSION_REQUEST_LIST req{};
  req.target_system = cfg_.target_system;
  req.target_component = cfg_.target_component;
  req.mission_type = 0;
  send_to_fcu(req);
}

void MavlinkBridge::send_heartbeat()
{
  mavlink::minimal::msg::HEARTBEAT hb{};
  hb.type = static_cast<uint8_t>(mavlink::minimal::MAV_TYPE::ONBOARD_CONTROLLER);
  hb.autopilot = static_cast<uint8_t>(mavlink::minimal::MAV_AUTOPILOT::INVALID);
  hb.base_mode = 0;
  hb.custom_mode = 0;
  hb.system_status = static_cast<uint8_t>(mavlink::minimal::MAV_STATE::ACTIVE);
  send_to_fcu(hb);
  send_timesync();
}

void MavlinkBridge::send_vision(const ExtNavSample & nav)
{
  if (!cfg_.send_vision_to_fcu) {
    return;
  }
  mavlink::common::msg::VISION_POSITION_ESTIMATE vp{};
  vp.usec = static_cast<uint64_t>(nav.stamp.nanoseconds() / 1000ULL);
  vp.x = nav.pos_enu[1];
  vp.y = nav.pos_enu[0];
  vp.z = -nav.pos_enu[2];
  const float w = nav.quat_wxyz[0], x = nav.quat_wxyz[1], y = nav.quat_wxyz[2],
    z = nav.quat_wxyz[3];
  const float siny_cosp = 2.0f * (w * z + x * y);
  const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
  const float yaw_enu = std::atan2(siny_cosp, cosy_cosp);
  vp.yaw = static_cast<float>(M_PI / 2.0) - yaw_enu;
  vp.roll = 0.0f;
  vp.pitch = 0.0f;
  send_to_fcu(vp);
}

void MavlinkBridge::send_manual_control(const ManualControlCmd & cmd)
{
  mavlink::common::msg::MANUAL_CONTROL mc{};
  mc.target = cfg_.target_system;
  mc.x = static_cast<int16_t>(cmd.x);
  mc.y = static_cast<int16_t>(cmd.y);
  mc.z = static_cast<int16_t>(cmd.z);
  mc.r = static_cast<int16_t>(cmd.r);
  mc.buttons = cmd.buttons;
  send_to_fcu(mc);
}

void MavlinkBridge::send_rc_override(const RcOverrideCmd & cmd)
{
  mavlink::common::msg::RC_CHANNELS_OVERRIDE ov{};
  ov.target_system = cfg_.target_system;
  ov.target_component = cfg_.target_component;
  ov.chan1_raw = cmd.channels[0];
  ov.chan2_raw = cmd.channels[1];
  ov.chan3_raw = cmd.channels[2];
  ov.chan4_raw = cmd.channels[3];
  ov.chan5_raw = cmd.channels[4];
  ov.chan6_raw = cmd.channels[5];
  ov.chan7_raw = cmd.channels[6];
  ov.chan8_raw = cmd.channels[7];
  ov.chan9_raw = cmd.channels[8];
  ov.chan10_raw = cmd.channels[9];
  ov.chan11_raw = cmd.channels[10];
  ov.chan12_raw = cmd.channels[11];
  ov.chan13_raw = cmd.channels[12];
  ov.chan14_raw = cmd.channels[13];
  ov.chan15_raw = cmd.channels[14];
  ov.chan16_raw = cmd.channels[15];
  ov.chan17_raw = cmd.channels[16];
  ov.chan18_raw = cmd.channels[17];
  send_to_fcu(ov);
}

void MavlinkBridge::request_mode(uint32_t custom_mode)
{
  const auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(mode_mu_);
    if (last_mode_request_.time_since_epoch().count() != 0 &&
      (now - last_mode_request_) < std::chrono::seconds(1))
    {
      return;
    }
    last_mode_request_ = now;
  }

  mavlink::common::msg::SET_MODE sm{};
  sm.target_system = cfg_.target_system;
  sm.base_mode = static_cast<uint8_t>(mavlink::minimal::MAV_MODE_FLAG::CUSTOM_MODE_ENABLED);
  sm.custom_mode = custom_mode;
  send_to_fcu(sm);
}

void MavlinkBridge::send_gps_rtcm(const uint8_t * data, size_t len)
{
  if (data == nullptr || len == 0 || !fcu_) {
    return;
  }
  mavlink::common::msg::GPS_RTCM_DATA msg{};
  const size_t max_frag = msg.data.size();

  auto send_slice = [&](uint8_t flags, const uint8_t * p, size_t n) {
      msg.flags = flags;
      msg.len = static_cast<uint8_t>(n);
      std::copy(p, p + n, msg.data.begin());
      if (n < max_frag) {
        std::fill(msg.data.begin() + n, msg.data.end(), 0);
      }
      send_to_fcu(msg);
    };

  if (len <= max_frag) {
    send_slice(static_cast<uint8_t>((rtcm_seq_++ & 0x1Fu) << 3), data, len);
    return;
  }
  if (len <= 4 * max_frag) {
    const uint8_t seq_u5 = static_cast<uint8_t>((rtcm_seq_++ & 0x1Fu) << 3);
    size_t off = 0;
    for (uint8_t frag = 0; frag < 4 && off < len; ++frag) {
      const size_t n = std::min(len - off, max_frag);
      send_slice(static_cast<uint8_t>(1u | (frag << 1) | seq_u5), data + off, n);
      off += n;
    }
    return;
  }
  // Larger than 4*180: GPS UART is a byte pipe — inject unfragmented slices.
  size_t off = 0;
  while (off < len) {
    const size_t n = std::min(len - off, max_frag);
    send_slice(static_cast<uint8_t>((rtcm_seq_++ & 0x1Fu) << 3), data + off, n);
    off += n;
  }
}

void MavlinkBridge::send_play_tune(const std::string & tune, uint32_t format)
{
  mavlink::common::msg::PLAY_TUNE_V2 pt{};
  pt.target_system = cfg_.target_system;
  pt.target_component = cfg_.target_component;
  pt.format = format;
  pt.tune.fill(0);
  const size_t n = std::min(tune.size(), pt.tune.size() - 1);
  if (n > 0) {
    std::memcpy(pt.tune.data(), tune.data(), n);
  }
  send_to_fcu(pt);
}

bool MavlinkBridge::take_mission(std::vector<MissionItem> & out)
{
  std::lock_guard<std::mutex> lock(mission_mu_);
  if (!mission_dirty_) {
    return false;
  }
  out = mission_items_;
  mission_dirty_ = false;
  return true;
}

}  // namespace hound_core

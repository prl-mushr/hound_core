#include "hound_core/ll_runner.hpp"

#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

namespace hound_core
{

LlRunner::LlRunner(rclcpp::Logger logger)
: logger_(logger)
{
  (void)logger_;
}

LlRunner::~LlRunner()
{
  stop();
}

void LlRunner::start(
  FcuBus & bus, LowLevelController & controller, int ll_cpu, DiagCallback diag_cb)
{
  stop();
  controller_ = &controller;
  diag_cb_ = std::move(diag_cb);
  worker_.start(
    bus, ll_cpu,
    [this](FcuBus & b, std::atomic<bool> & running) {loop(b, running);});
}

void LlRunner::stop()
{
  worker_.stop();
  controller_ = nullptr;
  diag_cb_ = nullptr;
}

void LlRunner::loop(FcuBus & bus, std::atomic<bool> & running)
{
  uint64_t last_seq = 0;
  ImuSample imu;
  while (running.load(std::memory_order_relaxed)) {
    if (!bus.imu.wait_new(last_seq, imu, running)) {
      break;
    }
    if (controller_ == nullptr) {
      continue;
    }

    RcSample rc;
    if (bus.rc.copy_latest(rc) && rc.nchan > 0) {
      controller_->update_rc(rc);
    }
    FcuStateSample st;
    if (bus.state.copy_latest(st)) {
      controller_->update_mode(st);
    }
    VescSample vesc;
    if (bus.vesc.copy_latest(vesc)) {
      controller_->update_vesc(vesc);
    }

    const LlStatus status = controller_->tick_imu(imu);

    // Pack BeamNG 17-vector for planning/control (synced to this IMU tick).
    EkfNavSample nav;
    if (bus.ekf_nav.copy_latest(nav)) {
      ControlStateSample cs;
      cs.stamp = imu.stamp;
      cs.x[0] = nav.pos_enu[0];
      cs.x[1] = nav.pos_enu[1];
      cs.x[2] = nav.pos_enu[2];
      cs.x[3] = nav.rpy[0];
      cs.x[4] = nav.rpy[1];
      cs.x[5] = nav.rpy[2];
      cs.x[6] = nav.vel_body[0];
      cs.x[7] = nav.vel_body[1];
      cs.x[8] = nav.vel_body[2];
      cs.x[9] = imu.ax;
      cs.x[10] = imu.ay;
      cs.x[11] = imu.az;
      cs.x[12] = imu.gx;
      cs.x[13] = imu.gy;
      cs.x[14] = imu.gz;
      float steer_rad = 0.0f;
      float wheelspeed_mps = 0.0f;
      if (controller_->plant_feedback(steer_rad, wheelspeed_mps)) {
        cs.x[15] = steer_rad;
        cs.x[16] = wheelspeed_mps;
      }
      bus.control_state.write(cs);
    }

    if (!status.active && status.diagnostics.empty()) {
      continue;
    }

    if (diag_cb_) {
      diagnostic_msgs::msg::DiagnosticArray dia;
      diagnostic_msgs::msg::DiagnosticStatus st_msg;
      st_msg.name = "LL_control";
      st_msg.level = status.intervention ?
        diagnostic_msgs::msg::DiagnosticStatus::WARN :
        diagnostic_msgs::msg::DiagnosticStatus::OK;
      st_msg.message = status.intervention ? "intervention" : "ok";
      for (const auto & kv : status.diagnostics) {
        diagnostic_msgs::msg::KeyValue entry;
        entry.key = kv.first;
        entry.value = kv.second;
        st_msg.values.push_back(entry);
      }
      dia.status.push_back(st_msg);
      diag_cb_(dia);
    }
  }
}

}  // namespace hound_core

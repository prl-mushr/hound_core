#include "hound_core/ll_runner.hpp"

#include <pthread.h>
#include <sched.h>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

namespace hound_core
{
namespace
{

bool pin_current_thread(int cpu)
{
  if (cpu < 0) {
    return true;
  }
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  return pthread_setaffinity_np(pthread_self(), sizeof(set), &set) == 0;
}

}  // namespace

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
  FcuBus & bus, LlController & ll, SendManualFn send_manual, int ll_cpu,
  DiagCallback diag_cb)
{
  stop();
  bus_ = &bus;
  ll_ = &ll;
  send_manual_ = std::move(send_manual);
  diag_cb_ = std::move(diag_cb);
  ll_cpu_ = ll_cpu;
  running_ = true;
  thread_ = std::thread([this] { worker(); });
}

void LlRunner::stop()
{
  running_ = false;
  if (bus_ != nullptr) {
    bus_->imu.cv.notify_all();
  }
  if (thread_.joinable()) {
    thread_.join();
  }
  bus_ = nullptr;
  ll_ = nullptr;
  send_manual_ = nullptr;
  diag_cb_ = nullptr;
}

void LlRunner::worker()
{
  pin_current_thread(ll_cpu_);
  uint64_t last_seq = 0;
  ImuSample imu;
  while (running_) {
    if (!bus_->imu.wait_new(last_seq, imu, running_)) {
      break;
    }

    // Pull latest RC / mode from bus (legacy applied these in MAVLink handlers).
    RcSample rc;
    if (bus_->rc.copy_latest(rc) && rc.nchan > 0) {
      ll_->update_rc(rc.channels.data(), rc.nchan);
    }
    FcuStateSample st;
    if (bus_->state.copy_latest(st)) {
      ll_->update_mode(st.armed, st.guided);
    }

    float q[4] = {imu.qw, imu.qx, imu.qy, imu.qz};
    const auto cmd = ll_->tick_imu(
      imu.gx, imu.gy, imu.gz, imu.ax, imu.ay, imu.az,
      imu.has_orientation ? q : nullptr);
    if (!cmd.active) {
      continue;
    }
    ManualControlCmd mc;
    mc.x = 1000.0f;
    mc.y = -cmd.steering_norm * 1000.0f;
    mc.z = cmd.throttle_duty * 1000.0f;
    mc.r = 1000.0f;
    if (send_manual_) {
      send_manual_(mc);
    }

    if (diag_cb_) {
      diagnostic_msgs::msg::DiagnosticArray dia;
      diagnostic_msgs::msg::DiagnosticStatus st_msg;
      st_msg.name = "LL_control";
      st_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      st_msg.message = "ok";
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "intervention";
      kv.value = std::to_string(cmd.intervention);
      st_msg.values.push_back(kv);
      dia.status.push_back(st_msg);
      diag_cb_(dia);
    }
  }
}

}  // namespace hound_core

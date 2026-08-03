#include "hound_core/vesc_runner.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

#include <boost/bind/bind.hpp>
#include <rclcpp/logging.hpp>

namespace hound_core
{

VescRunner::VescRunner(rclcpp::Logger logger)
: logger_(std::move(logger))
{
}

VescRunner::~VescRunner()
{
  stop();
}

void VescRunner::start(FcuBus & bus, const Config & config)
{
  stop();
  cfg_ = config;
  bus_ = &bus;
  if (cfg_.telemetry_hz <= 0.0) {
    throw std::runtime_error("vesc telemetry_hz must be > 0");
  }

  vesc_ = std::make_unique<vesc_driver::VescInterface>(
    std::string(),
    boost::bind(&VescRunner::on_packet, this, boost::placeholders::_1),
    boost::bind(&VescRunner::on_error, this, boost::placeholders::_1));

  try {
    vesc_->connect(cfg_.port);
  } catch (const vesc_driver::SerialException & e) {
    vesc_.reset();
    bus_ = nullptr;
    throw std::runtime_error(std::string("VESC connect failed: ") + e.what());
  }

  mode_ = kInitializing;
  fw_major_ = -1;
  fw_minor_ = -1;
  running_.store(true);
  thread_ = std::thread([this]() {loop();});
  RCLCPP_INFO(
    logger_, "VESC in-process telemetry: port=%s hz=%.1f",
    cfg_.port.c_str(), cfg_.telemetry_hz);
}

void VescRunner::stop()
{
  running_.store(false);
  if (thread_.joinable()) {
    thread_.join();
  }
  if (vesc_) {
    if (vesc_->isConnected()) {
      vesc_->disconnect();
    }
    vesc_.reset();
  }
  bus_ = nullptr;
}

void VescRunner::on_error(const std::string & error)
{
  RCLCPP_ERROR(logger_, "VESC: %s", error.c_str());
}

void VescRunner::on_packet(const vesc_driver::VescPacketConstPtr & packet)
{
  if (!packet || bus_ == nullptr) {
    return;
  }
  if (packet->name() == "Values") {
    const auto values =
      boost::dynamic_pointer_cast<vesc_driver::VescPacketValues const>(packet);
    if (!values) {
      return;
    }
    VescSample s;
    s.stamp = clock_.now();
    s.voltage_input = static_cast<float>(values->v_in());
    s.temperature_pcb = static_cast<float>(values->temp_pcb());
    s.current_motor = static_cast<float>(values->current_motor());
    s.current_input = static_cast<float>(values->current_in());
    s.speed = static_cast<float>(values->rpm());
    s.duty_cycle = static_cast<float>(values->duty_now());
    s.charge_drawn = static_cast<float>(values->amp_hours());
    s.charge_regen = static_cast<float>(values->amp_hours_charged());
    s.energy_drawn = static_cast<float>(values->watt_hours());
    s.energy_regen = static_cast<float>(values->watt_hours_charged());
    s.displacement = static_cast<float>(values->tachometer());
    s.distance_traveled = static_cast<float>(values->tachometer_abs());
    s.fault_code = static_cast<int32_t>(values->fault_code());
    bus_->vesc.write(s);
  } else if (packet->name() == "FWVersion") {
    const auto fw =
      boost::dynamic_pointer_cast<vesc_driver::VescPacketFWVersion const>(packet);
    if (!fw) {
      return;
    }
    fw_major_ = fw->fwMajor();
    fw_minor_ = fw->fwMinor();
  }
}

void VescRunner::loop()
{
  const auto period =
    std::chrono::duration<double>(1.0 / cfg_.telemetry_hz);
  while (running_.load(std::memory_order_relaxed)) {
    const auto t0 = std::chrono::steady_clock::now();
    if (!vesc_ || !vesc_->isConnected()) {
      RCLCPP_ERROR_THROTTLE(
        logger_, clock_, 2000, "VESC serial disconnected");
    } else if (mode_ == kInitializing) {
      vesc_->requestFWVersion();
      if (fw_major_ >= 0 && fw_minor_ >= 0) {
        RCLCPP_INFO(
          logger_, "Connected to VESC firmware %d.%d", fw_major_, fw_minor_);
        mode_ = kOperating;
      }
    } else {
      vesc_->requestState();
    }

    const auto elapsed = std::chrono::steady_clock::now() - t0;
    if (elapsed < period) {
      std::this_thread::sleep_for(period - elapsed);
    }
  }
}

}  // namespace hound_core

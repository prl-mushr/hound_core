#pragma once

#include <atomic>
#include <functional>
#include <thread>

#include <pthread.h>
#include <sched.h>

#include "hound_core/fcu_slots.hpp"

namespace hound_core
{

inline bool pin_current_thread(int cpu)
{
  if (cpu < 0) {
    return true;
  }
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  return pthread_setaffinity_np(pthread_self(), sizeof(set), &set) == 0;
}

/**
 * Shared IMU-paced worker thread lifecycle (pin + start/stop/join + wait_new loop).
 * Callers provide a LoopFn that runs on the worker thread; typically it blocks on
 * bus.imu.wait_new using the provided running flag.
 */
class ImuPacedWorker
{
public:
  using LoopFn = std::function<void (FcuBus & bus, std::atomic<bool> & running)>;

  ImuPacedWorker() = default;
  ~ImuPacedWorker() {stop();}

  ImuPacedWorker(const ImuPacedWorker &) = delete;
  ImuPacedWorker & operator=(const ImuPacedWorker &) = delete;

  void start(FcuBus & bus, int cpu, LoopFn loop)
  {
    stop();
    bus_ = &bus;
    cpu_ = cpu;
    loop_ = std::move(loop);
    running_ = true;
    thread_ = std::thread([this] {
        pin_current_thread(cpu_);
        if (loop_ && bus_) {
          loop_(*bus_, running_);
        }
      });
  }

  void stop()
  {
    running_ = false;
    if (bus_ != nullptr) {
      bus_->imu.cv.notify_all();
    }
    if (thread_.joinable()) {
      thread_.join();
    }
    bus_ = nullptr;
    loop_ = nullptr;
  }

  std::atomic<bool> & running_flag() {return running_;}

private:
  FcuBus * bus_{nullptr};
  int cpu_{-1};
  LoopFn loop_;
  std::atomic<bool> running_{false};
  std::thread thread_;
};

}  // namespace hound_core

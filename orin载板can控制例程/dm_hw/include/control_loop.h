#pragma once

#include "hardware_interface/DmHW.h"

#include <chrono>
#include <thread>


namespace damiao
{
using clock = std::chrono::steady_clock;
using duration = std::chrono::duration<double>;

class DmHWLoop
{
public:

  DmHWLoop(std::shared_ptr<DmHW> hardware_interface);

  ~DmHWLoop();

  void update();

private:

  // Settings
  double cycle_time_error_threshold_{};

  // Timing
  std::thread loop_thread_;
  std::atomic_bool loopRunning_;
  double loop_hz_{};
  duration elapsed_time_;
  clock::time_point last_time_;

  // Abstract Hardware Interface for your robot
  std::shared_ptr<DmHW> hardware_interface_;
};
}  // namespace rm_hw

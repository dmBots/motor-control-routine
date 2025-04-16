#include "control_loop.h"

namespace damiao
{
DmHWLoop::DmHWLoop(std::shared_ptr<DmHW> hardware_interface)
  :  hardware_interface_(std::move(hardware_interface)),loopRunning_(true)
{
  loop_hz_=500;
  cycle_time_error_threshold_=0.002;
  int thread_priority=95;
  std::vector<std::string> bus_names;
  bus_names.push_back("can0");//orin载板只有一个can0
  
  // Initialise the hardware interface:
  hardware_interface_->setCanBusThreadPriority(thread_priority);
  hardware_interface_->init(bus_names);

  // Get current time for use with first update
  last_time_ = clock::now();

  // Setup loop thread
  // std::thread loop_thread_
  loop_thread_ = std::thread([&]() {
    while (loopRunning_)
    {
        update();
    }
  });
  sched_param sched{ .sched_priority = thread_priority };
  if (pthread_setschedparam(loop_thread_.native_handle(), SCHED_FIFO, &sched) != 0)
  {
    std::cerr <<"Failed to set threads priority (one possible reason could be that the user and the group permissions are not set properly." <<std::endl;
   
  } 
}

void DmHWLoop::update()
{
  const auto current_time = clock::now();
  // 计算期望周期（
  const duration desired_duration(1.0 / loop_hz_);
  // 计算时间间隔
  duration time_span = current_time - last_time_;
  elapsed_time_ = time_span;
  last_time_ = current_time;

  const double cycle_time_error = (elapsed_time_ - desired_duration).count();
  if (cycle_time_error > cycle_time_error_threshold_)
  {
      std::cerr << "WARN: Cycle time exceeded error threshold by: " 
                << cycle_time_error - cycle_time_error_threshold_
                << "s, cycle time: " << elapsed_time_.count() 
                << "s, threshold: " << cycle_time_error_threshold_ << "s\n";
  }
  // Input
  hardware_interface_->read(std::chrono::system_clock::now(), elapsed_time_);

  // 控制（更新控制器）
  //控制器更新

  // 输出
  hardware_interface_->write(std::chrono::system_clock::now(), elapsed_time_);

  // Sleep
  const auto sleep_till = current_time + std::chrono::duration_cast<clock::duration>(desired_duration);
  std::this_thread::sleep_until(sleep_till);
}

DmHWLoop::~DmHWLoop()
{
  loopRunning_ = false;
  if (loop_thread_.joinable())
    loop_thread_.join();
}
}  // namespace rm_hw

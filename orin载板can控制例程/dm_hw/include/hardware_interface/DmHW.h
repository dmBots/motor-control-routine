#pragma once

#include <vector>
#include <string>
#include <memory>
#include <unordered_map>

#include "hardware_interface/damiao.h"


namespace damiao
{
using sysclock = std::chrono::system_clock;
using duration = std::chrono::duration<double>;
//在main函数里调用
class DmHW 
{
public:
  DmHW() = default;

  bool init(std::vector<std::string>& bus_name) ;

  void read(const sysclock::time_point& time, const duration& period) ;

  void write(const sysclock::time_point& time, const duration& period) ;

  void setCanBusThreadPriority(int thread_priority);

private:

  bool is_actuator_specified_ = false;
  int thread_priority_;

  std::vector<std::shared_ptr<damiao::Motor_Control>> motor_ports_{};

  std::unordered_map<std::string, std::unordered_map<uint16_t, damiao::DmActData>> bus_id2dm_data_{};
  
};

}  // namespace rm_hw

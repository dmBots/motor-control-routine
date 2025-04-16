#include "hardware_interface/DmHW.h"

namespace damiao
{
bool DmHW::init(std::vector<std::string>& bus_name)
{
  for (const std::string& bus : bus_name) 
  {
    //初始化一个canid为0x01，mstid为0x11，mit模式的DM4310电机
    uint16_t canid = 0x01;
    uint16_t mstid = 0x11;

    bus_id2dm_data_[bus].insert(std::make_pair(canid, DmActData{                                                          
                                                          .motorType = DM4310,
                                                          .mode = MIT_MODE,
                                                          .can_id=canid,
                                                          .mst_id=mstid,
                                                          .pos = 0,
                                                          .vel = 0,
                                                          .effort = 0,
                                                          .cmd_pos = 0,
                                                          .cmd_vel = 0,
                                                          .cmd_effort = 0 }));
   motor_ports_.push_back(std::make_shared<Motor_Control>(bus,&bus_id2dm_data_[bus]));
  }

  return true;
}

void DmHW::read(const sysclock::time_point& time, const duration& period)
{
  //遍历所有Motor_Control 一个Motor_Control代表一个can网络，其实只有一个can0  
  for(auto motor_port : motor_ports_)
  {
    motor_port->read();
  }
}

//定义DmHW类的write成员函数，该函数用于在给定的时间和周期内更新机器人的硬件状态  
void DmHW::write(const sysclock::time_point& time, const duration& period)
{
  //遍历所有Motor_Control 一个Motor_Control代表一个can网络，其实只有一个can0  
  for(auto motor_port : motor_ports_)
  {
    motor_port->write();
  } 
}

void DmHW::setCanBusThreadPriority(int thread_priority)
{
  thread_priority_ = thread_priority;
}


}  // namespace damiao

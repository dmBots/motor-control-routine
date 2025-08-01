/********************************************************************************
Modified Copyright (c) 2024-2025, Dm Robotics. 
********************************************************************************/

#include "dm_hw/DmHW.h"
#include "std_msgs/Float64MultiArray.h"
namespace damiao
{

bool DmHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  uint16_t canid1 = 0x01;
  uint16_t mstid1 = 0x11;
  uint16_t canid2 = 0x02;
  uint16_t mstid2 = 0x12;
  uint16_t canid3 = 0x03;
  uint16_t mstid3 = 0x13;
  uint16_t canid4 = 0x04;
  uint16_t mstid4 = 0x14;
  uint16_t canid5 = 0x05;
  uint16_t mstid5 = 0x15;
  uint16_t canid6 = 0x06;
  uint16_t mstid6 = 0x16;

  uint32_t nom_baud =1000000;//仲裁域 1M
  uint32_t dat_baud =5000000;//数据域 5M  
  std::vector<damiao::DmActData> init_data;
  init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
                                            .mode = damiao::MIT_MODE,
                                            .can_id=canid1,
                                            .mst_id=mstid1 });

//  init_data.push_back(damiao::DmActData{.motorType = damiao::DM4340,
//     .mode = damiao::POS_VEL_MODE,
//     .can_id=canid2,
//     .mst_id=mstid2 });

//   init_data.push_back(damiao::DmActData{.motorType = damiao::DM4340,
//     .mode = damiao::POS_VEL_MODE,
//     .can_id=canid3,
//     .mst_id=mstid3 });

//   init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
//     .mode = damiao::POS_VEL_MODE,
//     .can_id=canid4,
//     .mst_id=mstid4 });

//   init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
//     .mode = damiao::POS_VEL_MODE,
//     .can_id=canid5,
//     .mst_id=mstid5 });

//   init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
//     .mode = damiao::POS_VEL_MODE,
//     .can_id=canid6,
//     .mst_id=mstid6 });

  motorsInterface = std::make_shared<damiao::Motor_Control>(nom_baud,dat_baud,
    "14AA044B241402B10DDBDAFE448040BB",&init_data);

  registerInterface(&jointStateInterface_);//注册jointStateInterface_接口
  registerInterface(&hybridJointMitInterface_);//mit模式就注册这个hybridJointMitInterface_接口
  //registerInterface(&hybridJointPosInterface_);//Pos模式就注册这个hybridJointPosInterface_接口
  //registerInterface(&hybridJointVelInterface_);//Vel模式就注册这个hybridJointVelInterface_接口

  jointData_.resize(init_data.size());
  dmSendcmd_.resize(init_data.size());
  for (size_t index = 0; index < init_data.size(); ++index)
  {

    hardware_interface::JointStateHandle state_handle("joint" + std::to_string(index), &jointData_[index].pos_, &jointData_[index].vel_,
                                                      &jointData_[index].tau_);
    jointStateInterface_.registerHandle(state_handle);
    hybridJointMitInterface_.registerHandle(HybridJointMitHandle(state_handle, &jointData_[index].pos_des_,
                                                           &jointData_[index].vel_des_, &jointData_[index].kp_,
                                                           &jointData_[index].kd_, &jointData_[index].ff_));
  }

  time_pub_ = root_nh.advertise<std_msgs::Float64MultiArray>("time_interval", 20);
  return true;
}


void DmHW::read(const ros::Time& time, const ros::Duration& period)
{
  double pos,vel,tau;
  for (int i=0; i<jointData_.size(); i++)
  {
    pos=motorsInterface->getMotor(i+1)->Get_Position();
    vel=motorsInterface->getMotor(i+1)->Get_Velocity();
    tau=motorsInterface->getMotor(i+1)->Get_tau();

    jointData_[i].pos_ = pos ;
    jointData_[i].vel_ = vel ;
    jointData_[i].tau_ = tau ;
  }//电机到joint的映射
  
  std_msgs::Float64MultiArray time_msg;
  time_msg.data.resize(jointData_.size());
  for (size_t i = 0; i < jointData_.size(); ++i)
  {  
    time_msg.data[i] = motorsInterface->getMotor(i+1)->getTimeInterval();
  }
  std::cerr<<"pos: "<<jointData_[0].pos_<<" "
  <<"vel: "<<jointData_[0].vel_<<" "
  <<"tor: "<<jointData_[0].tau_<<" "
  <<"time_interval(s): "<<time_msg.data[0]<<std::endl;
  time_pub_.publish(time_msg);
}


void DmHW::write(const ros::Time& time, const ros::Duration& period)
{
  for (int i = 0; i < jointData_.size(); ++i)//as the urdf rank
  {
    dmSendcmd_[i].kp_ = jointData_[i].kp_;
    dmSendcmd_[i].kd_ = jointData_[i].kd_;
    //从joint到电机的方向映射
    dmSendcmd_[i].pos_des_ = jointData_[i].pos_des_ ;
    dmSendcmd_[i].vel_des_ = jointData_[i].vel_des_;
    
    dmSendcmd_[i].ff_ = jointData_[i].ff_;  
  }

  for (int i = 0; i < jointData_.size(); ++i)
  {
    motorsInterface->control_mit(*motorsInterface->getMotor(i+1),dmSendcmd_[i].kp_,dmSendcmd_[i].kd_ ,
                                  dmSendcmd_[i].pos_des_,dmSendcmd_[i].vel_des_, dmSendcmd_[i].ff_);
  }

  // control_mit(Motor &DM_Motor, float kp, float kd, float q, float dq, float tau)
}


}  // namespace damiao

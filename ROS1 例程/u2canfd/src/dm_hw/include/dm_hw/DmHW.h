
//
// Created by qiayuan on 6/24/22.
//

/********************************************************************************
Modified Copyright (c) 2024-2025, Dm Robotics. 
********************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>


// ROS control

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <dm_common/HybridJointInterface.h>
#include "dmbot_serial/protocol/damiao.h"
#include <urdf/model.h>

//这主要是硬件通信传输数据
namespace damiao
{

struct DmMotorData
{
  double pos_, vel_, tau_;
  double pos_des_, vel_des_, kp_, kd_, ff_;
};

class DmHW : public hardware_interface::RobotHW
{
public:
  DmHW() = default;

  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;

protected:

  // Interface
  hardware_interface::JointStateInterface jointStateInterface_;  
  HybridJointMitInterface hybridJointMitInterface_;                    
  HybridJointPosInterface hybridJointPosInterface_; 
  HybridJointVelInterface hybridJointVelInterface_; 

  std::shared_ptr<damiao::Motor_Control> motorsInterface;
private:
  std::vector<damiao::DmMotorData> jointData_;
  std::vector<damiao::DmMotorData> dmSendcmd_;

  //std::vector<int> directionMotor_{ 1, 1, -1, -1, -1, -1};//65432

  ros::Publisher time_pub_;
};

}  // namespace legged

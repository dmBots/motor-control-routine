#include "dm_controllers/DmController.h"

namespace damiao
{

bool DmController::init(HybridJointMitInterface* robot_hw, ros::NodeHandle& nh)
{
  std::cerr<<"Successfully got HybridEffort joint interface"<<std::endl;
 
  std::vector<std::string> joint_names{
    "joint0"
  };
  // std::vector<std::string> joint_names{
  //      "joint0","joint1","joint2","joint3","joint4","joint5"
  //    };
  for (const auto& joint_name : joint_names)
  {
    hybridJointMitHandles_.push_back(robot_hw->getHandle(joint_name));
  }
  
  return true;
}

void DmController::starting(const ros::Time& time)
{
 // ROS_INFO("DmController started.");
 std::cerr<<"DmController started."<<std::endl;
}

void DmController::update(const ros::Time& time, const ros::Duration& period)
{
  // 设置关节命令
 // std::cerr<<"dmcontroller update"<<std::endl;
 //std::cerr<<"size: "<<hybridJointMitHandles_.size()<<std::endl;
  float q = sin(std::chrono::system_clock::now().time_since_epoch().count() / 1e9);
  //setCommand(double pos_des, double vel_des, double kp, double kd, double ff)
  hybridJointMitHandles_[0].setCommand(0.0, 2.0,0.0,0.3,0.0);
  // hybridJointMitHandles_[1].setCommand(0.0, 0.0,0.0,0.0,0.0);
  // hybridJointMitHandles_[2].setCommand(0.0, 0.0,0.0,0.0,0.0);
  // hybridJointMitHandles_[3].setCommand(0.0, 0.0,0.0,0.0,0.0);
  // hybridJointMitHandles_[4].setCommand(0.0, 0.0,0.0,0.0,0.0);
  // hybridJointMitHandles_[5].setCommand(0.0, 0.0,0.0,0.0,0.0);
}

void DmController::stopping(const ros::Time& time)
{
 // ROS_INFO("DmController stopped.");
  std::cerr<<"DmController stop."<<std::endl;
}


}  // namespace damiao

// 注册插件
PLUGINLIB_EXPORT_CLASS(damiao::DmController, controller_interface::ControllerBase);
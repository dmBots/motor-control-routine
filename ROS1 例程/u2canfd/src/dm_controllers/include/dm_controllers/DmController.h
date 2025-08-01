#ifndef MY_CONTROLLER_PLUGIN_DM_CONTROLLER_H
#define MY_CONTROLLER_PLUGIN_DM_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dm_common/HybridJointInterface.h>
namespace damiao
{


class DmController : public controller_interface::Controller<HybridJointMitInterface>
{
public:
  DmController() = default;
  ~DmController() = default;

  bool init(HybridJointMitInterface* robot_hw, ros::NodeHandle& nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

private:
  std::vector<HybridJointMitHandle> hybridJointMitHandles_;
 // std::vector<HybridJointPosHandle> hybridJointPosHandles_;
  //std::vector<HybridJointVelHandle> hybridJointVelHandles_;
};

}  // namespace my_controller_plugin

#endif  // MY_CONTROLLER_PLUGIN_DM_CONTROLLER_H
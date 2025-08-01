/********************************************************************************
Modified Copyright (c) 2024-2025, Dm Robotics. 
********************************************************************************/

#include "dm_hw/DmHWLoop.h"
#include "dm_hw/DmHW.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "damiao");
  ros::NodeHandle nh;
  ros::NodeHandle robot_hw_nh("damiao");

  ros::AsyncSpinner spinner(3);
  spinner.start();

  try
  {
    std::shared_ptr<damiao::DmHW> motor_dm_hw = std::make_shared<damiao::DmHW>();
    
    motor_dm_hw->init(nh, robot_hw_nh);
    
    damiao::DmHWLoop control_loop(nh, motor_dm_hw);

    ros::waitForShutdown();
  }
  catch (const ros::Exception& e)
  {
    ROS_FATAL_STREAM("Error in the hardware interface:\n"
                     << "\t" << e.what());
    return 1;
  }

  return 0;
}

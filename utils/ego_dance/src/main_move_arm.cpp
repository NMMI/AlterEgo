#include <ros/ros.h>
#include "move_arm.h"


int main(int argc, char **argv)
{
  double rateHZ = 400;
  
  ros::init(argc, argv, "move_arm");
  
  ros::NodeHandle nh;
  
  MoveArm move_arm;
  
  move_arm.dt = 1 / rateHZ;
  ros::Rate r(rateHZ);
  //Aspetta i sensori e pubblica la posizione delle braccia
  while((move_arm.acquire_meas_arm_l || move_arm.acquire_meas_arm_r) && ros::ok())
  {
    ros::spinOnce(); 
  }

  //
  while(ros::ok())
  {

    move_arm.set_arm_position("right");
    move_arm.set_arm_position("left");

    ros::spinOnce();
    r.sleep();
        
  }

  return 0;
}

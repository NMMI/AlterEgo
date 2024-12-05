#include <ros/ros.h>
#include "ego_dance.h"



int main(int argc, char **argv)
{
    double rateHZ = 400;
  
  ros::init(argc, argv, "ego_dance");
  
  ros::NodeHandle nh;
  std::cout << "creato nodehandle" << std::endl;

  ego_dance Obj;  // to declare the object of the class ego_dance (object is the variable of a class that is a datatype)
  std::cout << "fatto obj" << std::endl;
  
  Obj.dt = 1 / rateHZ;
  ros::Rate r(rateHZ);
  
  // double t_begin = ros::Time::now().toSec();
  // std::cout << "tempo_init = "<< t_begin <<  std::endl;
  
  std::cout << "Initialized node" << std::endl;


  while(ros::ok())
  {
    
    while(Obj.acquire_meas_arm_l == 0 || Obj.acquire_meas_arm_r == 0)
    {
      ros::spinOnce();  // to call the callback
      std::cout << "callback executed" << std::endl;
    }
    
    if (Obj.compute_initial_pose == 0)
    {
      Obj.compute_initial_hand_pose();
      std::cout << "Initial_hand_pose_acquired" << std::endl;
    }
    
    Obj.run();  // to use the members (variables and functions) of the class we use dot operator
    // //std::cout << "finito di fare run" << std::endl;
    ros::spinOnce();
    r.sleep();
        
  }
  return 0;
}

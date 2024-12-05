#include <ros/ros.h>
#include "ego_dance.h"



int main(int argc, char **argv)
{
  double rateHZ = 400;
  
  ros::init(argc, argv, "EgoDance");
  
  ros::NodeHandle nh;
  std::cout << "creato nodehandle" << std::endl;
  
  EgoDance Obj;  // to declare the object of the class EgoDance (object is the variable of a class that is a datatype)
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
      ros::spinOnce(); 
    }
    
    if (Obj.compute_initial_pose == 0)
    {
      Obj.compute_initial_hand_pose();
    } 
    
    Obj.run();
    Obj.stability_check();

    ros::spinOnce();
    r.sleep();
        
  }
  return 0;
}

#include <ros/ros.h>
#include <ros/rate.h>

// my headers
#include "lqr.h"


//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
  double rateHZ = 0;

  ros::init(argc, argv, "lqr_node");
  ros::NodeHandle nh;
  // Get param from roslaunch or yaml file
	std::string ns_name;
	ns_name = ros::this_node::getNamespace();
  nh.getParam(ns_name+"/lqr_frequency", rateHZ);
  std::cout << "LQR frequency: "<<rateHZ<<std::endl;


  lqr Obj;
  Obj.dt = 1 / rateHZ;
  ros::Rate r(rateHZ);


  while(ros::ok())
  {
    Obj.run();

    ros::spinOnce();
    r.sleep();
        
  }// end while()
  Obj.stop_motor();
  return 0;
}
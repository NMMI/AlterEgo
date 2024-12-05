#include <ros/ros.h>
#include "follow_me.h"



int main(int argc, char **argv)
{
  double rateHZ = 400;
  
  ros::init(argc, argv, "FollowMe");
  
  ros::NodeHandle nh;
  
  FollowMe FM_obj;  
  
  ros::Rate r(rateHZ);
  int status = 0;
  
  while(ros::ok())
  {
    
    
    switch(status)
    {
      case 0: //Calcolo posizione delle mani a riposo
        FM_obj.P0 = FM_obj.getArmPosition(); 
        status = 1; 
        break;
      case 1: 
        //Calcolo posizione delle mani attuale
        FM_obj.Pt = FM_obj.getArmPosition();
        FM_obj.e = FM_obj.computeError(FM_obj.P0.position, FM_obj.Pt.position);
        FM_obj.phi = FM_obj.computeOrientation(FM_obj.e); 
        FM_obj.run(FM_obj.phi, FM_obj.e);

        break;
    }

    ros::spinOnce();
    r.sleep();
        
  }
  return 0;
}

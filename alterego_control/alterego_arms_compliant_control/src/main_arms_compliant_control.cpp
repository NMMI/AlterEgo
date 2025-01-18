#include <ros/ros.h>
#include "arms_compliant_control.h"


int main(int argc, char **argv)
{
  double rateHZ = 100;
  
  ros::init(argc, argv, "arms_compliant_control");
    
  ArmsCompliantControl SM_obj;
  ros::Rate r(rateHZ);
  // std_msgs::Bool enable_arms_compliant_control;
  // enable_arms_compliant_control.data = true;
  while(ros::ok())
  {
    if(SM_obj.enable_compliant)
    {
      SM_obj.act_pos = SM_obj.getArmPosition(); 

      //Pubblica la posizione delle mani
      SM_obj.pub_posture.publish(SM_obj.act_pos);

    }
    ros::spinOnce();
    r.sleep();

  }
  return 0;
}

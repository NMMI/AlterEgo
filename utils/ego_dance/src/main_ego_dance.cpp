#include <ros/ros.h>
#include "move_arm.h"
#include "ego_dance.h"



int main(int argc, char **argv)
{
  double rateHZ = 400;
  
  std::string kill_node;
  int state = 0;
  ros::init(argc, argv, "main_ego_dance");
  std::string ns = ros::this_node::getName();
  ros::NodeHandle nh;
  
  MoveArm ma_obj;
  EgoDance controller_obj;
  
  ma_obj.dt = 1 / rateHZ;
  ros::Rate r(rateHZ);

  kill_node = "rosnode kill "+ns;
  std::cout<<"\nNODE TO BE KILLED:"<<kill_node<<std::endl;

  //Wait the sensors node to be run
  while((ma_obj.acquire_meas_arm_l || ma_obj.acquire_meas_arm_r) && ros::ok())
  {
    ros::spinOnce(); 
  }

  while(ros::ok())
  {
    switch(state){

      case(SetArmPosition):                                       // SetArmPosition
        //Set arm
        while(!ma_obj.position_reached && ros::ok()){
          ma_obj.set_arm_position("right");
          ma_obj.set_arm_position("left");
          ros::spinOnce();
          r.sleep();
        }
        // Wait 3s in order to get a stable position
        ros::Duration(3.0).sleep();
        state = ComputeInitPose;
        break;
      
      case(ComputeInitPose):                                      // ComputeInitPose
        controller_obj.compute_initial_hand_pose();
        state = Run;
        break;
            
      case(Run):                                                  // Run
        controller_obj.stability_check();
        //Stability check and if it is good it runs
        if(controller_obj.stability_checked){
          controller_obj.run();
          // controller_obj.matlab_debug();
        }
        else{
          state = Reset;
        }
        break;

      case(Reset):                                                // Reset
        ma_obj.reset_arm();
        if(ma_obj.reset_done)
          int res = system(kill_node.c_str());
        break;

      default:
        break;
    }
    // controller_obj.print_data();
    ros::spinOnce();
    r.sleep();
        
  }

  return 0;
}

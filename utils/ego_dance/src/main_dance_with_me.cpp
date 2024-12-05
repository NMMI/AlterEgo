#include <ros/ros.h>
#include "follow_me.h"
#include "dance_with_me.h"
#include "ego_dance.h"
#include <cstdlib>
int main(int argc, char **argv)
{
  double rateHZ = 400;
  
  ros::init(argc, argv, "DanceWithMe");
  
  ros::NodeHandle nh;
  
  DanceWithMe FM_obj;
  EgoDance ED_obj;

  ros::Rate r(rateHZ);
  int arm_status = 0;
  int status_manager = 0;

  while(ros::ok())
  {
    
    //Get arm position offset


    switch(arm_status)
    {
      case GET_P0:

        FM_obj.P0_left = FM_obj.getArmPosition("left");
        FM_obj.P0_right = FM_obj.getArmPosition("right");
        ED_obj.compute_initial_hand_pose();
        if(ED_obj.acquire_meas_arm_l) arm_status = GET_Pt;
      break;

      case GET_Pt:
        FM_obj.Pt_left = FM_obj.getArmPosition("left");
        FM_obj.Pt_right = FM_obj.getArmPosition("right");
      
        //Compute Error
        FM_obj.e_left = FM_obj.computeError(FM_obj.P0_left.position, FM_obj.Pt_left.position);
        FM_obj.e_right = FM_obj.computeError(FM_obj.P0_right.position, FM_obj.Pt_right.position);

        //Compute norm
        FM_obj.e_norm_left = sqrt(pow(FM_obj.e_left.x,2) + pow(FM_obj.e_left.y,2) + pow(FM_obj.e_left.z,2));
        FM_obj.e_norm_right = sqrt(pow(FM_obj.e_right.x,2) + pow(FM_obj.e_right.y,2) + pow(FM_obj.e_right.z,2));

        //Compute Orientation
        FM_obj.phi_left = FM_obj.computeOrientation(FM_obj.e_left); 
        FM_obj.phi_right = FM_obj.computeOrientation(FM_obj.e_right); 

        //Check status of the arms
        
        //check che la velocità sia nulla e poi passo allo stato successivo

    



        //Switch status
        switch(status_manager)
        {
          case WAIT:
            if( FM_obj.e_norm_left > 0.05 && FM_obj.e_norm_right < 0.05){ 
              if(FM_obj.control_law.linear.x == 0 && FM_obj.control_law.angular.z == 0)
                status_manager = LEFT;
            }
            else if( FM_obj.e_norm_left < 0.05 && FM_obj.e_norm_right > 0.05){
              if(FM_obj.control_law.linear.x == 0 && FM_obj.control_law.angular.z == 0)
                status_manager = RIGHT;
            }


          break;
          case LEFT:

            FM_obj.run(FM_obj.phi_left, FM_obj.e_left, FM_obj.e_norm_left );

            if( FM_obj.e_norm_left > 0.05 && FM_obj.e_norm_right > 0.05){
                if(FM_obj.control_law.linear.x < 0.1 && FM_obj.control_law.angular.z < 0.1)
                  status_manager = DANCE;
            }
            else if(FM_obj.control_law.linear.x == 0 && FM_obj.control_law.angular.z == 0.0)
                status_manager = WAIT;

          break;
          case RIGHT:
            FM_obj.run(FM_obj.phi_right, FM_obj.e_right, FM_obj.e_norm_right );

            if( FM_obj.e_norm_left > 0.05 && FM_obj.e_norm_right > 0.05){
              if(FM_obj.control_law.linear.x < 0.1 && FM_obj.control_law.angular.z < 0.1)
                status_manager = DANCE;
            }
            else if(FM_obj.control_law.linear.x == 0 && FM_obj.control_law.angular.z == 0.0)
                status_manager = WAIT;


          break;
          case DANCE:
            FM_obj.control_law.linear.x = 0;
            FM_obj.control_law.angular.z = 0;
            ED_obj.run();
            if( FM_obj.e_norm_left < 0.05 && FM_obj.e_norm_right < 0.05){
              if(ED_obj.control_law.linear.x == 0 && ED_obj.control_law.angular.z == 0.0)
                  status_manager = WAIT;
            }

          break;

          default:
          break;
        }

      break;
    }



    ros::spinOnce();
    r.sleep();
        
  }
  return 0;
}

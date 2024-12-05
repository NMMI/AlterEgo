#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <rbdl/rbdl.h>
#include <rbdl/Constraints.h>
#include <urdf/model.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <geometry_msgs/Quaternion.h>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/JointState.h>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <tf/transform_broadcaster.h>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <ego_msgs/AlteregoState.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
#include <numeric>

#include <ros/package.h>
#include <std_msgs/Int8.h>
#define GET_P0 0
#define GET_Pt 1

#define WAIT 0
#define LEFT 1
#define RIGHT 2
#define DANCE 3



class DanceWithMe // class: user defined datatype
{               // we define the members (variables==properties and functions==methods) of the class
 public:         // variables and functions declare here are accessible aslo outside the class
   DanceWithMe();
   ~DanceWithMe();
   geometry_msgs::Pose   getArmPosition(std::string arm_side);
   geometry_msgs::Point  computeError(geometry_msgs::Point p0, geometry_msgs::Point pt);
   double computeOrientation(geometry_msgs::Point delta_p);
   void run(double delta_phi, geometry_msgs::Point delta_p, double e_norm);

   std::vector<geometry_msgs::Pose> P0;
   geometry_msgs::Pose		P0_right, P0_left, Pt_right, Pt_left;
   geometry_msgs::Point e_left, e_right;

   double phi_left, phi_right;
   double e_norm_left, e_norm_right, delta_e_norm;
   geometry_msgs::Twist control_law;
   
 private:
    ros::NodeHandle nh;
    int AlterEgoVersion;
    double lambda_1, lambda_2;
    std::string arm_side;
    double orientation;
    geometry_msgs::Pose		hand_pose;
    tf::TransformListener listener;
    tf::StampedTransform transform;
	  std::string CMD_VEL_IN_topic;

    double R_;                                                                                        //Raggio ruota
    double v, w, w_old, v_old;
  
    // // ---------------------------------------------------------------------------------------------------publisher:
    ros::Publisher pub_des_vel;                                                                          // Linear velocity and Angular rate desired to publish to segway_des_vel


};

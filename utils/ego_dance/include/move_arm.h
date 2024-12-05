#include <ros/ros.h>
#include <ros/package.h>

#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <boost/scoped_ptr.hpp>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include <ego_msgs/AlteregoState.h>

class MoveArm // class: user defined datatype
{             // we define the members (variables==properties and functions==methods) of the class
public:       // variables and functions declare here are accessible aslo outside the class
   MoveArm();
   ~MoveArm();
   void set_arm_position(std::string side);
   void reset_arm();

   double dt;
   bool acquire_meas_arm_l;
   bool acquire_meas_arm_r;
   bool position_reached;
   bool reset;
   bool reset_done;

private:
   ros::NodeHandle nh;

   void callback_robot_state(const ego_msgs::AlteregoState::ConstPtr &msg);
   // ---------------------------------------------------------------------------------------------------publisher:
   ros::Publisher pub_ref_eq_arm_left;
   ros::Publisher pub_ref_pr_arm_left;
   ros::Publisher pub_ref_eq_arm_right;
   ros::Publisher pub_ref_pr_arm_right;
   // ---------------------------------------------------------------------------------------------------subscriber:

   ros::Subscriber robot_state;

   std_msgs::Float64MultiArray arm_eq_ref_left_msg;
   std_msgs::Float64MultiArray arm_pr_ref_left_msg;
   std_msgs::Float64MultiArray arm_eq_ref_right_msg;
   std_msgs::Float64MultiArray arm_pr_ref_right_msg;

   Eigen::VectorXd q_send_left;
   Eigen::VectorXd q_send_left_old;
   Eigen::VectorXd q_send_right;
   Eigen::VectorXd q_send;
   Eigen::VectorXd q_send_right_old;
   Eigen::VectorXd meas_cube_shaft_left;
   Eigen::VectorXd meas_cube_shaft_right;

   std::vector<double> stiffn_vec_left;
   std::vector<double> stiffn_vec_right;
   int n_joints;
   int index_move_arm;
   int degree;
   int sec;
   int AlterEgoVersion;
   int bias;

   bool arm_cb_left;
   bool arm_cb_right;
   bool use_addon;

   double k;
   double pitch_angle;
};

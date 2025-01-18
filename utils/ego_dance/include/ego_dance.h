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
#include <alterego_msgs/UpperBodyState.h>
#include <alterego_msgs/LowerBodyState.h>
#include <geometry_msgs/TwistStamped.h>

#include <boost/scoped_ptr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <ros/package.h>
#include <std_msgs/Int8.h>

#define MAX_LIN_VEL 2.0
#define MAX_ANG_VEL 0.5
#define pi 3.1416
#define ENABLING 1
#define STARTING 2
#define WAITING 3

#define SetArmPosition 0
#define ComputeInitPose 1
#define Run 2
#define Reset 3

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class EgoDance // class: user defined datatype
{               // we define the members (variables==properties and functions==methods) of the class
 public:         // variables and functions declare here are accessible aslo outside the class
   EgoDance();
   ~EgoDance();

   void compute_kdl_tree();
   void compute_initial_hand_pose();
   void run();
   void move_arm();
   void matlab_debug();
   void print_data();
   void stability_check();
   void Enabled();

   Model *model;


   double dt;

   int acquire_meas_base;
   int acquire_meas_wheel;
   int acquire_meas_arm;
   int compute_initial_pose;

   int acquire_meas_base_real;
   int acquire_meas_arm_l;
   int acquire_meas_arm_r;
   geometry_msgs::Twist control_law;

   bool VERBOSE = true;
   bool stability_checked;
   int flag;
   int AlterEgoVersion;
   int flag_arms_ready;
   int 	mode;

   std::vector<std::string> body_names_;

   // flag when the arm is ready
   sensor_msgs::JointState joint_state_msg_;
   alterego_msgs::AlteregoState state_info_msg;


   
 private:
    void callback_upperbody_state(const alterego_msgs::UpperBodyState::ConstPtr &msg);
    void callback_lowerbody_state(const alterego_msgs::LowerBodyState::ConstPtr &msg);
    void callback_switch_mode(const std_msgs::Int8::ConstPtr &msg);

    void new_target_points();
    void transformation_1();
    void transformation_1_new();
    void controller_1();

    void compute_error();

    void transformation_2();
    void transformation_2_new();
    void controller_2();

    // void transformation_3();
    void transformation_3_new();
    void compute_error_3();
    void compute_vel_3();
    void controller_3();

    void compute_points();
    void unicycle_kinematics();
    int  sgn(double d);
    ros::NodeHandle nh;

    VectorNd Q; 
    std::string kill_node;
    double K, Kv, Kb, lambda_1, lambda_2, lambda_3, lambda_4, k, lambda_5, lambda_6, k1;              //Gains
    double R_;                                                                                        //Raggio ruota
    double v_ref, omega_ref;                                                                            
    double diff_angle;
    double theta_ref, theta_ref_old, eta;
    double v, omega, omega_old, v_old, linear_vel_real, omega_real, omega_ref_old, v_ref_old;
    double arm_L_q_addon_init;
    double arm_R_q_addon_init;
    double t;
    double t_old;
    std::string CMD_VEL_IN_topic;
    Eigen::VectorXd q;
    Eigen::VectorXd Pt1, Pt2;
    Eigen::VectorXd q_l;
    Eigen::VectorXd Pt1_l, Pt2_l, Pm_l, Pt1_t, Pt2_t;
    Eigen::VectorXd delta1_l, delta1_l_init, delta2_l, delta2_l_init, delta_l;
    Eigen::VectorXd Pm_ref, Pm_ref_old, Pm_ref_l;
    Eigen::Vector3d euler;
    Eigen::VectorXd base_pos, R_hand_pos, L_hand_pos;
    Eigen::Quaterniond base_pos_quat, R_hand_pos_quat, L_hand_pos_quat;
    Eigen::Matrix3d base_rot_mat,L_hand_rot_mat, R_hand_rot_mat;
    // ---------------------------------------------------------------------------------------------------publisher:
    ros::Publisher controller;                                                                          // Linear velocity and Angular rate desired to publish to segway_des_vel
    ros::Publisher rosbag_1, rosbag_2, rosbag_3, rosbag_4, time, ref;                                   // Debug for Matlab
    ros::Publisher joints_pub;                                                                          // Joint_states to publish to visualize the real time robot on Rviz
    
    // ---------------------------------------------------------------------------------------------------subscriber:
    ros::Subscriber sub_upperbody_state, sub_lowerbody_state;
   	ros::Subscriber sub_switch_mode;

    // ----------------------------------------------------------------
    int control_1; // Controller in Bipolar Coordinates
    int control_2; // Controller New
    int control_3; // Controller from Lecture Notes

    // ----------------------------------------------------------------

    // for move_arm function:

    int status;   // stato dello switch
    int el_cycl;  // contatore
    int run_freq; // frequenza
    int enabling_sec;
    double t_slerp;
    int starting_sec;
    std::string  				IN_topic_switch_mode;

    Eigen::Vector3d cmd_R_p, cmd_L_p;
    Eigen::Quaterniond cmd_R_q, cmd_L_q;

    Eigen::Quaterniond idle_R_q, idle_L_q ; // orientation del braccio destro
    Eigen::Vector3d idle_R_p, idle_L_p;

    Eigen::Quaterniond rest_R_q, rest_L_q;
    Eigen::Vector3d rest_R_p, rest_L_p; // pose desiderata fake joystick
    ros::Publisher pub_button_A;
    std_msgs::Bool msg_button_A;
    geometry_msgs::Pose msg_post_R, msg_post_L;
    ros::Publisher pub_post_R,  pub_post_L;

    // Rviz

    tf::Transform ik_tf0, ik_tf1, ik_tf2;
    tf::TransformBroadcaster ik_br0, ik_br1, ik_br2;




    // kdl to run controller_test.cpp
    
    // std::string pathURDF;
    // KDL::Tree kdl_tree;
    // KDL::Chain KDLChain;
    // KDL::JntArray qChain;
    // KDL::Frame Frame;
    // KDL::Rotation R_target;
    // Eigen::Vector3d xc;
    // double roll, pitch, yaw;


};

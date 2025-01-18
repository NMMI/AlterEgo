#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <algorithm>
#include <alterego_msgs/LowerBodyState.h>
#include <alterego_msgs/DebugLQR.h>

// ROS cuustom msg

#include <qb_interface/cubeRef.h>
#include <qb_interface/cubeCurrent.h>
#include <qb_interface/cubePos.h>

#define PI 3.1416
#define PI2 6.2832
#define MAX_LIN_VEL 2
#define MAX_ANG_VEL 0.5
#define PWM_EXT_RANGE_K 24 // PWM Extended Range [0-2400]

class lqr
{
public:
  lqr();
  ~lqr();

  void run();
  void stop_motor();
  double dt;

private:

  // ---------------------------------------------------------------------------------------------------Handle

  ros::NodeHandle n_;


  // ---------------------------------------------------------------------------------------------------functions

  void lowerbody_state__Callback(const alterego_msgs::LowerBodyState::ConstPtr &msg);

  void callback_des_vel(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void callback_offset_phi(const std_msgs::Float32::ConstPtr& msg);
  void Arms_Compliant__Callback(const std_msgs::Bool::ConstPtr &msg);
  void R_Botton_A__Callback(const std_msgs::Bool::ConstPtr &msg);
  void FallBack__Callback(const std_msgs::Bool::ConstPtr &msg);
  int sgn(double d);
  double unwrap(double previousAngle, double newAngle);
  double angleDiff(double a, double b);

  // --------------------------------------------------------------------------------------------------- custom

  qb_interface::cubeRef comm_pub_;
  alterego_msgs::DebugLQR state_debug_;
  alterego_msgs::LowerBodyState lowerbody_state_msg_;
  // --------------------------------------------------------------------------------------------------- double

  double pos1_des;
  double pos2_des;
  double vel1_des;
  double vel2_des;
  double enc1_g, enc2_g;
  double vel1_enc, vel2_enc, vel1, vel2;
  double pos1, pos2;
  double a, meas_vel_x_, meas_yaw_, meas_yaw_rate_;
  double offset_pitch_;

  double meas_pitch_rate_; 
  double pitch_angle;      
  double command_int;
  double des_wheels_angular_pos_, des_wheels_angular_vel_, des_yaw_, des_yaw_rate_, sensor_2_;
  double meas_pos_x_;
  double enc1_, enc2_, enc1_old_, enc2_old_, enc1_of_, enc2_of_, vel1_old_, vel2_old_, vel_old_, w_old_;
  double R_, W_, N_;
  double trsh_L_, trsh_R_;
  double th_curr_, phi_curr_;
  double dphi_des_filt_, dphi_des_filt_old_;
  double des_wheels_angular_vel_filt_, des_wheels_angular_vel_filt_old_;
  double des_yaw_rate_filt_old_, des_yaw_rate_filt_;
  double com_R, com_L;

  // --------------------------------------------------------------------------------------------------- Subscriber

  ros::Subscriber sub_ego_state; 
  ros::Subscriber sub_lowerbody_state_; //new
  ros::Subscriber sub_gyro_, sub_acc_, sub_euler_, sub_des_vel, sub_lqr, sub_Button_A_R;
  ros::Subscriber sub_enc, sub_pitch_off_, sub_sensor_IR, sub_test_joy_android, sub_fallback;

  // --------------------------------------------------------------------------------------------------- Publisher

  ros::Publisher pub_comm_;
  ros::Publisher pub_rec_;
  ros::Publisher pub_right_wheel_;
  ros::Publisher pub_left_wheel_;
  ros::Publisher pub_base_lin_vel;
  ros::Publisher pub_base_ang_vel;
  ros::Publisher pub_debug_;

  ros::Time last_cmd_time_arms_compliant_control;

  // --------------------------------------------------------------------------------------------------- Eigen

  Eigen::Vector2d x_pre_, x_old_;
  Eigen::Matrix2d P_pre_, P_old_;
	Eigen::MatrixXd k_feed;
  Eigen::Vector3d gyro_, acc_, euler_;
  Eigen::VectorXd sensor_IR_, arr_sensor_;
	std::vector<double> k_feed_from_yaml;
  Eigen::MatrixXd command, command_feed;
	Eigen::MatrixXd state;

  // --------------------------------------------------------------------------------------------------- Bool

  bool enabled_pilot, enable_arms_compliant_control;
  bool flag_run1_, flag_run2_, flag_run3_, flag_run4_;
  bool fallback_;
  bool has_fallback_;
	bool sim;

  // --------------------------------------------------------------------------------------------------- Time
  ros::Time old_t_;
  ros::Time new_t_;
  ros::Time time_cmd_vel_, time_first_cmd_fall_, time_second_cmd_fall_;
  ros::Duration max_dur_cmd_vel_;
  ros::Time last_cmd_time;
  // --------------------------------------------------------------------------------------------------- Int


  int state_fallback_;
  int AlterEgoVersion;
  // --------------------------------------------------------------------------------------------------- String

  std::string IN_topic_button_dance;
  std::string robot_name;

}; // End of class SubscribeAndPublish

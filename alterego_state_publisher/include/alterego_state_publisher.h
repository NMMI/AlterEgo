#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <alterego_msgs/LowerBodyState.h>
#include <alterego_msgs/UpperBodyState.h>
#include <qb_interface/cubePos.h>

#define PI 3.1416
#define PI2 6.2832
#define SETOFFSET 0
#define GETMEASUREMENT 1


class alterego_state_publisher
{
public:
  alterego_state_publisher(double rate);
  ~alterego_state_publisher();

  void Publish();
  void LowerBodyState();
  void UpperBodyState();
  void Init();

  double dt;
  int AlterEgoVersion;
  int n_act_arm;


private:
  //-------------------------------------------BodyState Publishers


  ros::Publisher pub_lowerbody_; //-------------------------------------------LowerBodyState 
  ros::Publisher pub_upperbody_; //-------------------------------------------UpperBodyState 
  

  //-------------------------------------------LowerBodyState msgs
  double right_wheel_enc;
  double left_wheel_enc;
  double base_battery_voltage;
  double right_wheel_pos;
  double left_wheel_pos;
  double wheels_angular_pos;
  double right_wheel_vel;
  double left_wheel_vel;
  double wheels_angular_vel;
  double mobile_base_pos_x;
  double mobile_base_pos_y;
  double mobile_base_lin_displacement;
  double mobile_base_lin_vel;
  double yaw_angle;
  double yaw_rate;
  double mobile_base_ang_vel;
  double pitch_angle;
  double pitch_rate;

  //-------------------------------------------LowerBodyState Subscriber
  ros::Subscriber sub_meas_encoders_;
  ros::Subscriber sub_pitch_angle_;
  ros::Subscriber sub_pitch_rate_;

  //-------------------------------------------UpperBodyState Subscriber

  ros::Subscriber arm_Q_left;
  ros::Subscriber arm_Q_right;
  ros::Subscriber motor_m1_left;
  ros::Subscriber motor_m1_right;
  ros::Subscriber motor_m2_left;
  ros::Subscriber motor_m2_right;
  ros::Subscriber neck_Q_yaw;
  ros::Subscriber neck_Q_pitch;

  ros::Subscriber left_weight_sub;
  ros::Subscriber right_weight_sub;
  double prevOutput;
  bool initialized = false;
  double pitch_rate_msg_double;
  ros::Time last_time_encoders;
  ros::Duration max_dur_encoders;

  // ---------------------------------------------------------------

  void left_arm__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
  void right_arm__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
  void yaw_neck__Callback(const std_msgs::Float64::ConstPtr &msg);
  void pitch_neck__Callback(const std_msgs::Float64::ConstPtr &msg);
  void left_m1__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
  void right_m1__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
  void left_m2__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
  void right_m2__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
  void right_weight__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
  void left_weight__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg);

  // ---------------------------------------------------------------

  void meas_encoders__Callback(const qb_interface::cubePos::ConstPtr &msg);
  void pitch_rate__Callback(const geometry_msgs::Vector3::ConstPtr &msg);
  void pitch_angle__Callback(const geometry_msgs::Vector3::ConstPtr &msg);

  // ---------------------------------------------------------------

  ros::NodeHandle nh;

  double encL_g, encR_g, velL_enc, velR_enc, velL, velR, posL, posR, enc_filter_coeff, dth, phi, dphi;

  std::vector<double> left_a_mot_;
  std::vector<double> left_k_mot_;
  std::vector<double> right_a_mot_;
  std::vector<double> right_k_mot_;
  Eigen::Vector3d gyro_;

  int enc_state = 0;
  bool acquire_meas_arm_l;
  bool acquire_meas_arm_r;

  bool acquire_meas_m1_l;
  bool acquire_meas_m1_r;

  bool acquire_meas_m2_l;
  bool acquire_meas_m2_r;

  double arm_L_q_addon_init;
  double arm_R_q_addon_init;

  double m1_L_q_addon_init;
  double m1_R_q_addon_init;

  double m2_L_q_addon_init;
  double m2_R_q_addon_init;

  double linear_vel_real, angular_vel_real;
  double encL_, encR_, encL_old_, encR_old_, encL_of_, encR_of_, velL_old_, velR_old_, vel_old_, w_old_;
  double R_, W_, N_;
  bool flag_first_enc_;

  bool use_addon = false;

  alterego_msgs::LowerBodyState lowerbody_state_msgs;
  alterego_msgs::UpperBodyState upperbody_state_msgs;

  double angleDiff(double a, double b);
  double unwrap(double previousAngle, double newAngle);
};

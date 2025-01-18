// ROS Headers
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>

// Custom ROS messages


// General Headers
#include <vector>
#include <iostream>

#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <qb_interface/cubePos.h>
#include <qb_interface/cubeRef.h>

# define PI 3.1416
# define PWM_EXT_RANGE_K  24  // PWM Extended Range [0-2400]
# define Rm 0.84278
# define Kt 0.015056
# define n 160

using namespace std;


class sim_qb_class_wheels{

	public:
		// Costructor
		sim_qb_class_wheels();

		// Destructor
		~sim_qb_class_wheels();

		// SpinOnce function
		// void spinOnce();

		// Spin function
		void run();

	private:

		void read_joint_states__Callback(const sensor_msgs::JointState& msg);
		void read_gazebo_imu_plugin__Callback(const sensor_msgs::Imu& msg);
		void read_cube_ref__Callback(const qb_interface::cubeRef::ConstPtr& msg);
		void read_gazebo_link_state__Callback(const gazebo_msgs::LinkStates & msg);

		
		ros::NodeHandle node_;

		ros::Publisher    				pub_cube_measurement;
		ros::Publisher                  pub_right_wheel_command;
		ros::Publisher                  pub_left_wheel_command;
		ros::Publisher    				pub_gyro_good;
		ros::Publisher    				pub_RPY;

		ros::Subscriber                 sub_joint_states;
		ros::Subscriber                 sub_cube_ref;
		ros::Subscriber               	sub_gazebo_link_state;
		ros::Subscriber 	            sub_gazebo_imu_plugin;



		bool 							flag_run1_;
		bool 							flag_run2_;

		string 				            cube_measurement_topic;
		string 				            cube_ref_topic;
		string 				            joint_states_topic;
		string 				            right_wheel_command_topic;
		string 				            left_wheel_command_topic;
		string 		                    ns;

		double 							pos_L_js;
		double 							pos_R_js;
		vector<float>					pos_R_tk;
		vector<float> 					pos_L_tk;
		double 							com_R;
		double 							com_L;
		double 							command_R;
		double 							command_L;
		double 							N_, th, r_R, r_L, enc1_, enc2_;

		int 							q_R;
		int								q_L;
		int								sign;
		int 							pos_R_off;
		int	 						 	pos_L_off;
		std_msgs::Float64				command_L_msg;
		std_msgs::Float64 				command_R_msg;
		qb_interface::cubePos cube_measurement_msg;




		Eigen::Vector3d     euler_;
		geometry_msgs::Vector3 msg_gyro_good, msg_RPY;
		string 				gazebo_link_state_topic;
		string 				gazebo_imu_plugin_topic;
		string 				gyro_good_topic;
		string 				RPY_topic;


};
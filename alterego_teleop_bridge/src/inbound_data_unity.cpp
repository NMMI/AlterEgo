#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <alterego_msgs/EnableAutoModeService.h>
#include <alterego_msgs/WaitPilotService.h>
using namespace KDL;
using namespace std;

ros::Publisher pub_posture_L;
ros::Publisher pub_index_L;
ros::Publisher pub_hand_L;
ros::Publisher pub_Botton_X_L;
ros::Publisher pub_Botton_Y_L;
ros::Publisher pub_Thumbstick_x_L;
ros::Publisher pub_Thumbstick_y_L;

ros::Publisher pub_posture_R;
ros::Publisher pub_index_R;
ros::Publisher pub_hand_R;
ros::Publisher pub_Botton_A_R;
ros::Publisher pub_Botton_B_R;
ros::Publisher pub_Thumbstick_x_R;
ros::Publisher pub_Thumbstick_y_R;

ros::Publisher pub_posture_H;
ros::Publisher    	pub_des_vel;

ros::Publisher pub_stiff_L;
ros::Publisher pub_stiff_R;
ros::Publisher pub_mic;
ros::Publisher pub_alive;
ros::Publisher pub_auto_mode_status;

ros::ServiceServer enable_auto_mode_service;
ros::ServiceServer wait_help_from_pilot_service;

int AlterEgoVersion(3);

double vel_lin(0);
double vel_ang(0);
double scaling_factor_lin_vel(1);
double scaling_factor_ang_vel(1);
double alpha_vel(0.9);

Eigen::Matrix4d T_b_0_left;
Eigen::Matrix3d R_o_b_left;
Eigen::Matrix3d R_offset_left;

Eigen::Matrix4d T_b_0_right;
Eigen::Matrix3d R_o_b_right;
Eigen::Matrix3d R_offset_right;

Eigen::Matrix4d T_b_0_head;
Eigen::Matrix3d R_o_b_head;
Eigen::Matrix3d 	R_p_2_r_l;
Eigen::Matrix3d 	R_p_2_r_r;
std::vector<double>		R_p2r_l;
std::vector<double>		R_p2r_r;
double hand_cl;
double arm_l(0.504);
// Microfono
bool audio_off = false;
bool audio_state = false;
std_msgs::Bool com_state;
double 				shoulder_l(0.2);
double 				shoulder_r(0.2);
bool auto_mode_enabled = false;
bool move_base_enabled = false;

bool enable_out = false;
int restart_agent_cnt = 0;
Frame 				ref_frame;
ros::Time last_cmd_time, last_cmd_vel_time;
bool waitPilotCallback(alterego_msgs::WaitPilotService::Request &req, alterego_msgs::WaitPilotService::Response &res)
{
	if(req.request == true)
		restart_agent_cnt = 0;
    if (restart_agent_cnt==2)
    {
		restart_agent_cnt=0;
        res.success = true;
		return res.success;

    }
	res.success = false;
    return res.success;
}
bool enableAutoModeCallback(alterego_msgs::EnableAutoModeService::Request &req, alterego_msgs::EnableAutoModeService::Response &res)
{
    std_msgs::Bool status_msg;
    if (enable_out)
    {
        res.success = false;
        ROS_WARN("Cannot enable auto mode while enable_out is 1");
    }
    else
    {
        auto_mode_enabled = req.enable;
        res.success = true;
        ROS_INFO("Auto mode enabled: %s", auto_mode_enabled ? "true" : "false");
    }
    status_msg.data = auto_mode_enabled;
    pub_auto_mode_status.publish(status_msg);
    return res.success;
}

void L_posture__Callback(const geometry_msgs::Pose::ConstPtr &msg)
{
	if (enable_out)
	{ 
		Eigen::Quaterniond ref_quat;
		static Eigen::Quaterniond old_quat;
		double sign_check;
		Eigen::Vector3d r_p;
		Eigen::Matrix3d r_M;

		r_p << msg->position.x, msg->position.y, msg->position.z;
		r_p = r_p * arm_l;
		ref_quat.x() = msg->orientation.x;
		ref_quat.y() = msg->orientation.y;
		ref_quat.z() = msg->orientation.z;
		ref_quat.w() = msg->orientation.w;

		// sign_check = ref_quat.w() * old_quat.w() + ref_quat.x() * old_quat.x() + ref_quat.y() * old_quat.y() + ref_quat.z() * old_quat.z();
		// if (sign_check < 0.0)
		// {
		// 	ref_quat.w() = -ref_quat.w();
		// 	ref_quat.vec() = -ref_quat.vec();
		// }
		// old_quat = ref_quat;
		// ref_quat = (R_o_b_left * ref_quat * R_o_b_left.transpose()) * T_b_0_left.block<3, 3>(0, 0).transpose() * R_offset_left;
		// r_p = R_o_b_left * r_p + T_b_0_left.block<3, 1>(0, 3);


		//Unity
		ref_quat = Eigen::Quaterniond(R_o_b_left * R_offset_left * ref_quat.toRotationMatrix());
		r_p = T_b_0_left.block<3, 1>(0, 3) + r_p;   



		geometry_msgs::Pose msg_out;
		msg_out.orientation.x = ref_quat.x();
		msg_out.orientation.y = ref_quat.y();
		msg_out.orientation.z = ref_quat.z();
		msg_out.orientation.w = ref_quat.w();
		msg_out.position.x = r_p(0);
		msg_out.position.y = r_p(1);
		msg_out.position.z = r_p(2);
		pub_posture_L.publish(msg_out);
	}
}


void L_index_trigger__Callback(const std_msgs::Float64::ConstPtr &msg)
{
	if (enable_out)
		pub_index_L.publish(msg);
}

void L_hand_trigger__Callback(const std_msgs::Float64::ConstPtr &msg)
{
	if (enable_out)
		pub_hand_L.publish(msg);
}

void L_Thumbstick_y__Callback(const std_msgs::Float64::ConstPtr &msg)
{
	double vel_lin_act;

	if (enable_out)
	{
		pub_Thumbstick_y_L.publish(msg);
		vel_lin_act = msg->data * scaling_factor_lin_vel;
	}
	else
	{
		vel_lin_act = 0;
	}

	if(!auto_mode_enabled) vel_lin = alpha_vel * (vel_lin) + (1 - alpha_vel) * vel_lin_act; //solo se non sono autonomo
}

void L_Thumbstick_x__Callback(const std_msgs::Float64::ConstPtr &msg)
{
	if (enable_out)
		pub_Thumbstick_x_L.publish(msg);
}

void L_Botton_X__Callback(const std_msgs::Bool::ConstPtr &msg)
{
	if (enable_out)
		pub_Botton_X_L.publish(msg);
}

void L_Botton_Y__Callback(const std_msgs::Bool::ConstPtr &msg)
{
	if (enable_out)
		pub_Botton_Y_L.publish(msg);
}


void R_posture__Callback(const geometry_msgs::Pose::ConstPtr &msg)
{

	if (enable_out)
	{
		Eigen::Quaterniond ref_quat;
		static Eigen::Quaterniond old_quat;
		double sign_check;
		Eigen::Vector3d r_p;
		Eigen::Matrix3d r_M;

		r_p << msg->position.x, msg->position.y, msg->position.z;
		r_p = r_p * arm_l;
		ref_quat.x() = msg->orientation.x;
		ref_quat.y() = msg->orientation.y;
		ref_quat.z() = msg->orientation.z;
		ref_quat.w() = msg->orientation.w;

		// sign_check = ref_quat.w() * old_quat.w() + ref_quat.x() * old_quat.x() + ref_quat.y() * old_quat.y() + ref_quat.z() * old_quat.z();
		// if (sign_check < 0.0)
		// {
		// 	ref_quat.w() = -ref_quat.w();
		// 	ref_quat.vec() = -ref_quat.vec();
		// }
		// old_quat = ref_quat;
		// ref_quat = (R_o_b_right * ref_quat * R_o_b_right.transpose()) * T_b_0_right.block<3, 3>(0, 0).transpose() * R_offset_right;
		// r_p = R_o_b_right * r_p + T_b_0_right.block<3, 1>(0, 3);


		//Unity
		ref_quat = Eigen::Quaterniond(R_o_b_right * R_offset_right * ref_quat.toRotationMatrix());
		r_p = T_b_0_right.block<3, 1>(0, 3) + r_p;

		geometry_msgs::Pose msg_out;
		msg_out.orientation.x = ref_quat.x();
		msg_out.orientation.y = ref_quat.y();
		msg_out.orientation.z = ref_quat.z();
		msg_out.orientation.w = ref_quat.w();
		msg_out.position.x = r_p(0);
		msg_out.position.y = r_p(1);
		msg_out.position.z = r_p(2);
		pub_posture_R.publish(msg_out);
	}

}

void R_index_trigger__Callback(const std_msgs::Float64::ConstPtr &msg)
{
	if (enable_out)
		pub_index_R.publish(msg);
}

void R_hand_trigger__Callback(const std_msgs::Float64::ConstPtr &msg)
{
	if (enable_out)
		pub_hand_R.publish(msg);
}

void R_Thumbstick_y__Callback(const std_msgs::Float64::ConstPtr &msg)
{
	if (enable_out)
		pub_Thumbstick_y_R.publish(msg);
}

void L_IN_stiff__Callback(const std_msgs::Float64::ConstPtr &msg)
{
	if (enable_out)
		pub_stiff_L.publish(msg);
}

void R_IN_stiff__Callback(const std_msgs::Float64::ConstPtr &msg)
{
	if (enable_out)
		pub_stiff_R.publish(msg);
}

void R_Thumbstick_x__Callback(const std_msgs::Float64::ConstPtr &msg)
{
	double vel_ang_act;

	if (enable_out)
	{
		pub_Thumbstick_x_R.publish(msg);
		vel_ang_act = -msg->data * scaling_factor_ang_vel;
	}
	else
	{
		vel_ang_act = 0;
	}

	if(!auto_mode_enabled) vel_ang = alpha_vel * (vel_ang) + (1 - alpha_vel) * vel_ang_act;  //solo se non sono autonomo
}

void R_Botton_A__Callback(const std_msgs::Bool::ConstPtr &msg)
{


	if (msg->data == true )
	{

		std_msgs::Bool status_msg;

		enable_out = true;
		auto_mode_enabled = false;
		status_msg.data = auto_mode_enabled;
		cout << "Pilot in" << endl;
		pub_auto_mode_status.publish(status_msg);
		com_state.data = true;
		pub_Botton_A_R.publish(com_state);
		restart_agent_cnt = 1;

		
	}
	else{
		enable_out = false;
		cout << "Pilot out" << endl;
		com_state.data = false;
		pub_Botton_A_R.publish(com_state);
		restart_agent_cnt = 2;

	}



	last_cmd_time = ros::Time::now();
}




void R_Botton_B__Callback(const std_msgs::Bool::ConstPtr &msg)
{
	if (enable_out)
		pub_Botton_B_R.publish(msg);
}

void H_posture__Callback(const geometry_msgs::Pose::ConstPtr &msg)
{
	if (enable_out)
	{
		Eigen::Quaterniond ref_quat;
		static Eigen::Quaterniond old_quat;
		double sign_check;
		Eigen::Vector3d r_p;
		Eigen::Matrix3d r_M;

		r_p << msg->position.x, msg->position.y, msg->position.z;
		r_p.normalize();
		r_p *= 0.132;
		
		ref_quat.x() = msg->orientation.x;
		ref_quat.y() = msg->orientation.y;
		ref_quat.z() = msg->orientation.z;
		ref_quat.w() = msg->orientation.w;

		sign_check = ref_quat.w() * old_quat.w() + ref_quat.x() * old_quat.x() + ref_quat.y() * old_quat.y() + ref_quat.z() * old_quat.z();
		if (sign_check < 0.0)
		{
			ref_quat.w() = -ref_quat.w();
			ref_quat.vec() = -ref_quat.vec();
		}
		old_quat = ref_quat;

		// ref_quat = (R_o_b_head * ref_quat * R_o_b_head.transpose()) * T_b_0_head.block<3, 3>(0, 0).transpose();

		// r_p = R_o_b_head * r_p + T_b_0_head.block<3, 1>(0, 3);
		geometry_msgs::Pose msg_out;

		//how to double the pitch angle of ref_quat
		double roll, pitch, yaw;
		tf::Quaternion q(ref_quat.x(), ref_quat.y(), ref_quat.z(), ref_quat.w());
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);
		pitch = pitch * 2;
		tf::Quaternion q_new;
		q_new.setRPY(roll, pitch, yaw);
		ref_quat.x() = q_new.x();
		ref_quat.y() = q_new.y();
		ref_quat.z() = q_new.z();
		ref_quat.w() = q_new.w();

		msg_out.orientation.x = ref_quat.x();
		msg_out.orientation.y = ref_quat.y();
		msg_out.orientation.z = ref_quat.z();
		msg_out.orientation.w = ref_quat.w();
		msg_out.position.x = r_p(0);
		msg_out.position.y = r_p(1);
		msg_out.position.z = r_p(2);
		pub_posture_H.publish(msg_out);
	}

}

//callback di velocità che viene da move_base
void Cmd_vel__Callback(const geometry_msgs::Twist::ConstPtr &msg)
{
	double alpha_filtro(0.3);
	if(auto_mode_enabled){

		vel_lin = alpha_filtro * (vel_lin) + (1 - alpha_filtro) * msg->linear.x;
		vel_ang = alpha_filtro * (vel_ang) + (1 - alpha_filtro) * msg->angular.z;
		// vel_lin = msg->linear.x;
		// vel_ang = msg->angular.z;
		move_base_enabled = true;
		
	}
	last_cmd_vel_time = ros::Time::now();
}
void Cmd_vel_Unity__Callback(const geometry_msgs::Twist::ConstPtr &msg)
{
	double alpha_filtro(0.3);

	vel_lin = alpha_filtro * (vel_lin) + (1 - alpha_filtro) * msg->linear.x;
	vel_ang = alpha_filtro * (vel_ang) + (1 - alpha_filtro) * msg->angular.z;
		
	last_cmd_vel_time = ros::Time::now();
}



int main(int argc, char **argv)
{
	// string IN_topic_arms_compliant_control;
	string L_IN_topic_track;
	string L_IN_topic_index;
	string L_IN_topic_hand;
	string L_IN_topic_button_X;
	string L_IN_topic_button_Y;
	string L_IN_topic_thumbstick_x; 

	string L_IN_topic_thumbstick_y;
	string R_IN_topic_track;
	string R_IN_topic_index;
	string R_IN_topic_hand;
	string R_IN_topic_button_A;
	string R_IN_topic_button_B;
	string R_IN_topic_thumbstick_x;
	string R_IN_topic_thumbstick_y;
	string H_IN_topic_track;
	string CMD_VEL_IN_topic;
	
	string CMD_VEL_UNITY_topic ;

	string L_OUT_topic_track;
	string L_OUT_topic_index;
	string L_OUT_topic_hand;
	string L_OUT_topic_button_X;
	string L_OUT_topic_button_Y;
	string L_OUT_topic_thumbstick_x;
	string L_OUT_topic_thumbstick_y;
	string R_OUT_topic_track;
	string R_OUT_topic_index;
	string R_OUT_topic_hand;
	string R_OUT_topic_button_A;
	string R_OUT_topic_button_B;
	string R_OUT_topic_thumbstick_x;
	string R_OUT_topic_thumbstick_y;
	string H_OUT_topic_track;
	string BASE_VEL_topic;
	string L_IN_topic_stiff;
	string R_IN_topic_stiff;
	string L_OUT_topic_stiff;
	string R_OUT_topic_stiff;
	string BASE_IN_VEL_topic;
	string BASE_OUT_VEL_topic;
	string auto_mode_status_topic;
	

	ros::Subscriber sub_posture_L;
	ros::Subscriber sub_index_L;
	ros::Subscriber sub_hand_L;
	ros::Subscriber sub_Botton_X_L;
	ros::Subscriber sub_Botton_Y_L;
	ros::Subscriber sub_Thumbstick_x_L;
	ros::Subscriber sub_Thumbstick_y_L;

	ros::Subscriber sub_posture_R;
	ros::Subscriber sub_index_R;
	ros::Subscriber sub_hand_R;
	ros::Subscriber sub_Botton_A_R;
	ros::Subscriber sub_Botton_B_R;
	ros::Subscriber sub_Thumbstick_x_R;
	ros::Subscriber sub_Thumbstick_y_R;
  	ros::Subscriber sub_des_vel;

	ros::Subscriber sub_stiff_L;
	ros::Subscriber sub_stiff_R;
	ros::Subscriber sub_Button_dance;

	// Subscriber per il microfono
	ros::Subscriber sub_microphone;
	ros::Subscriber sub_posture_H;

	int run_freq;
	com_state.data = false;
	// geometry_msgs::Vector3 base_vel;
	geometry_msgs::TwistStamped base_vel;

	std_msgs::Bool alive_msg;
	ros::Duration max_cmd_latency = ros::Duration(2);
	int alive_cnt = 0;


	// ------------------------------------------------------------------------------------- Init node
	ros::init(argc, argv, "inbound_data");
	ros::NodeHandle n;
	run_freq = 100;
	ros::Rate loop_rate(run_freq);

	// ------------------------------------------------------------------------------------- Check/save args
	n.getParam("AlterEgoVersion", AlterEgoVersion);
  	std::cout<<"AlterEgoVersion: "<<AlterEgoVersion<<std::endl;
	
	// n.getParam("IN_topic_arms_compliant_control", IN_topic_arms_compliant_control);
	
	n.getParam("L_IN_topic_track", L_IN_topic_track);
	n.getParam("L_IN_topic_index", L_IN_topic_index);
	n.getParam("L_IN_topic_hand", L_IN_topic_hand);
	n.getParam("L_IN_topic_button_X", L_IN_topic_button_X);
	n.getParam("L_IN_topic_button_Y", L_IN_topic_button_Y);
	n.getParam("L_IN_topic_thumbstick_x", L_IN_topic_thumbstick_x);
	n.getParam("L_IN_topic_thumbstick_y", L_IN_topic_thumbstick_y);
	n.getParam("L_IN_topic_stiff", L_IN_topic_stiff);
	n.getParam("L_OUT_topic_stiff", L_OUT_topic_stiff);

	n.getParam("R_IN_topic_track", R_IN_topic_track);
	n.getParam("R_IN_topic_index", R_IN_topic_index);
	n.getParam("R_IN_topic_hand", R_IN_topic_hand);
	n.getParam("R_IN_topic_button_A", R_IN_topic_button_A);
	n.getParam("R_IN_topic_button_B", R_IN_topic_button_B);
	n.getParam("R_IN_topic_thumbstick_x", R_IN_topic_thumbstick_x);
	n.getParam("R_IN_topic_thumbstick_y", R_IN_topic_thumbstick_y);
	n.getParam("R_IN_topic_stiff", R_IN_topic_stiff);
	n.getParam("R_OUT_topic_stiff", R_OUT_topic_stiff);

	n.getParam("H_IN_topic_track", H_IN_topic_track);
	n.getParam("CMD_VEL_IN_topic", CMD_VEL_IN_topic);

	n.getParam("L_OUT_topic_track", L_OUT_topic_track);
	n.getParam("L_OUT_topic_index", L_OUT_topic_index);
	n.getParam("L_OUT_topic_hand", L_OUT_topic_hand);
	n.getParam("L_OUT_topic_button_X", L_OUT_topic_button_X);
	n.getParam("L_OUT_topic_button_Y", L_OUT_topic_button_Y);
	n.getParam("L_OUT_topic_thumbstick_x", L_OUT_topic_thumbstick_x);
	n.getParam("L_OUT_topic_thumbstick_y", L_OUT_topic_thumbstick_y);

	n.getParam("R_OUT_topic_track", R_OUT_topic_track);
	n.getParam("R_OUT_topic_index", R_OUT_topic_index);
	n.getParam("R_OUT_topic_hand", R_OUT_topic_hand);
	n.getParam("R_OUT_topic_button_A", R_OUT_topic_button_A);
	n.getParam("R_OUT_topic_button_B", R_OUT_topic_button_B);
	n.getParam("R_OUT_topic_thumbstick_x", R_OUT_topic_thumbstick_x);
	n.getParam("R_OUT_topic_thumbstick_y", R_OUT_topic_thumbstick_y);

	n.getParam("H_OUT_topic_track", H_OUT_topic_track);
	n.getParam("BASE_scaling_factor_lin_vel", scaling_factor_lin_vel);
	n.getParam("BASE_scaling_factor_ang_vel", scaling_factor_ang_vel);
	n.getParam("BASE_IN_VEL_topic", BASE_IN_VEL_topic);
	n.getParam("BASE_OUT_VEL_topic", BASE_OUT_VEL_topic);
	n.getParam("auto_mode_status_topic", auto_mode_status_topic);

	n.getParam("right/R_p2r", R_p2r_r);
	n.getParam("left/R_p2r", R_p2r_l);
	n.getParam("right/shoulder_l", shoulder_r);
	n.getParam("left/shoulder_l", shoulder_l);
	n.getParam("left/arm_l", arm_l);

	std::vector<double> T_t2s;
	std::vector<double> T_o2t;
	std::vector<double> R_o2b;
	std::vector<double> R_5to6;

	// //----------------	LEFT
	n.getParam("left/T_o2t", T_o2t);
	n.getParam("left/R_o2b", R_o2b);
	n.getParam("left/R_5to6", R_5to6);

	// std::cout << "\n --------LEFT --------- \n";

	T_b_0_left << T_o2t[0], T_o2t[1], T_o2t[2], T_o2t[3],
		T_o2t[4], T_o2t[5], T_o2t[6], T_o2t[7],
		T_o2t[8], T_o2t[9], T_o2t[10], T_o2t[11],
		T_o2t[12], T_o2t[13], T_o2t[14], T_o2t[15];

	// std::cout << "\n T_b_0_left\n"<<T_b_0_left<<std::endl;


	R_o_b_left << R_o2b[0], R_o2b[1], R_o2b[2],
		R_o2b[3], R_o2b[4], R_o2b[5],
		R_o2b[6], R_o2b[7], R_o2b[8];

	// std::cout << "\n R_o2b_left\n"<<R_o_b_left<<std::endl;

	R_offset_left << R_5to6[0], R_5to6[1], R_5to6[2],
		R_5to6[3], R_5to6[4], R_5to6[5],
		R_5to6[6], R_5to6[7], R_5to6[8];
	// std::cout << "\n R_5to6\n"<<R_offset_left<<std::endl;

	//---------------- 	RIGHT
	n.getParam("right/T_o2t", T_o2t);
	n.getParam("right/R_o2b", R_o2b);
	n.getParam("right/R_5to6", R_5to6);
	// std::cout << "\n --------RIGHT --------- \n";

	T_b_0_right << T_o2t[0], T_o2t[1], T_o2t[2], T_o2t[3],
		T_o2t[4], T_o2t[5], T_o2t[6], T_o2t[7],
		T_o2t[8], T_o2t[9], T_o2t[10], T_o2t[11],
		T_o2t[12], T_o2t[13], T_o2t[14], T_o2t[15];

	R_o_b_right << R_o2b[0], R_o2b[1], R_o2b[2],
		R_o2b[3], R_o2b[4], R_o2b[5],
		R_o2b[6], R_o2b[7], R_o2b[8];
		
	// std::cout << "\n R_o2b_right\n"<<R_o_b_right<<std::endl;

	R_offset_right << R_5to6[0], R_5to6[1], R_5to6[2],
		R_5to6[3], R_5to6[4], R_5to6[5],
		R_5to6[6], R_5to6[7], R_5to6[8];
	//---------------- 	HEAD
	// std::cout << "\n --------HEAD --------- \n";

	n.getParam("head/T_o2t", T_o2t);
	n.getParam("head/R_o2b", R_o2b);
	T_b_0_head << T_o2t[0], T_o2t[1], T_o2t[2], T_o2t[3],
		T_o2t[4], T_o2t[5], T_o2t[6], T_o2t[7],
		T_o2t[8], T_o2t[9], T_o2t[10], T_o2t[11],
		T_o2t[12], T_o2t[13], T_o2t[14], T_o2t[15];

	R_o_b_head << R_o2b[0], R_o2b[1], R_o2b[2],
		R_o2b[3], R_o2b[4], R_o2b[5],
		R_o2b[6], R_o2b[7], R_o2b[8];

	// std::cout << "\n R_o_b_head\n"<<R_o_b_head<<std::endl;


	R_p_2_r_l << R_p2r_l[0], R_p2r_l[1], R_p2r_l[2],
	R_p2r_l[3], R_p2r_l[4], R_p2r_l[5],
	R_p2r_l[6], R_p2r_l[7], R_p2r_l[8];

	R_p_2_r_r << R_p2r_r[0], R_p2r_r[1], R_p2r_r[2],
	R_p2r_r[3], R_p2r_r[4], R_p2r_r[5],
	R_p2r_r[6], R_p2r_r[7], R_p2r_r[8];
	
	// ------------------------------------------------------------------------------------- Subscribe to topics
	// sub_Button_dance = n.subscribe(IN_topic_arms_compliant_control, 1, Arms_Compliant__Callback);
	sub_posture_L = n.subscribe(L_IN_topic_track, 1, L_posture__Callback);
	sub_index_L = n.subscribe(L_IN_topic_index, 1, L_index_trigger__Callback);				// INDEX TRIGGER
	sub_hand_L = n.subscribe(L_IN_topic_hand, 1, L_hand_trigger__Callback);					// HAND TRIGGER
	sub_Botton_X_L = n.subscribe(L_IN_topic_button_X, 1, L_Botton_X__Callback);				// BUTTON 1
	sub_Botton_Y_L = n.subscribe(L_IN_topic_button_Y, 1, L_Botton_Y__Callback);				// BUTTON 2
	sub_Thumbstick_x_L = n.subscribe(L_IN_topic_thumbstick_x, 1, L_Thumbstick_x__Callback); // THUMBSTICK X-AXIS
	sub_Thumbstick_y_L = n.subscribe(L_IN_topic_thumbstick_y, 1, L_Thumbstick_y__Callback); // THUMBSTICK Y-AXIS
	sub_stiff_L = n.subscribe(L_IN_topic_stiff, 1, L_IN_stiff__Callback);					// THUMBSTICK Y-AXIS
	// sub_des_vel		= n.subscribe(BASE_IN_VEL_topic, 1, BASE_VEL__Callback);

	sub_posture_R = n.subscribe(R_IN_topic_track, 1, R_posture__Callback);
	sub_index_R = n.subscribe(R_IN_topic_index, 1, R_index_trigger__Callback);				// INDEX TRIGGER
	sub_hand_R = n.subscribe(R_IN_topic_hand, 1, R_hand_trigger__Callback);					// HAND TRIGGER
	sub_Botton_A_R = n.subscribe(R_IN_topic_button_A, 1, R_Botton_A__Callback);				// BUTTON 1
	sub_Botton_B_R = n.subscribe(R_IN_topic_button_B, 1, R_Botton_B__Callback);				// BUTTON 2
	sub_Thumbstick_x_R = n.subscribe(R_IN_topic_thumbstick_x, 1, R_Thumbstick_x__Callback); // THUMBSTICK X-AXIS
	sub_Thumbstick_y_R = n.subscribe(R_IN_topic_thumbstick_y, 1, R_Thumbstick_y__Callback); // THUMBSTICK Y-AXIS
	sub_stiff_R = n.subscribe(R_IN_topic_stiff, 1, R_IN_stiff__Callback);					// THUMBSTICK Y-AXIS

	sub_posture_H = n.subscribe(H_IN_topic_track, 1, H_posture__Callback);

	ros::Subscriber sub_cmd_vel_move_base = n.subscribe(CMD_VEL_IN_topic, 1, Cmd_vel__Callback);
	ros::Subscriber sub_cmd_vel_unity = n.subscribe(CMD_VEL_UNITY_topic, 1, Cmd_vel_Unity__Callback);

	// ------------------------------------------------------------------------------------- Published topics
	pub_posture_L		= n.advertise<geometry_msgs::Pose>(L_OUT_topic_track, 1);
	pub_index_L = n.advertise<std_msgs::Float64>(L_OUT_topic_index, 1);
	pub_hand_L = n.advertise<std_msgs::Float64>(L_OUT_topic_hand, 1);
	pub_Botton_X_L = n.advertise<std_msgs::Bool>(L_OUT_topic_button_X, 1);
	pub_Botton_Y_L = n.advertise<std_msgs::Bool>(L_OUT_topic_button_Y, 1);
	pub_Thumbstick_x_L = n.advertise<std_msgs::Float64>(L_OUT_topic_thumbstick_x, 1);
	pub_Thumbstick_y_L = n.advertise<std_msgs::Float64>(L_OUT_topic_thumbstick_y, 1);
	pub_stiff_L = n.advertise<std_msgs::Float64>(L_OUT_topic_stiff, 1);

	pub_posture_R = n.advertise<geometry_msgs::Pose>(R_OUT_topic_track, 1);
	pub_index_R = n.advertise<std_msgs::Float64>(R_OUT_topic_index, 1);
	pub_hand_R = n.advertise<std_msgs::Float64>(R_OUT_topic_hand, 1);
	pub_Botton_A_R = n.advertise<std_msgs::Bool>(R_OUT_topic_button_A, 1);
	pub_Botton_B_R = n.advertise<std_msgs::Bool>(R_OUT_topic_button_B, 1);
	pub_Thumbstick_x_R = n.advertise<std_msgs::Float64>(R_OUT_topic_thumbstick_x, 1);
	pub_Thumbstick_y_R = n.advertise<std_msgs::Float64>(R_OUT_topic_thumbstick_y, 1);
	pub_stiff_R = n.advertise<std_msgs::Float64>(R_OUT_topic_stiff, 1);

	pub_posture_H = n.advertise<geometry_msgs::Pose>(H_OUT_topic_track, 1);
	// pub_des_vel = n.advertise<geometry_msgs::Vector3>(BASE_OUT_VEL_topic, 1);
	pub_des_vel			= n.advertise<geometry_msgs::TwistStamped>(BASE_OUT_VEL_topic, 1);

	pub_alive = n.advertise<std_msgs::Bool>("robot_alive", 1);


	enable_auto_mode_service = n.advertiseService("/enable_auto_mode_service", enableAutoModeCallback);
	wait_help_from_pilot_service = n.advertiseService("/wait_help_from_pilot_service", waitPilotCallback);
    pub_auto_mode_status = n.advertise<std_msgs::Bool>(auto_mode_status_topic, 1);

	while (ros::ok())
	{

		//Se il pilota è uscito dal robot inavvertitamente e c'è ancora un comando di velocita
		//Aumenta la latenza dall'ultima callback del Bottone A e si setta cosi a zero le velocità
		// if (((ros::Time::now() - last_cmd_time) > max_cmd_latency) && !move_base_enabled )
		// {
		// 	enable_out = false;
		// 	vel_ang = 0;
		// 	vel_lin = 0;
		// 	com_state.data = false;
		// 	pub_Botton_A_R.publish(com_state);
		// }

		// if (((ros::Time::now() - last_cmd_vel_time) > max_cmd_latency) )
		// {
		// 	move_base_enabled = false;
		// }

		//Se il pilota è in play la variabile enable_out è 1 e cosi comando le velocità
		base_vel.header.stamp = ros::Time::now();
		base_vel.twist.linear.x = vel_lin;
		base_vel.twist.linear.y = 0;
		base_vel.twist.linear.z = 0;
		base_vel.twist.angular.x = 0;
		base_vel.twist.angular.y = 0;
		base_vel.twist.angular.z = vel_ang;

		pub_des_vel.publish(base_vel);


		alive_cnt++;
		if (alive_cnt == 10)
		{
			alive_msg.data = true;
			pub_alive.publish(alive_msg);
			alive_cnt = 0;
		}

		// --- cycle ---
		ros::spinOnce();
		loop_rate.sleep();
	}
}
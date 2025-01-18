#include "ros/ros.h"
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <vector>
#include <string>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <boost/scoped_ptr.hpp>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Pose.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <pseudo_inversion.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <alterego_msgs/UpperBodyState.h>

using namespace KDL;
using namespace std;

// #define arm_cubes_n 6
// #define n_joints_addon 2
// #define use_addon true

int arm_cubes_n;
int n_joints_addon;
int AlterEgoVersion;
bool use_addon;
KDL::JntArray arm_L_q;
KDL::JntArray arm_R_q;
double arm_L_q_addon_init;
double arm_R_q_addon_init;

bool arm_R_cb = false;
bool arm_L_cb = false;

double weight_L = 0.0;
double weight_R = 0.0;

/*---------------------------------------------------------------------*
 * ROBOT STATE CALLBACK                                                 *
 *                                                                      *
 *----------------------------------------------------------------------*/
void robot_state__Callback(const alterego_msgs::UpperBodyState::ConstPtr &msg)
{

	weight_L = (double)msg->left_weight;
	weight_R = (double)msg->right_weight;

	// get left arm data
	for (int i = 0; i < arm_cubes_n; i++)
	{
		arm_L_q(i) = (double)msg->left_meas_arm_shaft[i];
		arm_R_q(i) = (double)msg->right_meas_arm_shaft[i];
	}

	if (!arm_L_cb)
	{
		arm_R_q_addon_init = arm_R_q(1) / 2;
		arm_L_q_addon_init = arm_L_q(1) / 2;
	}

	if (use_addon)
	{
		arm_R_q(1) = arm_R_q(1) / 2 - arm_R_q_addon_init;
		arm_L_q(1) = arm_L_q(1) / 2 - arm_L_q_addon_init;
	}

	arm_R_cb = true;
	arm_L_cb = true;
}

/*---------------------------------------------------------------------*
 * LEFT ARM CALLBACK                                                    *
 *                                                                      *
 *----------------------------------------------------------------------*/
void arm_L__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{

	// arm_L_q(0) = -(double)msg->data[0];

	for (int i = 0; i < arm_cubes_n; i++)
		arm_L_q(i) = (double)msg->data[i];

	if (!arm_L_cb)
	{
		arm_L_q_addon_init = arm_L_q(1) / 2;
	}

	if (use_addon)
	{
		arm_L_q(1) = arm_L_q(1) / 2 - arm_L_q_addon_init;
	}

	arm_L_cb = true;
}

/*---------------------------------------------------------------------*
 * WEIGHT LEFT ARM CALLBACK                                                    *
 *                                                                      *
 *----------------------------------------------------------------------*/
void weight_L__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
	weight_L = msg->data[0];
}

/*---------------------------------------------------------------------*
 * RIGHT ARM CALLBACK                                                   *
 *                                                                      *
 *----------------------------------------------------------------------*/
void arm_R__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
	// arm_R_q(0) = -(double)msg->data[0];

	for (int i = 0; i < arm_cubes_n; i++)
		arm_R_q(i) = (double)msg->data[i];

	if (!arm_R_cb)
	{
		arm_R_q_addon_init = arm_R_q(1) / 2;
	}

	if (use_addon)
	{
		arm_R_q(1) = arm_R_q(1) / 2 - arm_R_q_addon_init;
	}

	arm_R_cb = true;
}

// void hand_R__Callback(const std_msgs::Float64::ConstPtr& msg){
// 	hand_R = msg->data;
// }

/*---------------------------------------------------------------------*
 * WEIGHT RIGHT ARM CALLBACK                                                   *
 *                                                                      *
 *----------------------------------------------------------------------*/
void weight_R__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
	weight_R = msg->data[0];
}

/*---------------------------------------------------------------------*
 * MAIN                                                                 *
 *                                                                      *
 *----------------------------------------------------------------------*/
int main(int argc, char **argv)
{

	// ------------------------------------------------------------------------------------- Init node
	ros::init(argc, argv, "pitch_correction");
	ros::NodeHandle n;
	string ns;
	std::string robot_name;
	ns = ros::this_node::getNamespace();
	robot_name = std::getenv("ROBOT_NAME");
	n.getParam("arm_cubes_n", arm_cubes_n);

	int k(0);
	int run_freq;

	double body_m(27.3);
	double cube_m(0.5);
	double cube_wrist_m(0.713);
	double cube_m_shoulder(0.8);
	double cube_m_addon(0.65);
	double hand_m(0.5);
	double wheel_m(1);
	double tot_mass;
	double standing_pitch_offset;
	double offset_phi_old;
	double alpha = 0.95;

	string arm_L_TN;
	string arm_R_TN;
	// string 					hand_L_TN;
	// string 					hand_R_TN;
	string weight_L_TN;
	string weight_R_TN;

	string pitch_corr_TN;

	std::vector<double> DH_Xtr_L;
	std::vector<double> DH_Xrot_L;
	std::vector<double> DH_Ztr_L;
	std::vector<double> DH_Zrot_L;
	std::vector<double> DH_Xtr_R;
	std::vector<double> DH_Xrot_R;
	std::vector<double> DH_Ztr_R;
	std::vector<double> DH_Zrot_R;
	std::vector<double> T_b2t;
	std::vector<double> T_t2s_L;
	std::vector<double> T_t2s_R;
	std::vector<double> body_cm_vec;

	KDL::Frame cart_R, cart_L;

	Eigen::MatrixXd arm_L_CM(3, arm_cubes_n);
	Eigen::MatrixXd arm_R_CM(3, arm_cubes_n);

	Eigen::Vector3d tot_cm;
	Eigen::Vector3d body_cm;

	ros::Subscriber sub_arm_L;
	ros::Subscriber sub_robot_state;
	ros::Subscriber sub_arm_R;
	// ros::Subscriber 		sub_hand_L;
	// ros::Subscriber 		sub_hand_R;
	ros::Subscriber sub_weight_L;
	ros::Subscriber sub_weight_R;

	ros::Publisher pub_offset_phi;

	KDL::Chain chain_L;
	KDL::Chain chain_R;
	boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_L, jnt_to_pose_solver_R;
	boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_L, jnt_to_jac_solver_R;
	boost::scoped_ptr<KDL::ChainDynParam> dyn_solver_L, dyn_solver_R;

	std_msgs::Float32 offset_phi_pub;

	// ------------------------------------------------------------------------------------- Check/save args
	n.getParam("body_mass", body_m);
	n.getParam("cube_mass", cube_m);
	n.getParam("cube_wrist_mass", cube_wrist_m);
	n.getParam("cube_addon_mass", cube_m_addon);
	n.getParam("cube_m_shoulder", cube_m_shoulder);
	n.getParam("hand_mass", hand_m);
	n.getParam("wheel_mass", wheel_m);
	n.getParam("body_cm", body_cm_vec);
	n.getParam("armL_topic_name", arm_L_TN);
	n.getParam("armR_topic_name", arm_R_TN);
	// n.getParam("handL_topic_name", hand_L_TN);
	// n.getParam("handR_topic_name", hand_R_TN);
	n.getParam("weightL_topic_name", weight_L_TN);
	n.getParam("weightR_topic_name", weight_R_TN);
	n.getParam("pitch_corr_topic_name", pitch_corr_TN);
	n.getParam("DH_Xtr_L", DH_Xtr_L);
	n.getParam("DH_Xrot_L", DH_Xrot_L);
	n.getParam("DH_Ztr_L", DH_Ztr_L);
	n.getParam("DH_Zrot_L", DH_Zrot_L);
	n.getParam("DH_Xtr_R", DH_Xtr_R);
	n.getParam("DH_Xrot_R", DH_Xrot_R);
	n.getParam("DH_Ztr_R", DH_Ztr_R);
	n.getParam("DH_Zrot_R", DH_Zrot_R);
	n.getParam("T_b2t", T_b2t);
	n.getParam("T_t2s_L", T_t2s_L);
	n.getParam("T_t2s_R", T_t2s_R);
	n.getParam("/" + robot_name + "/AlterEgoVersion", AlterEgoVersion);
	n.getParam("use_addon", use_addon);
	bool sim = false;
	n.param("/" + robot_name + "/SimulatedEgo", sim, false);

	n.getParam("/" + robot_name +"/wheels/standing_pitch_offset", standing_pitch_offset);
	ROS_INFO("standing_pitch_offset:\t %.3f",standing_pitch_offset);

	offset_phi_old = standing_pitch_offset;

	run_freq = 50;
	n.getParam("pitch_correction_frequency", run_freq); // Override if configured
	ros::Rate loop_rate(run_freq);

	// ------------------------------------------------------------------------------------- Subscribe to topics
	sub_robot_state = n.subscribe("alterego_state/upperbody", 1, robot_state__Callback);
	// sub_arm_L		= n.subscribe(arm_L_TN, 1, arm_L__Callback);
	// sub_arm_R		= n.subscribe(arm_R_TN, 1, arm_R__Callback);
	// sub_weight_L	= n.subscribe(weight_L_TN, 1, weight_L__Callback);
	// sub_weight_R	= n.subscribe(weight_R_TN, 1, weight_R__Callback);

	// ------------------------------------------------------------------------------------- Published topics
	pub_offset_phi = n.advertise<std_msgs::Float32>(pitch_corr_TN, 1);

	// ------------------------------------------------------------------------------------- Init chains
	chain_L.addSegment(Segment(Joint(Joint::None), Frame(Rotation(T_b2t[0], T_b2t[4], T_b2t[8], T_b2t[1], T_b2t[5], T_b2t[9], T_b2t[2], T_b2t[6], T_b2t[10]),
														 Vector(T_b2t[3], T_b2t[7], T_b2t[11]))));
	chain_L.addSegment(Segment(Joint(Joint::None), Frame(Rotation(T_t2s_L[0], T_t2s_L[4], T_t2s_L[8], T_t2s_L[1], T_t2s_L[5], T_t2s_L[9], T_t2s_L[2], T_t2s_L[6], T_t2s_L[10]),
														 Vector(T_t2s_L[3], T_t2s_L[7], T_t2s_L[11]))));
	chain_L.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr_L[0], DH_Xrot_L[0], DH_Ztr_L[0], DH_Zrot_L[0])));
	chain_L.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr_L[1], DH_Xrot_L[1], DH_Ztr_L[1], DH_Zrot_L[1])));
	chain_L.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr_L[2], DH_Xrot_L[2], DH_Ztr_L[2], DH_Zrot_L[2])));
	chain_L.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr_L[3], DH_Xrot_L[3], DH_Ztr_L[3], DH_Zrot_L[3])));
	chain_L.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr_L[4], DH_Xrot_L[4], DH_Ztr_L[4], DH_Zrot_L[4])));
	if (AlterEgoVersion == 3)
		chain_L.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr_L[5], DH_Xrot_L[5], DH_Ztr_L[5], DH_Zrot_L[5])));

	chain_R.addSegment(Segment(Joint(Joint::None), Frame(Rotation(T_b2t[0], T_b2t[4], T_b2t[8], T_b2t[1], T_b2t[5], T_b2t[9], T_b2t[2], T_b2t[6], T_b2t[10]),
														 Vector(T_b2t[3], T_b2t[7], T_b2t[11]))));
	chain_R.addSegment(Segment(Joint(Joint::None), Frame(Rotation(T_t2s_R[0], T_t2s_R[4], T_t2s_R[8], T_t2s_R[1], T_t2s_R[5], T_t2s_R[9], T_t2s_R[2], T_t2s_R[6], T_t2s_R[10]),
														 Vector(T_t2s_R[3], T_t2s_R[7], T_t2s_R[11]))));
	chain_R.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr_R[0], DH_Xrot_R[0], DH_Ztr_R[0], DH_Zrot_R[0])));
	chain_R.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr_R[1], DH_Xrot_R[1], DH_Ztr_R[1], DH_Zrot_R[1])));
	chain_R.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr_R[2], DH_Xrot_R[2], DH_Ztr_R[2], DH_Zrot_R[2])));
	chain_R.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr_R[3], DH_Xrot_R[3], DH_Ztr_R[3], DH_Zrot_R[3])));
	chain_R.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr_R[4], DH_Xrot_R[4], DH_Ztr_R[4], DH_Zrot_R[4])));
	if (AlterEgoVersion == 3)
		chain_R.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr_R[5], DH_Xrot_R[5], DH_Ztr_R[5], DH_Zrot_R[5])));

	arm_R_q.resize(chain_R.getNrOfJoints());
	arm_L_q.resize(chain_L.getNrOfJoints());
	KDL::SetToZero(arm_R_q);
	KDL::SetToZero(arm_L_q);

	jnt_to_pose_solver_L.reset(new KDL::ChainFkSolverPos_recursive(chain_L));
	jnt_to_pose_solver_R.reset(new KDL::ChainFkSolverPos_recursive(chain_R));

	arm_L_q_addon_init = 0;
	arm_R_q_addon_init = 0;

	// cout << "standing_pitch_offset: " << standing_pitch_offset << endl;
	// cout << "arm L topic: " << arm_L_TN << endl;
	// cout << "arm R topic: " << arm_R_TN << endl;
	// cout << "weight L topic: " << weight_L_TN << endl;
	// cout << "weight R topic: " << weight_R_TN << endl;
	// cout << "[AlterEgoVersion]: " << AlterEgoVersion << endl;
	// cout << "[Use Addon]: " << use_addon << endl;
	// cout << "[left arm] Number of joints:" << chain_L.getNrOfJoints() << endl;
	// cout << "[right arm] Number of joints:" << chain_R.getNrOfJoints() << endl;

	if (!use_addon)
	{
		tot_mass = body_m + 2 * (arm_cubes_n - 1) * cube_m + 2 * hand_m;
	}
	else
	{
		tot_mass = body_m + 2 * (arm_cubes_n - 3) * cube_m + 2 * cube_m_addon + 2 * cube_wrist_m + 2 * hand_m;
	}

	body_cm << body_cm_vec[0], body_cm_vec[1], body_cm_vec[2];

	offset_phi_pub.data = standing_pitch_offset;

	while (!(arm_R_cb && arm_L_cb) && ros::ok())
	{
		ros::spinOnce();
	}
	// ------------------------------------------------------------------------------------- MAIN LOOP

	while (ros::ok())
	{
		k = 0;

		for (int i = 1; i < arm_cubes_n + 1; i++)
		{
			jnt_to_pose_solver_R->JntToCart(arm_R_q, cart_R, i + 2);
			jnt_to_pose_solver_L->JntToCart(arm_L_q, cart_L, i + 2);

			arm_L_CM(0, k) = cart_L.p(0);
			arm_L_CM(1, k) = cart_L.p(1);
			arm_L_CM(2, k) = cart_L.p(2);

			arm_R_CM(0, k) = cart_R.p(0);
			arm_R_CM(1, k) = cart_R.p(1);
			arm_R_CM(2, k) = cart_R.p(2);

			k++;
		}

		if (!use_addon)
		{
			tot_cm = (body_cm * body_m + arm_L_CM.block<3, 1>(0, 0) * cube_m + arm_L_CM.block<3, 1>(0, 1) * cube_m + arm_L_CM.block<3, 1>(0, 2) * cube_m + arm_L_CM.block<3, 1>(0, 3) * cube_m + arm_L_CM.block<3, 1>(0, 4) * hand_m + arm_R_CM.block<3, 1>(0, 0) * cube_m + arm_R_CM.block<3, 1>(0, 1) * cube_m + arm_R_CM.block<3, 1>(0, 2) * cube_m + arm_R_CM.block<3, 1>(0, 3) * cube_m + arm_R_CM.block<3, 1>(0, 4) * hand_m) / (tot_mass);
		}
		else
		{

			tot_cm = (body_cm * body_m + arm_L_CM.block<3, 1>(0, 0) * cube_m_addon + arm_L_CM.block<3, 1>(0, 1) * cube_m + arm_L_CM.block<3, 1>(0, 2) * (cube_m + 0.1) + arm_L_CM.block<3, 1>(0, 3) * cube_m + arm_L_CM.block<3, 1>(0, 4) * cube_wrist_m + arm_L_CM.block<3, 1>(0, 5) * hand_m //(hand_m+l_w_current)
					  + arm_R_CM.block<3, 1>(0, 0) * cube_m_addon + arm_R_CM.block<3, 1>(0, 1) * cube_m + arm_R_CM.block<3, 1>(0, 2) * (cube_m + 0.1) + arm_R_CM.block<3, 1>(0, 3) * cube_m + arm_R_CM.block<3, 1>(0, 4) * cube_wrist_m + arm_R_CM.block<3, 1>(0, 5) * hand_m				   //(hand_m+r_w_current)
					  ) /
					 (tot_mass + 0.2); //(tot_mass + 0.1 +l_w_current+r_w_current);
		}

		offset_phi_pub.data = (1 - alpha) * offset_phi_old + (alpha) * (atan2(tot_cm(0), tot_cm(2)) + standing_pitch_offset);

		offset_phi_old = offset_phi_pub.data;

		pub_offset_phi.publish(offset_phi_pub);

		// --- cycle ---
		ros::spinOnce();
		loop_rate.sleep();
	}
}

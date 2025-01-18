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
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

using namespace KDL;
using namespace std;

#define MAX_STIFF 0.3

Frame ref_frame;
Eigen::VectorXd ref_twist(6);
Eigen::VectorXd ref_stiffness(6);
int arm_side;
std::string ns;
Eigen::Matrix4d T_b_0;
Eigen::Matrix3d R_o_b;
ros::Time cmd_time, cmd_time_old;
KDL::JntArray meas_neck_shaft;
bool neck_right;
bool neck_left;
/*---------------------------------------------------------------------*
 * POSTURE CALLBACK                                                     *
 *                                                                      *
 *----------------------------------------------------------------------*/
void posture__Callback(const geometry_msgs::Pose::ConstPtr& msg)
{
	Eigen::Quaterniond	ref_quat;
	static Eigen::Quaterniond	old_quat;
	double sign_check;
	Eigen::Vector3d		r_p;
	Eigen::Matrix3d		r_M;
	static tf::TransformBroadcaster br;
  	tf::Transform transform;
	  
	ref_quat.x() = msg->orientation.x;
	ref_quat.y() = msg->orientation.y;
	ref_quat.z() = msg->orientation.z;
	ref_quat.w() = msg->orientation.w;

	sign_check = ref_quat.w() * old_quat.w() + ref_quat.x() * old_quat.x() + ref_quat.y() * old_quat.y() + ref_quat.z() * old_quat.z();
	if(sign_check < 0.0){
		ref_quat.w() = -ref_quat.w(); 
		ref_quat.vec() = -ref_quat.vec(); 
	}
	old_quat = ref_quat;
	//dubbio ref_quat
	//ref_quat = (R_o_b*ref_quat*R_o_b.transpose())*T_b_0.block<3,3>(0,0).transpose();
	//r_p = R_o_b*r_p+T_b_0.block<3,1>(0,3);
	r_M = ref_quat;


	ref_frame = Frame( Rotation(r_M(0,0), r_M(0,1), r_M(0,2), r_M(1,0), r_M(1,1), r_M(1,2), r_M(2,0), r_M(2,1), r_M(2,2)), Vector(r_p(0), r_p(1), r_p(2)));

	cmd_time_old = cmd_time;
	cmd_time = ros::Time::now();
}

/*---------------------------------------------------------------------*
 * STIFFNESS CALLBACK                                                   *
 *                                                                      *
 *----------------------------------------------------------------------*/
void stiffness__Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
	for (int i = 0; i < 2; i++)
		ref_stiffness(i) = msg->data[i];
}


/*---------------------------------------------------------------------*
 * MEAS RIGHT HEAD SHAFT CALLBACK                                                     *
 *                                                                      *
 *----------------------------------------------------------------------*/
void right_meas_neck__Callback(const std_msgs::Float64::ConstPtr &msg)
{

    meas_neck_shaft(1) = msg->data;

    neck_right = true;
}
/*---------------------------------------------------------------------*
 * MEAS LEFT HEAD SHAFT CALLBACK                                                     *
 *                                                                      *
 *----------------------------------------------------------------------*/
void left_meas_neck__Callback(const std_msgs::Float64::ConstPtr &msg)
{

	meas_neck_shaft(0) = msg->data;


    neck_left = true;
}
/*---------------------------------------------------------------------*
 * MAIN                                                                 *
 *                                                                      *
 *----------------------------------------------------------------------*/
int main(int argc, char **argv)
{
	// ------------------------------------------------------------------------------------- Init Var
	// --- node param ---
	std::string chain_topic;
	std::string pose_ref_topic;
	std::string stiff_ref_topic;
	std::string phantom_arm_topic;
	std::string ref_head_Lc_topic;
	std::string ref_head_Rc_topic;
	std::string cubes_shaft_topic;
	
	int run_freq;
	ros::Subscriber sub_posture, sub_left_meas_neck, sub_right_meas_neck;
	ros::Subscriber sub_stiffness;
	ros::Publisher pub_cart_ref;
	ros::Publisher pub_phantom;
	ros::Publisher pub_head_Lc_eq;
	ros::Publisher pub_head_Rc_eq;
	geometry_msgs::Pose cart_ref_msg;
	sensor_msgs::JointState phantom_msg;
	char buffer[50];

	std_msgs::Float64 head_ref_Lc_msg;
	std_msgs::Float64 head_ref_Rc_msg;

	// --- kinematics ---
	std::vector<double> DH;
	std::vector<double> DH_Xtr;
	std::vector<double> DH_Xrot;
	std::vector<double> DH_Ztr;
	std::vector<double> DH_Zrot;
	std::vector<double> T_t2s;
	std::vector<double> T_o2t;
	std::vector<double> R_o2b;
	std::vector<double> q_min;
	std::vector<double> q_max;
	std::vector<int> qbmove_tf_ids;
	int softhand_tf_id;

	KDL::Chain chain;
	boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver;
	boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
	boost::scoped_ptr<KDL::ChainFkSolverPos> shaft_to_pose_solver;

	Eigen::VectorXd qMax_left(2);
	Eigen::VectorXd qMin_left(2);
	Eigen::VectorXd qMax_right(2);
	Eigen::VectorXd qMin_right(2);

	KDL::JntArray q;
	KDL::Jacobian JA_kdl;
	KDL::Frame act_frame, shaft_frame;
	KDL::Twist err_twist;
	Eigen::VectorXd err_post(6);
	Eigen::VectorXd x_post(6);
	Eigen::MatrixXd K_v(6, 6);
	K_v << 50 * Eigen::MatrixXd::Identity(6, 6);
	Eigen::MatrixXd JA(6, 2);
	Eigen::MatrixXd JA_pinv(2, 6);
	Eigen::MatrixXd K_d(6, 6);
	K_d << 0.1 * Eigen::MatrixXd::Identity(6, 6);
	double k_0;
	k_0 = 2;

	Eigen::VectorXd eq_dot(2);
	Eigen::VectorXd eq(2);
	Eigen::VectorXd eq_f(2);

	double arm_l;
	Eigen::Quaterniond act_quat, act_shaft_quat;

	unsigned long int msg_seq(0);
	double stiffn;

	ros::Duration max_cmd_time = ros::Duration(10);
	ros::Duration filt_time = ros::Duration(5);
	ros::Duration max_cmd_latency = ros::Duration(1);
	ros::Time start_f_time;
	int act_bp(1);
	double alpha(1);

	// ------------------------------------------------------------------------------------- Init node
	ros::init(argc, argv, "head_manager");
	ros::NodeHandle n;
	ns = ros::this_node::getNamespace();
	ns = ns.substr(1, ns.length() - 1);
	std::string robot_name = std::getenv("ROBOT_NAME");
	// --- Rviz ---
	tf::TransformBroadcaster ik_br, ik_shaft;
	tf::Transform ik_tf,ik_tf_shaft;

	// ------------------------------------------------------------------------------------- Check/save args
	n.getParam("DH_Xtr", DH_Xtr);
	n.getParam("DH_Xrot", DH_Xrot);
	n.getParam("DH_Ztr", DH_Ztr);
	n.getParam("DH_Zrot", DH_Zrot);
	n.getParam("T_t2s", T_t2s);
	n.getParam("T_o2t", T_o2t);
	n.getParam("R_o2b", R_o2b);
	n.getParam("q_min", q_min);
	n.getParam("q_max", q_max);
	n.getParam("qbmove_tf_ids", qbmove_tf_ids);
	n.getParam("pose_ref_topic", pose_ref_topic);
	n.getParam("stiff_ref_topic", stiff_ref_topic);
	n.getParam("phantom_arm_topic", phantom_arm_topic);
	n.getParam("ref_head_Lc_topic", ref_head_Lc_topic);
	n.getParam("ref_head_Rc_topic", ref_head_Rc_topic);
	n.getParam("active_back_pos", act_bp);
	n.getParam("cubes_shaft_topic", cubes_shaft_topic);

	run_freq = 50;
	n.getParam("head_frequency", run_freq); // Override if configured
	ros::Rate loop_rate(run_freq);

	// ------------------------------------------------------------------------------------- Kinematics
	chain.addSegment(Segment(Joint(Joint::None),Frame(Rotation(T_t2s[0],T_t2s[4],T_t2s[8],T_t2s[1],T_t2s[5],T_t2s[9],T_t2s[2],T_t2s[6],T_t2s[10]),
		Vector(T_t2s[3],T_t2s[7],T_t2s[11]))));
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr[0], DH_Xrot[0], DH_Ztr[0], DH_Zrot[0])));
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr[1], DH_Xrot[1], DH_Ztr[1], DH_Zrot[1])));

	// T_b_0 << T_o2t[0], T_o2t[1], T_o2t[2], T_o2t[3],
	// 	T_o2t[4], T_o2t[5], T_o2t[6], T_o2t[7],
	// 	T_o2t[8], T_o2t[9], T_o2t[10], T_o2t[11],
	// 	T_o2t[12], T_o2t[13], T_o2t[14], T_o2t[15];

	// R_o_b << R_o2b[0], R_o2b[1], R_o2b[2],
	// 	R_o2b[3], R_o2b[4], R_o2b[5],
	// 	R_o2b[6], R_o2b[7], R_o2b[8];

	jnt_to_pose_solver.reset(new KDL::ChainFkSolverPos_recursive(chain));
	jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(chain));
    shaft_to_pose_solver.reset(new KDL::ChainFkSolverPos_recursive(chain));
    meas_neck_shaft.resize(chain.getNrOfJoints());

	q.resize(chain.getNrOfJoints());
	JA_kdl.resize(chain.getNrOfJoints());
	KDL::SetToZero(q);

	jnt_to_pose_solver->JntToCart(q, ref_frame);
	jnt_to_jac_solver->JntToJac(q, JA_kdl);
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 2; j++)
			if (fabs(JA_kdl(i, j)) > 0.000001)
				JA(i, j) = JA_kdl(i, j);
			else
				JA(i, j) = 0;
	}

	JA_pinv = JA.transpose() * ((JA * JA.transpose() + K_d).inverse());

	ref_twist << 0, 0, 0, 0, 0, 0;

	eq << 0, 0;
	eq_f << 0, 0;

	// ------------------------------------------------------------------------------------- Subscribe to topics
	sub_posture = n.subscribe(pose_ref_topic, 1, posture__Callback);
	sub_left_meas_neck = n.subscribe("/"+robot_name+"/left/"+cubes_shaft_topic, 1, left_meas_neck__Callback);
	sub_right_meas_neck = n.subscribe("/"+robot_name+"/right/"+cubes_shaft_topic, 1, right_meas_neck__Callback);
	sub_stiffness = n.subscribe(stiff_ref_topic, 1, stiffness__Callback);

	// ------------------------------------------------------------------------------------- Published topics
	// ros::Publisher    	pub_inv_kin		= n.advertise<qb_interface::cubeEq_Preset>(target_chain + "_eq_pre", 1000);
	// pub_cart_ref	= n.advertise<geometry_msgs::Pose>(chain_topic, 1);
	pub_head_Lc_eq = n.advertise<std_msgs::Float64>(ref_head_Lc_topic, 1);
	pub_head_Rc_eq = n.advertise<std_msgs::Float64>(ref_head_Rc_topic, 1);
	// pub_phantom = n.advertise<sensor_msgs::JointState>(phantom_arm_topic, 1);

	stiffn = MAX_STIFF * 0.9;

	cmd_time = ros::Time::now();
	cmd_time_old = ros::Time::now();
	neck_right = false;
	neck_left = false;
	// ------------------------------------------------------------------------------------- MAIN LOOP
	while (ros::ok())
	{
		// --- Inverse Kinematics ---
		jnt_to_pose_solver->JntToCart(q, act_frame);
		jnt_to_jac_solver->JntToJac(q, JA_kdl);
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 2; j++)
				if (fabs(JA_kdl(i, j)) > 0.000001)
					JA(i, j) = JA_kdl(i, j);
				else
					JA(i, j) = 0;
		}

		err_twist = KDL::diff(act_frame, ref_frame);

		err_post << err_twist[0], err_twist[1], err_twist[2], err_twist[3], err_twist[4], err_twist[5];
		err_post = K_v * err_post;

		JA_pinv = JA.transpose() * ((JA * JA.transpose() + K_d).inverse());

		eq_dot = JA_pinv * err_post;

		eq += eq_dot / run_freq;

		// Back position
		if (act_bp == 1 && (ros::Time::now() - cmd_time > max_cmd_time))
		{
			eq << 0, 0;
			start_f_time = ros::Time::now();
			// cout << "1  " << eq_f.transpose() << endl;
			alpha = 1;
		}

		// Check latency between msgs
		// if (cmd_time-cmd_time_old > max_cmd_latency){
		// 	start_f_time = ros::Time::now();
		// 	// cout << "2  " << cmd_time << endl;
		// 	// cout << "2  " << cmd_time_old << endl;
		// }

		// Filtering position
		if (ros::Time::now() - start_f_time < filt_time)
		{
			alpha -= 1 / (filt_time.toSec() * run_freq);
			if (alpha < 0)
				alpha = 0;
			eq_f = alpha * eq_f + (1 - alpha) * eq;
			// cout << "3  " << eq_f.transpose() << endl;
		}
		else
			eq_f = eq;

		for (int i = 0; i < 2; i++)
		{
			if (eq_f(i) > q_max[i])
			{
				eq_f(i) = q_max[i];
			}
			if (eq_f(i) < q_min[i])
			{
				eq_f(i) = q_min[i];
			}

			q(i) = eq_f(i);
			eq(i) = eq_f(i);
		}

		// --- publish all messages ---
		// Rviz TF:
		ref_frame.M.GetQuaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w());
		ik_tf.setOrigin(tf::Vector3(act_frame.p[0], act_frame.p[1], act_frame.p[2]));
		ik_tf.setRotation(tf::Quaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w()));
		ik_br.sendTransform(tf::StampedTransform(ik_tf, ros::Time::now(), "torso", "head_ref"));

		shaft_to_pose_solver->JntToCart(meas_neck_shaft, shaft_frame);
		shaft_frame.M.GetQuaternion(act_shaft_quat.x(), act_shaft_quat.y(), act_shaft_quat.z(), act_shaft_quat.w());
		ik_tf_shaft.setOrigin(tf::Vector3(shaft_frame.p[0], shaft_frame.p[1], shaft_frame.p[2]));
		ik_tf_shaft.setRotation(tf::Quaternion(act_shaft_quat.x(), act_shaft_quat.y(), act_shaft_quat.z(), act_shaft_quat.w()));
		ik_shaft.sendTransform(tf::StampedTransform(ik_tf_shaft, ros::Time::now(), "torso", "head_curr"));

		

		if (q(1) < q_min[1])
			q(1) = q_min[1];
		if (q(1) > q_max[1])
			q(1) = q_max[1];

		head_ref_Rc_msg.data = q(1);
		head_ref_Lc_msg.data = q(0);

		// Rviz model:
		phantom_msg.name.resize(chain.getNrOfJoints() + 1);
		phantom_msg.position.resize(chain.getNrOfJoints() + 1);
		for (int i = 0; i < 2; i++)
		{
			sprintf(buffer, "phantom_cube%d_shaft_joint", qbmove_tf_ids[i]);
			phantom_msg.name[i] = buffer;
			phantom_msg.position[i] = q(i);
		}

		pub_head_Lc_eq.publish(head_ref_Lc_msg);
		pub_head_Rc_eq.publish(head_ref_Rc_msg);
		// pub_phantom.publish(phantom_msg);

		// --- cycle ---
		ros::spinOnce();
		loop_rate.sleep();
	}
}
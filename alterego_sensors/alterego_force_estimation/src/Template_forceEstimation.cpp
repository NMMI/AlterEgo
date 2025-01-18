
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

//AndCav 
#include "optoforce_sensor/opto.h"
#include "geometry_msgs/WrenchStamped.h"
#include <unistd.h>
#include <string.h>

using namespace KDL;
using namespace std;

#define n_joints_subchain 3
#define n_joints_addon 2
#define use_addon true

KDL::JntArray		arm_R_q_r2s(n_joints_subchain);
KDL::JntArray		arm_R_q_s2h(n_joints_subchain);
KDL::JntArray		arm_L_q_r2s(n_joints_subchain);
KDL::JntArray		arm_L_q_s2h(n_joints_subchain);
Eigen::VectorXd     arm_L_q_addon_init(n_joints_addon);
Eigen::VectorXd     arm_R_q_addon_init(n_joints_addon);

bool				arm_R_cb = false;
bool				arm_L_cb = false;

double 				hand_L = 0.0;
double 				hand_R = 0.0;
double 				weight_L = 0.0;
double 				weight_R = 0.0;

/*---------------------------------------------------------------------*
* LEFT ARM CALLBACK                                                    *
*                                                                      *
*----------------------------------------------------------------------*/
void arm_L__Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	for (int i=0; i<n_joints_subchain; i++)
	{
		arm_L_q_r2s(i) = (double)msg->data[i];
		arm_L_q_s2h(i) = (double)msg->data[i+3];
		
	}

	if(use_addon)
	{
		for(int i=0; i<n_joints_addon; i++)
			arm_L_q_r2s(i) = arm_L_q_r2s(i)/2 - arm_L_q_addon_init(i);
	}
	
	arm_L_cb = true;
}

void hand_L__Callback(const std_msgs::Float64::ConstPtr& msg){
	hand_L = msg->data;
}

/*---------------------------------------------------------------------*
* RIGHT ARM CALLBACK                                                   *
*                                                                      *
*----------------------------------------------------------------------*/
void arm_R__Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	for (int i=0; i<n_joints_subchain; i++)
	{
		arm_R_q_r2s(i) = (double)msg->data[i];
		arm_R_q_s2h(i) = (double)msg->data[i+3];
		
	}

	if(use_addon)
	{
		for(int i=0; i<n_joints_addon; i++)
			arm_R_q_r2s(i) = arm_R_q_r2s(i)/2 - arm_R_q_addon_init(i);
	}

	arm_R_cb = true;
}

void hand_R__Callback(const std_msgs::Float64::ConstPtr& msg){
	hand_R = msg->data;
}

/*---------------------------------------------------------------------*
* MAIN                                                                 *
*                                                                      *
*----------------------------------------------------------------------*/
int main(int argc, char **argv){
	
	int 					k(0);
	int 					run_freq;



	string 					arm_L_TN;
	string 					arm_R_TN;
	string 					hand_L_TN;
	string 					hand_R_TN;
	string 					weight_L_TN;
	string 					weight_R_TN;
	
	string 					ns;


	std::vector<double>		DH_Xtr_L;
	std::vector<double>		DH_Xrot_L;
	std::vector<double>		DH_Ztr_L;
	std::vector<double>		DH_Zrot_L;
	std::vector<double>		DH_Xtr_R;
	std::vector<double>		DH_Xrot_R;
	std::vector<double>		DH_Ztr_R;
	std::vector<double>		DH_Zrot_R;
	std::vector<double>		T_t2s_L;
	std::vector<double>		T_t2s_R;
	std::vector<double>		T_init_R;
	std::vector<double>		body_cm_vec;

  	KDL::Frame     			cart_R, cart_L;

	Eigen::Quaterniond	act_quat;


	ros::Subscriber 		sub_arm_L;
	ros::Subscriber 		sub_arm_R;
	ros::Subscriber 		sub_hand_L;
	ros::Subscriber 		sub_hand_R;


	KDL::Chain 									chain_L_root2s, chain_L_s2h;
	KDL::Chain 									chain_R_root2s, chain_R_s2h;
	boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_L_root2s, jnt_to_pose_solver_R_root2s, jnt_to_pose_solver_L_s2h, jnt_to_pose_solver_R_s2h;
	boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_L_s2h, jnt_to_jac_solver_R_s2h;
	boost::scoped_ptr<KDL::ChainDynParam> 		dyn_solver_L, dyn_solver_R;

  	double 				cube_m(0.5);
	double              cube_wrist_m(0.713);
	double 				cube_m_addon(0.65);
	double 				hand_m(0.5);



	// ------------------------------------------------------------------------------------- Init node
  	ros::init(argc, argv, "force_estimation");
  	ros::NodeHandle n;
	
	ns = ros::this_node::getNamespace();
	ns = ns.substr(1,ns.length()-1);



	// AndCav 
	 OptoDAQ daq;
	OptoPorts ports;
	msSleep(2500); // We wait some ms to be sure about OptoPorts enumerated PortList
	ros::Rate rate(spin_rate);
 
	int iSpeed = 30;         //impostare Rate a 30Hz                                                                                                      // Speed in Hz
	int iFilter = 15;                                                                                                           // Filter in Hz
	std::vector<double> cn_values = {450.47, 492.7, 65.175, 457.43, 466.23, 71.88, 446.54, 459.19, 69.98, 440.34, 469.9, 64.65}; // sensors 84 85 86 and 87
	SensorConfig sensorConfig;
	sensorConfig.setSpeed(iSpeed);
	sensorConfig.setFilter(iFilter);
	u_int8_t sens_params_set = 1;

	OPort *portlist = ports.listPorts(true);
	daq.open(portlist[0]);
	daq.zeroAll();
	bool bConfig = false;
	bConfig = daq.sendConfig(sensorConfig);
	sleep(1.0);
	 while ((bConfig == false) && ros::ok())
	{
		ROS_ERROR_STREAM("Could not set config");
		bConfig = daq.sendConfig(sensorConfig);
		sleep(1.0);
		//         ros::shutdown();
		//         return 0;
	}

	double fx_gain, fy_gain, fz_gain; // sensitivity gain in counts/N

	int speed, filter;

	OptoPackage *pack3D = 0;
	int size = daq.readAll(pack3D, false);
	int sensSize = daq.getSensorSize();
	std::cout << "STARTING \n"
	      << sensSize;
	double temp_x, temp_y, temp_z, temp_abs;

	daq.zeroAll();

	// ------------------------------------------------------------------------------------- Check/save args
	n.getParam("armL_topic_name", arm_L_TN);
	n.getParam("armR_topic_name", arm_R_TN);
	n.getParam("handL_topic_name", hand_L_TN);
	n.getParam("handR_topic_name", hand_R_TN);
	n.getParam("weightL_topic_name", weight_L_TN);
	n.getParam("weightR_topic_name", weight_R_TN);
	n.getParam("DH_Xtr_L", DH_Xtr_L);
	n.getParam("DH_Xrot_L", DH_Xrot_L);
	n.getParam("DH_Ztr_L", DH_Ztr_L);
	n.getParam("DH_Zrot_L", DH_Zrot_L);
	n.getParam("DH_Xtr_R", DH_Xtr_R);
	n.getParam("DH_Xrot_R", DH_Xrot_R);
	n.getParam("DH_Ztr_R", DH_Ztr_R);
	n.getParam("DH_Zrot_R", DH_Zrot_R);
	n.getParam("T_t2s_L", T_t2s_L);
	n.getParam("T_t2s_R", T_t2s_R);
	n.getParam("T_init_R", T_init_R);

	run_freq = 50;
	n.getParam("/force_estimation_frequency", run_freq);	// Override if configured
	ros::Rate loop_rate(run_freq);

	tf::TransformBroadcaster ik_br;
  	tf::Transform ik_tf;


	// cout << "topic" << arm_L_TN << endl;
	// cout << "topic" << arm_R_TN << endl;

	// ------------------------------------------------------------------------------------- Subscribe to topics
  	sub_arm_L		= n.subscribe(arm_L_TN, 1, arm_L__Callback);
  	sub_arm_R		= n.subscribe(arm_R_TN, 1, arm_R__Callback);
  	sub_hand_L		= n.subscribe(hand_L_TN, 1, hand_L__Callback);
  	sub_hand_R		= n.subscribe(hand_R_TN, 1, hand_R__Callback);


	// ------------------------------------------------------------------------------------- Inertias computation
  	RigidBodyInertia 	inert_Q_0, inert_Q_1, inert_Q_2, inert_Q_3, inert_Q_4, inert_Q_5, inert_Q_6;

  	inert_Q_0 = KDL::RigidBodyInertia(0, KDL::Vector(0.0, 0.0, 0.0)); // secondo cubo
	inert_Q_1 = KDL::RigidBodyInertia(cube_m_addon, KDL::Vector(0.0, 0.0, 0.0)); // secondo cubo
	inert_Q_2 = KDL::RigidBodyInertia(cube_m, KDL::Vector(0.0, 0.0, 0.09));
	inert_Q_3 = KDL::RigidBodyInertia(cube_m+0.07, KDL::Vector(0.0, 0.0, 0.0));
	inert_Q_4 = KDL::RigidBodyInertia(cube_m, KDL::Vector(0.0, 0.0, 0.09));
	inert_Q_5 = KDL::RigidBodyInertia(cube_wrist_m, KDL::Vector(0.0, 0.0, 0.0));//wrist
	inert_Q_6 = KDL::RigidBodyInertia(hand_m, KDL::Vector(0.0, 0.0, 0.0));//mano


	// ------------------------------------------------------------------------------------- Init chains
  	chain_L_root2s.addSegment(Segment(Joint(Joint::None),Frame(Rotation(T_t2s_L[0],T_t2s_L[4],T_t2s_L[8],T_t2s_L[1],T_t2s_L[5],T_t2s_L[9],T_t2s_L[2],T_t2s_L[6],T_t2s_L[10]),
		Vector(T_t2s_L[3],T_t2s_L[7],T_t2s_L[11]))));
	chain_L_root2s.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_L[0], DH_Xrot_L[0], DH_Ztr_L[0], DH_Zrot_L[0]),inert_Q_1));
	chain_L_root2s.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_L[1], DH_Xrot_L[1], DH_Ztr_L[1], DH_Zrot_L[1]),inert_Q_2));
	chain_L_root2s.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_L[2], DH_Xrot_L[2], DH_Ztr_L[2], DH_Zrot_L[2]),inert_Q_3));

	chain_L_s2h.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_L[3], DH_Xrot_L[3], DH_Ztr_L[3], DH_Zrot_L[3]),inert_Q_4));
	chain_L_s2h.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_L[4], DH_Xrot_L[4], DH_Ztr_L[4], DH_Zrot_L[4]),inert_Q_5));
	chain_L_s2h.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_L[5], DH_Xrot_L[5], DH_Ztr_L[5], DH_Zrot_L[5]),inert_Q_6));


	chain_R_root2s.addSegment(Segment(Joint(Joint::None),Frame(Rotation(T_t2s_R[0],T_t2s_R[4],T_t2s_R[8],T_t2s_R[1],T_t2s_R[5],T_t2s_R[9],T_t2s_R[2],T_t2s_R[6],T_t2s_R[10]),
		Vector(T_t2s_R[3],T_t2s_R[7],T_t2s_R[11]))));
	chain_R_root2s.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_R[0], DH_Xrot_R[0], DH_Ztr_R[0], DH_Zrot_R[0]),inert_Q_1));
	chain_R_root2s.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_R[1], DH_Xrot_R[1], DH_Ztr_R[1], DH_Zrot_R[1]),inert_Q_2));
	chain_R_root2s.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_R[2], DH_Xrot_R[2], DH_Ztr_R[2], DH_Zrot_R[2]),inert_Q_3));


	chain_R_s2h.addSegment(Segment(Joint(Joint::None),Frame(Rotation(T_init_R[0],T_init_R[4],T_init_R[8],T_init_R[1],T_init_R[5],T_init_R[9],T_init_R[2],T_init_R[6], T_init_R[10]),
		Vector(T_init_R[3],T_init_R[7],T_init_R[11]))));
	// chain_R_s2h.addSegment(Segment(Joint(Joint::None),Frame(Rotation(1,0,0,0,1,0,0,0,1),Vector(T_init_R[3],T_init_R[7],T_init_R[11]))));
	chain_R_s2h.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_R[3], DH_Xrot_R[3], DH_Ztr_R[3], DH_Zrot_R[3]),inert_Q_4));
	chain_R_s2h.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_R[4], DH_Xrot_R[4], DH_Ztr_R[4], DH_Zrot_R[4]),inert_Q_5));
	chain_R_s2h.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_R[5], DH_Xrot_R[5], DH_Ztr_R[5], DH_Zrot_R[5]),inert_Q_6));

	arm_R_q_r2s.resize(chain_R_root2s.getNrOfJoints());
	arm_R_q_s2h.resize(chain_R_s2h.getNrOfJoints());
	arm_L_q_r2s.resize(chain_L_root2s.getNrOfJoints());
	arm_L_q_s2h.resize(chain_L_s2h.getNrOfJoints());
	KDL::SetToZero(arm_R_q_r2s);
	KDL::SetToZero(arm_R_q_s2h);
	KDL::SetToZero(arm_L_q_r2s);
	KDL::SetToZero(arm_L_q_s2h);

  	jnt_to_pose_solver_L_root2s.reset(new KDL::ChainFkSolverPos_recursive(chain_L_root2s));
  	jnt_to_pose_solver_R_root2s.reset(new KDL::ChainFkSolverPos_recursive(chain_R_root2s));

  	jnt_to_pose_solver_L_s2h.reset(new KDL::ChainFkSolverPos_recursive(chain_L_s2h));
  	jnt_to_pose_solver_R_s2h.reset(new KDL::ChainFkSolverPos_recursive(chain_R_s2h));

  	jnt_to_jac_solver_L_s2h.reset(new KDL::ChainJntToJacSolver(chain_L_s2h));
  	jnt_to_jac_solver_R_s2h.reset(new KDL::ChainJntToJacSolver(chain_R_s2h));

  	arm_L_q_addon_init = Eigen::VectorXd::Zero(n_joints_addon);
  	arm_R_q_addon_init = Eigen::VectorXd::Zero(n_joints_addon);

  	// scout << T_init_R << endl;

	// cout << "[left arm] Number of joints:" << chain_L_s2h.getNrOfJoints() << endl;
	// cout << "[right arm] Number of joints:" << chain_L_s2h.getNrOfJoints() << endl;


	// while(!(arm_R_cb && arm_L_cb) && ros::ok()){
	// 	ros::spinOnce();

	// 	for(int i=0; i<n_joints_addon; i++)
	// 	{
	// 		arm_L_q_addon_init(i) = arm_L_q_r2s(i);
	// 		arm_R_q_addon_init(i) = arm_R_q_r2s(i);
	// 	}
	// }

  	// ------------------------------------------------------------------------------------- MAIN LOOP 
	while (ros::ok())
  	{
  		k=0;

		cart_R.M.GetQuaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w());
  		jnt_to_pose_solver_R_s2h->JntToCart(arm_R_q_s2h, cart_R);
  		ik_tf.setOrigin( tf::Vector3(cart_R.p[0],cart_R.p[1],cart_R.p[2]));
		ik_tf.setRotation(tf::Quaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w()));
		ik_br.sendTransform(tf::StampedTransform(ik_tf, ros::Time::now(), "root_link", "elbow_ego_ik"));
  		// --- cycle ---
  		
  		//DAC readings ------
		// std::cout << "test" << std::endl;
	size = daq.readAll(pack3D, false);
	// std::cout << "SIZE " << size << std::endl;

		if (size == 0)
		    continue;
		for (int k = 0; k < sensSize; k++)
		{
		    temp_x = pack3D[k * size + size - 1].x / cn_values[k * 3];
		    temp_y = pack3D[k * size + size - 1].y / cn_values[k * 3 + 1];
		    temp_z = pack3D[k * size + size - 1].z / cn_values[k * 3 + 2];
		    temp_abs = sqrt(temp_x * temp_x + temp_y * temp_y + temp_z * temp_z);
		    std::cout << std::setprecision(2) << "Sensor number: " << k << "|| x: \t" << temp_x << "\t y: \t" << temp_y << "\t z: \t" << temp_z << std::setprecision(4)<<"\t |a|: \t" << temp_abs << std::endl; // std::cout << "Sensor " << k << " datas:" << std::endl;

		    std::cout << "(Size: " << size << ")" << std::endl;

		    // for (int i = 0; i < size; i++)
		    // {
		    //     std::cout << "x: " << pack3D[k * size + i].x << " y: " << pack3D[k * size + i].y << " z: " << pack3D[k * size + i].z << std::endl;
		    // }
		    std::cout << "End \n";
		}
	  		ros::spinOnce();
	    	loop_rate.sleep();
  	}
}


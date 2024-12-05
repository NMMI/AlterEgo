
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
#include "optoforce-ros-publisher/pseudo_inversion.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include "std_msgs_stamped/Float64MultiArrayStamped.h"


#include "optoforce_sensor/opto.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <MovingMedianFilter.h>
#include <unistd.h>
#include <string.h>


using namespace KDL;
using namespace std;

#define n_joints_subchain 3
#define n_joints_addon 2
#define use_addon true
#define PI 3.14157

KDL::JntArray		arm_R_q_r2s(n_joints_subchain);
KDL::JntArray		arm_R_q_s2h(n_joints_subchain);
KDL::JntArray		arm_L_q_r2s(n_joints_subchain);
KDL::JntArray		arm_L_q_s2h(n_joints_subchain);

KDL::JntArray      	arm_L_q_old(6);
KDL::JntArray      	arm_R_q_old(6);
KDL::JntArray		full_arm_R(6);
double			    arm_L_q_addon_init(0);
double     			arm_R_q_addon_init(0);
const std::vector<double> cn_values = {450.47, 492.7, 65.175, 457.43, 466.23, 71.88, 446.54, 459.19, 69.98, 440.34, 469.9, 64.65}; // sensors 84 85 86 and 87
//const std::vector<double> cn_values = {450.47, 492.7, 64, 457.43, 466.23, 64, 446.54, 459.19, 64, 440.34, 469.9, 64}; // sensors 84 85 86 and 87



KDL::Vector gravity_v = KDL::Vector(0,0,-9.81);

bool				arm_R_cb = false;
bool				arm_L_cb = false;

bool				pub_weight_R = false;
bool				pub_weight_L = false;

double 				A_r_ext_old= 0;
double 				A_l_ext_old= 0;
double 				a_s,b_s,c_s;
std_msgs::Float64MultiArray 	weight_R;
std_msgs::Float64MultiArray 	weight_L;

double alpha_left(1);
double alpha(1);


/*---------------------------------------------------------------*
* IMU CALLBACK

*---------------------------------------------------------------*/

void imu_callback(const geometry_msgs::Vector3& msg){
gravity_v = KDL::Vector(9.81 * sin(msg.y),0,-9.81 * cos(msg.y)); // - * - = +
// std::cout<< std::fixed<<std::setprecision(2)<<"gravity:"<<gravity_v;

}

/*---------------------------------------------------------------------*
* LEFT ARM CALLBACK                                                    *
*                                                                      *
*----------------------------------------------------------------------*/
void arm_L__Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	for (int i=0; i<n_joints_subchain; i++)
	{
		if(i==0){
			arm_L_q_r2s(i) = -alpha_left*(double)msg->data[i]; //encoder nella nuova spalla sono montati al contrario
		}
		else{
			arm_L_q_r2s(i) = alpha_left*(double)msg->data[i];
		}
		arm_L_q_s2h(i) = alpha_left*(double)msg->data[i+3];
	
	}
	

	if(!arm_L_cb)
	{
			arm_L_q_addon_init = arm_L_q_r2s(1)/2;

	}

	if(use_addon)
	{
			arm_L_q_r2s(1) = arm_L_q_r2s(1)/2 - arm_L_q_addon_init;			
	}

	for (int i=0; i<n_joints_subchain; i++)
		{
			arm_L_q_old(i)=arm_L_q_r2s(i);
			arm_L_q_old(i+3)=arm_L_q_s2h(i);
		}
	arm_L_cb = true;
}

void hand_L__Callback(const std_msgs::Float64::ConstPtr& msg){
	pub_weight_L=(msg->data>0.5);
}

/*---------------------------------------------------------------------*
* RIGHT ARM CALLBACK                                                   *
*                                                                      *
*----------------------------------------------------------------------*/
void arm_R__Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	for (int i=0; i<n_joints_subchain; i++)
	{
		if(i==0){
			arm_R_q_r2s(i) = -alpha_left*(double)msg->data[i] + (1-alpha_left)*arm_R_q_old(i);
		}
		else{
			arm_R_q_r2s(i) = alpha_left*(double)msg->data[i] + (1-alpha_left)*arm_R_q_old(i);
		}
		

		arm_R_q_s2h(i) = alpha_left*(double)msg->data[i+3] + (1-alpha_left)*arm_R_q_old(i+3);
		
	}

	if(!arm_R_cb)
	{	
		arm_R_q_addon_init= arm_R_q_r2s(1)/2;
	}

	if(use_addon)
	{
		arm_R_q_r2s(1) = arm_R_q_r2s(1)/2 - arm_R_q_addon_init;
			
	}

	for (int i=0; i<n_joints_subchain; i++)
		{
			arm_R_q_old(i)=arm_R_q_r2s(i);
			arm_R_q_old(i+3)=arm_R_q_s2h(i);
		}
	arm_R_cb = true;
}

void hand_R__Callback(const std_msgs::Float64::ConstPtr& msg){
	pub_weight_R=(msg->data>0.5);
}

/*---------------------------------------------------------------------*
* MAIN                                                                 *
*                                                                      *
*----------------------------------------------------------------------*/
int  				firstreading=0;
int main(int argc, char **argv){
	double 					weightbias=0; 
	double 					weightbias_L=0; 


	int 					k(0);
	int 					run_freq;



	string 					arm_L_TN;
	string 					arm_R_TN;
	string 					hand_L_TN;
	string 					hand_R_TN;
	string 					weight_L_TN;
	string 					weight_R_TN;
	string 					IMU_TN;
	string                  right_weight_topic;
	string                  left_weight_topic;
	
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


	weight_R.data.resize(2);
	weight_L.data.resize(2);


	

	bool DATACOLLECT =true;
	std_msgs_stamped::Float64MultiArrayStamped estimations; //[lxSensor,A_l_ext,A_l,bias,lx_m,lx_bias, ||  rxSensor,A_r_ext,A_r,rx_m,rx_bias]
	estimations.data.resize(13);
	std_msgs_stamped::Float64MultiArrayStamped lxArm;
	lxArm.data.resize(6);
	std_msgs_stamped::Float64MultiArrayStamped rxArm;
	rxArm.data.resize(6);
	geometry_msgs::Vector3Stamped lx_hand_pos;
	geometry_msgs::Vector3Stamped rx_hand_pos;


	KDL::Frame     			cart_R, cart_L, cart_R_hand, cart_L_hand;


	Eigen::Quaterniond	act_quat;


	ros::Subscriber 		sub_arm_L;
	ros::Subscriber 		sub_arm_R;
	ros::Subscriber 		sub_hand_L;
	ros::Subscriber 		sub_hand_R;
	ros::Subscriber 		sub_imu;

	ros::Publisher			weight_R_pub,weight_L_pub;
	ros::Publisher			debug_estimation,debug_lx_hand,debug_rx_hand,debug_rx_joints,debug_lx_joints;


	KDL::Chain 									chain_L_root2elbow, chain_L_s2h;
	KDL::Chain 									chain_R_root2elbow, chain_R_s2h, chain_R_fullarm;
	boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_L_root2elbow, jnt_to_pose_solver_R_root2elbow, jnt_to_pose_solver_L_s2h, jnt_to_pose_solver_R_s2h,joint_to_pose_R_fullarm;
	boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_L_s2h, jnt_to_jac_solver_R_s2h;
	boost::scoped_ptr<KDL::ChainDynParam> 		dyn_solver_L, dyn_solver_R;



	KDL::Vector grav_elbowkdl(0,0,0);
	KDL::Vector grav_elbowkdl_left(0,0,0);
	KDL::Vector braccio_0(0,0,0);
	KDL::Vector braccio_1(0,0,0);
	KDL::Vector braccio_2(0,0,0);
	KDL::Vector momentum(0,0,0);
	KDL::Vector weightbias_v(0,0,0);
	KDL::Vector weightbias_v_global(0,0,0);
	KDL::Vector weightbias_v_L(0,0,0);
	KDL::Vector weightbias_v_global_L(0,0,0);



	double 				cube_m(0.5);
	double              cube_wrist_m(0.620); 
	// double              cube_wrist_m(0.75); 
	double 				cube_m_addon(0.65);
	double 				hand_m(0.390);
	// double 				hand_m(0.5);
	double              flange_m(0.085);




	// ------------------------------------------------------------------------------------- Init node
	ros::init(argc, argv, "force_estimation");
	ros::NodeHandle n;
	
	ns = ros::this_node::getNamespace();
	ns = ns.substr(1,ns.length()-1);

	// ------------------------------------------------------------------------------------- Check/save args
	n.getParam("force_estimation/armL_topic_name", arm_L_TN);
	n.getParam("force_estimation/armR_topic_name", arm_R_TN);
	n.getParam("force_estimation/handL_topic_name", hand_L_TN);
	n.getParam("force_estimation/imu_grav_topic_name",IMU_TN);
	n.getParam("force_estimation/handR_topic_name", hand_R_TN);
	n.getParam("force_estimation/weightL_topic_name", weight_L_TN);
	n.getParam("force_estimation/weightR_topic_name", weight_R_TN);
	n.getParam("force_estimation/DH_Xtr_L", DH_Xtr_L);
	n.getParam("force_estimation/DH_Xrot_L", DH_Xrot_L);
	n.getParam("force_estimation/DH_Ztr_L", DH_Ztr_L);
	n.getParam("force_estimation/DH_Zrot_L", DH_Zrot_L);
	n.getParam("force_estimation/DH_Xtr_R", DH_Xtr_R);
	n.getParam("force_estimation/DH_Xrot_R", DH_Xrot_R);
	n.getParam("force_estimation/DH_Ztr_R", DH_Ztr_R);
	n.getParam("force_estimation/DH_Zrot_R", DH_Zrot_R);
	n.getParam("force_estimation/T_t2s_L", T_t2s_L);
	n.getParam("force_estimation/T_t2s_R", T_t2s_R);
	n.getParam("force_estimation/T_init_R", T_init_R);
	n.getParam("force_estimation/right_weight_topic",right_weight_topic);
	n.getParam("force_estimation/left_weight_topic",left_weight_topic);
	n.getParam("force_estimation/a_sensor",a_s);
	n.getParam("force_estimation/b_sensor",b_s);
	n.getParam("force_estimation/c_sensor",c_s);
	n.getParam("force_estimation/debug",DATACOLLECT);

	n.getParam("force_estimation/cube_mass", cube_m);
	n.getParam("force_estimation/cube_wrist_mass", cube_wrist_m);
	n.getParam("force_estimation/cube_addon_mass", cube_m_addon);
	n.getParam("force_estimation/hand_mass", hand_m);


	const double 				total_arm_mass=cube_m+cube_wrist_m+hand_m+flange_m;



	//initialise filter
	int winSize=25;
    n.getParam("weight_w_size", winSize);
    MovingMedianFilter weight_filter_L(winSize,0);
    MovingMedianFilter weight_filter_R(winSize,0);

	run_freq = 15;
	n.getParam("/force_estimation_frequency", run_freq);	// Override if configured
	ros::Rate loop_rate(run_freq);

	tf::TransformBroadcaster ik_br;
	tf::Transform ik_tf;

	// AndCav 
	OptoDAQ daq;
	OptoPorts ports;
	usleep(2500000); // We wait some ms to be sure about OptoPorts enumerated PortList

		


	int iSpeed = 30;         //impostare Rate a 30Hz                                                                                                      // Speed in Hz
	int iFilter = 15;
	uint8_t to_green_left=0;
	uint8_t to_green_right=0;                                                                                                           // Filter in Hz
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

	double cube_bias=0;
	double wrist_bias=0;
	double hand_bias=0;


	int speed, filter;

	OptoPackage *pack3D = 0;
	int size = daq.readAll(pack3D, false);
	int sensSize = daq.getSensorSize();
	std::cout << "STARTING \n"<< sensSize;
	sensSize=2;
	double A_r,A_l,A_r_ext,A_l_ext,A_l_elbow,A_r_elbow;
	Eigen::Matrix<double, 3, 2> R_sensors;
	Eigen::Matrix<double, 3, 2> L_sensors;
	Eigen::Matrix<double, 3, 3> R_sensors_rotation_r2elbow;
	Eigen::Matrix<double, 3, 3> L_sensors_rotation_r2elbow;
	Eigen::Matrix<double, 3, 3> R_x_minus_90;
	Eigen::Matrix<double, 3, 3> R_y;

	Eigen::Matrix<double, 3,1> R_force_measured;
	Eigen::Matrix<double, 3,1> gravity_elbowFrame;
	Eigen::Matrix<double, 3,1> R_weightArm_estimated;
	Eigen::Matrix<double, 3,1> gravity_R;
	Eigen::Matrix<double, 3,1> R_weight_estimated;

	Eigen::Matrix<double, 3,1> L_force_measured;
	// Eigen::Matrix<double, 3,1> gravity_L_elbowFrame;
	Eigen::Matrix<double, 3,1> L_weightArm_estimated;
	Eigen::Matrix<double, 3,1> gravity_L;
	Eigen::Matrix<double, 3,1> L_weight_estimated;


	Eigen::Matrix<double, 3,1> g;
	g << 0,0,-9.81;

	KDL::JntArray 		gravity_R_kdl;
	KDL::JntArray 		gravity_L_kdl;
	KDL::Jacobian  		J_R_s2h_kdl;
	Eigen::Matrix<double, 3,3> J_R_s2h;
	Eigen::Matrix<double, 3,3> J_R_s2h_transp_inv;
	Eigen::Matrix<double, 3,3> K_d;
	K_d =  0.1*Eigen::Matrix<double, 3, 3>::Identity();

	daq.zeroAll();

	R_x_minus_90 = Eigen::Matrix<double, 3, 3>::Zero();
	R_x_minus_90(0,0) = 1;
	R_x_minus_90(1,2) = 1;
	R_x_minus_90(2,1) = -1;
	R_y = Eigen::Matrix<double, 3, 3>::Zero();
	R_y(1,1) = 1;
	double _initialShoulder=0;

	// cout << "topic" << arm_L_TN << endl;
	// cout << "topic" << arm_R_TN << endl;

	// ------------------------------------------------------------------------------------- Subscribe to topics
	sub_arm_L		 = n.subscribe(arm_L_TN, 1, arm_L__Callback);
	sub_arm_R		 = n.subscribe(arm_R_TN, 1, arm_R__Callback);
	sub_hand_L		 = n.subscribe(hand_L_TN, 1, hand_L__Callback);
	sub_hand_R		 = n.subscribe(hand_R_TN, 1, hand_R__Callback);
	sub_imu 		 = n.subscribe(IMU_TN, 1, imu_callback);


	//-------------------------------------------------------------------------------------- Publish to topics

	weight_R_pub     =n.advertise<std_msgs::Float64MultiArray>(weight_R_TN, 1);
	weight_L_pub     =n.advertise<std_msgs::Float64MultiArray>(weight_L_TN, 1);
	debug_estimation =n.advertise<std_msgs_stamped::Float64MultiArrayStamped>("/debug/estimations",1);
	debug_lx_hand 	 =n.advertise<geometry_msgs::Vector3Stamped>("/debug/lx_hand",1);
	debug_rx_hand 	 =n.advertise<geometry_msgs::Vector3Stamped>("/debug/rx_hand",1);
	debug_rx_joints  =n.advertise<std_msgs_stamped::Float64MultiArrayStamped>("/debug/rx_joints",1);
	debug_lx_joints  =n.advertise<std_msgs_stamped::Float64MultiArrayStamped>("/debug/lx_joints",1);

	// ------------------------------------------------------------------------------------- Inertias computation
	RigidBodyInertia 	inert_Q_0, inert_Q_1, inert_Q_2, inert_Q_3, inert_Q_4, inert_Q_5, inert_Q_6,inert_Q_4_L;

  	inert_Q_0 = KDL::RigidBodyInertia(0, KDL::Vector(0.0, 0.0, 0.0)); // secondo cubo
	inert_Q_1 = KDL::RigidBodyInertia(cube_m_addon, KDL::Vector(0.0, 0.0, 0.0)); // secondo cubo
	inert_Q_2 = KDL::RigidBodyInertia(cube_m, KDL::Vector(0.0, 0.0, 0.09));
	inert_Q_3 = KDL::RigidBodyInertia(cube_m, KDL::Vector(0.0, 0.0, 0.0));
	inert_Q_4 = KDL::RigidBodyInertia(cube_m, KDL::Vector(0.0, 0.0, 0.09));
	inert_Q_4_L = KDL::RigidBodyInertia(flange_m, KDL::Vector(0.0, 0.0, 0.028));
	inert_Q_5 = KDL::RigidBodyInertia(cube_wrist_m, KDL::Vector(0.0, 0.0, 0.0));//wrist
	inert_Q_6 = KDL::RigidBodyInertia(hand_m, KDL::Vector(0.0, 0.0, 0.0));//mano


	// ------------------------------------------------------------------------------------- Init chains
	chain_L_root2elbow.addSegment(Segment(Joint(Joint::None),Frame(Rotation(T_t2s_L[0],T_t2s_L[4],T_t2s_L[8],T_t2s_L[1],T_t2s_L[5],T_t2s_L[9],T_t2s_L[2],T_t2s_L[6],T_t2s_L[10]),
		Vector(T_t2s_L[3],T_t2s_L[7],T_t2s_L[11]))));
	chain_L_root2elbow.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_L[0], DH_Xrot_L[0], DH_Ztr_L[0], DH_Zrot_L[0]),inert_Q_1));
	chain_L_root2elbow.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_L[1], DH_Xrot_L[1], DH_Ztr_L[1], DH_Zrot_L[1]),inert_Q_2));
	chain_L_root2elbow.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_L[2], DH_Xrot_L[2], DH_Ztr_L[2], DH_Zrot_L[2]),inert_Q_3));


	// chain_L_s2h.addSegment(Segment(Joint(Joint::None),Frame(Rotation(-1, 0, 0, 0, 0, 1, 0, 1, 0),Vector(0,0,0))));
	chain_L_s2h.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_L[3], DH_Xrot_L[3], DH_Ztr_L[3], DH_Zrot_L[3]),inert_Q_4_L));
	chain_L_s2h.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_L[4], DH_Xrot_L[4], DH_Ztr_L[4], DH_Zrot_L[4]),inert_Q_5));
	chain_L_s2h.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_L[5], DH_Xrot_L[5], DH_Ztr_L[5], DH_Zrot_L[5]),inert_Q_6));


	chain_R_root2elbow.addSegment(Segment(Joint(Joint::None),Frame(Rotation(T_t2s_R[0],T_t2s_R[4],T_t2s_R[8],T_t2s_R[1],T_t2s_R[5],T_t2s_R[9],T_t2s_R[2],T_t2s_R[6],T_t2s_R[10]),
		Vector(T_t2s_R[3],T_t2s_R[7],T_t2s_R[11]))));
	chain_R_root2elbow.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_R[0], DH_Xrot_R[0], DH_Ztr_R[0], DH_Zrot_R[0]),inert_Q_1));
	chain_R_root2elbow.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_R[1], DH_Xrot_R[1], DH_Ztr_R[1], DH_Zrot_R[1]),inert_Q_2));
	chain_R_root2elbow.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_R[2], DH_Xrot_R[2], DH_Ztr_R[2], DH_Zrot_R[2]),inert_Q_3));
	
	// chain_R_s2h.addSegment(Segment(Joint(Joint::None),Frame(Rotation(1,0,0,0,0, -1,0,1, 0),Vector(0,0,0))));
	chain_R_s2h.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_R[3], DH_Xrot_R[3], DH_Ztr_R[3], DH_Zrot_R[3]),inert_Q_4));
	chain_R_s2h.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_R[4], DH_Xrot_R[4], DH_Ztr_R[4], DH_Zrot_R[4]),inert_Q_5));
	chain_R_s2h.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_R[5], DH_Xrot_R[5], DH_Ztr_R[5], DH_Zrot_R[5]),inert_Q_6));




	//chain just for debug. delete when unnecessary
	// chain_R_fullarm.addSegment(Segment(Joint(Joint::None),Frame(Rotation(T_t2s_R[0],T_t2s_R[4],T_t2s_R[8],T_t2s_R[1],T_t2s_R[5],T_t2s_R[9],T_t2s_R[2],T_t2s_R[6],T_t2s_R[10]),
	// 	Vector(T_t2s_R[3],T_t2s_R[7],T_t2s_R[11]))));
	// chain_R_fullarm.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_R[0], DH_Xrot_R[0], DH_Ztr_R[0], DH_Zrot_R[0]),inert_Q_1));
	// chain_R_fullarm.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_R[1], DH_Xrot_R[1], DH_Ztr_R[1], DH_Zrot_R[1]),inert_Q_2));
	// chain_R_fullarm.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_R[2], DH_Xrot_R[2], DH_Ztr_R[2], DH_Zrot_R[2]),inert_Q_3));
	// chain_R_fullarm.addSegment(Segment(Joint(Joint::None),Frame(Rotation(T_init_R[0],T_init_R[4],T_init_R[8],T_init_R[1],T_init_R[5],T_init_R[9],T_init_R[2],T_init_R[6], T_init_R[10]),
	// 	Vector(T_init_R[3],T_init_R[7],T_init_R[11]))));
	// chain_R_fullarm.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_R[3], DH_Xrot_R[3], DH_Ztr_R[3], DH_Zrot_R[3]),inert_Q_4));
	// chain_R_fullarm.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_R[4], DH_Xrot_R[4], DH_Ztr_R[4], DH_Zrot_R[4]),inert_Q_5));
	// chain_R_fullarm.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(DH_Xtr_R[5], DH_Xrot_R[5], DH_Ztr_R[5], DH_Zrot_R[5]),inert_Q_6));



	arm_R_q_r2s.resize(chain_R_root2elbow.getNrOfJoints());
	arm_R_q_s2h.resize(chain_R_s2h.getNrOfJoints());
	// full_arm_R.resize(chain_R_fullarm.getNrOfJoints());
	arm_L_q_r2s.resize(chain_L_root2elbow.getNrOfJoints());
	arm_L_q_s2h.resize(chain_L_s2h.getNrOfJoints());

	KDL::SetToZero(arm_R_q_r2s);
	KDL::SetToZero(arm_R_q_s2h);
	KDL::SetToZero(arm_L_q_r2s);
	KDL::SetToZero(arm_L_q_s2h);
	KDL::SetToZero(arm_L_q_old);

	jnt_to_pose_solver_L_root2elbow.reset(new KDL::ChainFkSolverPos_recursive(chain_L_root2elbow));
	jnt_to_pose_solver_R_root2elbow.reset(new KDL::ChainFkSolverPos_recursive(chain_R_root2elbow));

	jnt_to_pose_solver_L_s2h.reset(new KDL::ChainFkSolverPos_recursive(chain_L_s2h));
	jnt_to_pose_solver_R_s2h.reset(new KDL::ChainFkSolverPos_recursive(chain_R_s2h));

	jnt_to_jac_solver_L_s2h.reset(new KDL::ChainJntToJacSolver(chain_L_s2h));
	jnt_to_jac_solver_R_s2h.reset(new KDL::ChainJntToJacSolver(chain_R_s2h));
	J_R_s2h_kdl.resize(chain_R_s2h.getNrOfJoints());


	// joint_to_pose_R_fullarm.reset(new KDL::ChainFkSolverPos_recursive(chain_R_fullarm));


	dyn_solver_R.reset(new KDL::ChainDynParam(chain_R_s2h,gravity_v));
	gravity_R_kdl.resize(chain_R_s2h.getNrOfJoints());


	arm_L_q_addon_init = 0;
	arm_R_q_addon_init = 0;

  	// scout << T_init_R << endl;

	// cout << "[left arm] Number of joints:" << chain_L_s2h.getNrOfJoints() << endl;
	// cout << "[right arm] Number of joints:" << chain_L_s2h.getNrOfJoints() << endl;

	while(!(arm_R_cb) && !(arm_L_cb) &&
	 ros::ok()){
		ros::spinOnce();
	}

  	// ------------------------------------------------------------------------------------- MAIN LOOP 

  	double weight_l_old=0;
  	double weight_r_old=0;

	while (ros::ok())
	{
		k=0;

		// jnt_to_pose_solver_R_root2elbow->JntToCart(arm_R_q_r2s, cart_R);
		// jnt_to_pose_solver_L_root2elbow->JntToCart(arm_L_q_r2s, cart_L);

		for (int i = 0; i<3; i++)
			for(int j = 0; j<3; j++){
				R_sensors_rotation_r2elbow(i,j) = cart_R.M(i,j);
				L_sensors_rotation_r2elbow(i,j) = cart_L.M(i,j);
			}



		//-------just debugging
		/*
	

		cart_R.M.GetQuaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w());
		ik_tf.setOrigin( tf::Vector3(cart_R.p[0],cart_R.p[1],cart_R.p[2]));
		ik_tf.setRotation(tf::Quaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w()));
		ik_br.sendTransform(tf::StampedTransform(ik_tf, ros::Time::now(), "root_link", "right_shoulder"));
		
		jnt_to_pose_solver_R_root2elbow->JntToCart(arm_R_q_r2s, cart_R,2);
		cart_R.M.GetQuaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w());
		ik_tf.setOrigin( tf::Vector3(cart_R.p[0],cart_R.p[1],cart_R.p[2]));
		ik_tf.setRotation(tf::Quaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w()));
		ik_br.sendTransform(tf::StampedTransform(ik_tf, ros::Time::now(), "root_link", "right_shou2"));
		
		jnt_to_pose_solver_R_root2elbow->JntToCart(arm_R_q_r2s, cart_R,3);
		cart_R.M.GetQuaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w());
		ik_tf.setOrigin( tf::Vector3(cart_R.p[0],cart_R.p[1],cart_R.p[2]));
		ik_tf.setRotation(tf::Quaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w()));
		ik_br.sendTransform(tf::StampedTransform(ik_tf, ros::Time::now(), "root_link", "right_e1"));		

		*/
		jnt_to_pose_solver_L_root2elbow->JntToCart(arm_L_q_r2s, cart_L,2);
		cart_L.M.GetQuaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w());
		ik_tf.setOrigin( tf::Vector3(cart_L.p[0],cart_L.p[1],cart_L.p[2]));
		ik_tf.setRotation(tf::Quaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w()));
		ik_br.sendTransform(tf::StampedTransform(ik_tf, ros::Time::now(), "root_link", "left_shoulder"));
		/*
		jnt_to_pose_solver_L_root2elbow->JntToCart(arm_L_q_r2s, cart_L,2);
		cart_L.M.GetQuaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w());
		ik_tf.setOrigin( tf::Vector3(cart_L.p[0],cart_L.p[1],cart_L.p[2]));
		ik_tf.setRotation(tf::Quaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w()));
		ik_br.sendTransform(tf::StampedTransform(ik_tf, ros::Time::now(), "root_link", "leftshou2"));


		jnt_to_pose_solver_L_root2elbow->JntToCart(arm_L_q_r2s, cart_L,2);
		cart_L.M.GetQuaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w());
		ik_tf.setOrigin( tf::Vector3(cart_L.p[0],cart_L.p[1],cart_L.p[2]));
		ik_tf.setRotation(tf::Quaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w()));
		ik_br.sendTransform(tf::StampedTransform(ik_tf, ros::Time::now(), "root_link", "left_elbow1"));



		*/		
		//--------end just debugging

		
		jnt_to_pose_solver_R_root2elbow->JntToCart(arm_R_q_r2s, cart_R);
		cart_R.M.GetQuaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w());
		ik_tf.setOrigin( tf::Vector3(cart_R.p[0],cart_R.p[1],cart_R.p[2]));
		ik_tf.setRotation(tf::Quaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w()));
		ik_br.sendTransform(tf::StampedTransform(ik_tf, ros::Time::now(), "root_link", "right_elbow"));
		

		// jnt_to_pose_solver_R_s2h->JntToCart(arm_R_q_s2h, cart_R_hand, 1);
		// cart_R_hand.M.GetQuaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w());
		// ik_tf.setOrigin( tf::Vector3(cart_R_hand.p[0],cart_R_hand.p[1],cart_R_hand.p[2]));
		// ik_tf.setRotation(tf::Quaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w()));
		// ik_br.sendTransform(tf::StampedTransform(ik_tf, ros::Time::now(), "root_link", "elbow_R_secondchain"));

		jnt_to_pose_solver_R_s2h->JntToCart(arm_R_q_s2h, cart_R_hand);
		cart_R_hand.M.GetQuaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w());
		ik_tf.setOrigin( tf::Vector3(cart_R_hand.p[0],cart_R_hand.p[1],cart_R_hand.p[2]));
		ik_tf.setRotation(tf::Quaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w()));
		ik_br.sendTransform(tf::StampedTransform(ik_tf, ros::Time::now(), "right_elbow", "hand_R"));


		jnt_to_pose_solver_L_root2elbow->JntToCart(arm_L_q_r2s, cart_L);
		cart_L.M.GetQuaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w());
		ik_tf.setOrigin( tf::Vector3(cart_L.p[0],cart_L.p[1],cart_L.p[2]));
		ik_tf.setRotation(tf::Quaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w()));
		ik_br.sendTransform(tf::StampedTransform(ik_tf, ros::Time::now(), "root_link", "left_elbow"));

		
		// jnt_to_pose_solver_L_s2h->JntToCart(arm_L_q_s2h, cart_L_hand,2);
		// cart_L_hand.M.GetQuaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w());
		// ik_tf.setOrigin( tf::Vector3(cart_L_hand.p[0],cart_L_hand.p[1],cart_L_hand.p[2]));
		// ik_tf.setRotation(tf::Quaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w()));
		// ik_br.sendTransform(tf::StampedTransform(ik_tf, ros::Time::now(), "left_elbow", "wrist2"));

		jnt_to_pose_solver_L_s2h->JntToCart(arm_L_q_s2h, cart_L_hand);
		cart_L_hand.M.GetQuaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w());
		ik_tf.setOrigin( tf::Vector3(cart_L_hand.p[0],cart_L_hand.p[1],cart_L_hand.p[2]));
		ik_tf.setRotation(tf::Quaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w()));
		ik_br.sendTransform(tf::StampedTransform(ik_tf, ros::Time::now(), "left_elbow", "hand_L"));
		

  		// --- cycle ---

  		//DAC readings ------
		// std::cout << "test" << std::endl;
			size = daq.readAll(pack3D, false);
		// std::cout << "SIZE " << size << std::endl;


		//std::cout<<"Right: x="<<cart_R_hand.p[0]<<"\t y="<<cart_R_hand.p[1]<<"\t z="<<cart_R_hand.p[2]<<std::endl;
		// std::cout<<"Left : x="<<cart_L_hand.p.x()<<"\t y="<<cart_L_hand.p.y()<<"\t z="<<cart_L_hand.p.z()<<std::endl;

			if (size == 0)
				continue;
			for (int k = 0; k < sensSize; k++)
			{
				R_sensors(0,k) = pack3D[k * size + size - 1].x / cn_values[k * 3];
				R_sensors(1,k) = pack3D[k * size + size - 1].y / cn_values[k * 3 + 1];
				R_sensors(2,k) = pack3D[k * size + size - 1].z / cn_values[k * 3 + 2];
				L_sensors(0,k) = pack3D[(k+2) * size + size - 1].x / cn_values[(k+2) * 3];
				L_sensors(1,k) = pack3D[(k+2) * size + size - 1].y / cn_values[(k+2) * 3 + 1];
				L_sensors(2,k) = pack3D[(k+2) * size + size - 1].z / cn_values[(k+2) * 3 + 2];
				// std::cout<< std::fixed<<std::setprecision(2)<<"sensor 3 "<<L_sensors(2,0)<<"\t sensor 4 "<< L_sensors(2,1)<<" || L_readings"<< L_force_measured(2)<<std::endl;
				// std::cout<< std::fixed<<std::setprecision(2)<<"LeftArm: "<< arm_L_q_r2s(0)*57.295<<"\t"<<arm_L_q_r2s(1)*57.295<<"\t"<<arm_L_q_r2s(2)*57.295<<"\t"<<arm_L_q_s2h(0)*57.295<<std::endl;
				// std::cout<< std::fixed<<std::setprecision(2)<<"sensor 1 "<<R_sensors(2,0)<<"\t sensor 2 "<< R_sensors(2,1)<<" || R_readings"<< R_force_measured(2)<<std::endl;
				// std::cout<< std::fixed<<std::setprecision(2)<<"RightArm: "<< arm_R_q_r2s(0)*57.295<<"\t"<<arm_R_q_r2s(1)*57.295<<"\t"<<arm_R_q_r2s(2)*57.295<<"\t"<<arm_R_q_s2h(0)*57.295<<std::endl;
				//sembrano corrette 
			}




		/*
			 M - Rb
		A = --------
			 a + b
		ExternalForce = Readings - A ;
		*/

		KDL::Vector dmp;

		R_force_measured = (R_sensors.block<3,1>(0,0) + R_sensors.block<3,1>(0,1));
		grav_elbowkdl=cart_R.M.Inverse()*gravity_v;
		

		braccio_0=Vector(0.030*sin(arm_R_q_s2h(0)),0.030*-cos(arm_R_q_s2h(0)),0);
		
		dmp = braccio_0*grav_elbowkdl*flange_m;
		momentum = dmp;
		
		braccio_1=Vector(0.09*sin(arm_R_q_s2h(0)),0.09*-cos(arm_R_q_s2h(0)),0);
		dmp = braccio_1*grav_elbowkdl*cube_m;
		momentum += dmp;


		braccio_2= Vector(0.165*sin(arm_R_q_s2h(0)),0.165*-cos(arm_R_q_s2h(0)),0);
		dmp = braccio_2*grav_elbowkdl*cube_wrist_m;

		momentum +=dmp;
		dmp = cart_R_hand.p*grav_elbowkdl*hand_m;

		momentum+=dmp;

		A_r_elbow=(-cube_m*grav_elbowkdl.x()*c_s + cube_m*grav_elbowkdl.y()*b_s)/(a_s+b_s);
		A_r=A_r_elbow + (-momentum.z()+grav_elbowkdl.y()*total_arm_mass*b_s-grav_elbowkdl.x()*total_arm_mass*c_s)/(a_s+b_s); //it's better with 1*cube_m INSTEAD OF 2+, why??
	



		// LEFT ARM

		momentum=Vector(0,0,0);
		L_force_measured = (L_sensors.block<3,1>(0,0)*2); // SOMMA DEI SENSORI ( sensor 4 is completely useless) 
		grav_elbowkdl_left=cart_L.M.Inverse()*gravity_v;


		braccio_0=Vector(0.030*sin(arm_L_q_s2h(0)),0.030*-cos(arm_L_q_s2h(0)),0);

		dmp = braccio_0*grav_elbowkdl_left*flange_m;
		momentum = dmp;
		// std::cout<<std::fixed<<std::setprecision(3)<<"momentum alla flangia:\t"<<dmp<<std::endl;
		braccio_1=Vector(0.09*sin(arm_L_q_s2h(0)),0.09*-cos(arm_L_q_s2h(0)),0);
		dmp = braccio_1*grav_elbowkdl_left*cube_m;
		momentum += dmp;
		// std::cout<<std::fixed<<std::setprecision(3)<<"momentum al cubo:\t"<<dmp<<std::endl;

		braccio_2= Vector(0.165*sin(arm_L_q_s2h(0)),0.165*-cos(arm_L_q_s2h(0)),0);
		dmp = braccio_2*grav_elbowkdl_left*cube_wrist_m;

		momentum +=dmp;

		// std::cout<<std::fixed<<std::setprecision(3)<<"momentum al polso:\t"<<dmp<<std::endl;


		dmp = cart_L_hand.p*grav_elbowkdl_left*hand_m;

		momentum+=dmp;

		// std::cout<<std::fixed<<std::setprecision(3)<<"momentum alla mano:\t"<<dmp<<std::endl;
		//DEBUGmomentum=0*momentum; 
		A_l_elbow=( cube_m*grav_elbowkdl_left.x()*c_s + cube_m*grav_elbowkdl_left.y()*b_s )/(a_s+b_s);
		A_l= A_l_elbow + ( momentum.z() + grav_elbowkdl_left.y() * total_arm_mass*b_s + grav_elbowkdl_left.x() * total_arm_mass* c_s )/( a_s + b_s );  //whole arm
		
		// A_l=-((momentum.z()-grav_elbowkdl_left.y()*(hand_m+cube_m+cube_wrist_m)*b_s)/(a_s+b_s));//A_l= A_l_elbow +(-momentum.z()+grav_elbowkdl_left.y()*(flange_m)*b_s+flange_m*grav_elbowkdl_left.x()*c_s)/(a_s+b_s); //only elbow
		// std::cout<<std::fixed<<std::setprecision(3)<<cart_L_hand.p.x()<<cart_L_hand.p.y()<<std::endl;
		// std::cout<<std::fixed<<std::setprecision(3)<<"gravity:"<<grav_elbowkdl_left[0]<<"\t"<<grav_elbowkdl_left[1]<<"\t"<<grav_elbowkdl_left[2]<<std::endl;
		// std::cout<<std::fixed<<std::setprecision(3)<<"momentum:"<<momentum.z()<<"\t R_y:"<<grav_elbowkdl_left[1]*total_arm_mass + cube_m*grav_elbowkdl_left.y()<<"\t R_x:"<<grav_elbowkdl_left[0]*total_arm_mass +cube_m*grav_elbowkdl_left.x()<<std::endl;
		
		if(firstreading<30){
			A_r_ext= R_force_measured(2) - A_r;
			A_l_ext= L_force_measured(2) - A_l;
			firstreading+=1;
			//std::cout<<"\n sample n:"<< firstreading;
			if(firstreading>=20) {weightbias+=A_r; weightbias_L+= A_l;}
			
			if(firstreading==30){
				weightbias=weightbias/11;
				weightbias_L=weightbias_L/11;
				A_r_ext_old=A_r;
				A_l_ext_old=A_l;
			}
			// std::cout<< std::fixed<<std::setprecision(2)<<"Reading:"<<L_force_measured(2)<<"\tatteso:"<<A_l<<"\tbias:"<<weightbias_L<<std::endl;
			// std::cout<< std::fixed<<std::setprecision(2)<<"Reading_R:"<<R_force_measured(2)<<"\tatteso:"<<A_r<<"\tbias:"<<weightbias<<std::endl;

		}
		if (firstreading>=30)
		{

			A_r_ext= alpha*(R_force_measured(2)+ weightbias - A_r) + (1-alpha)*A_r_ext_old;
			A_r_ext=weight_filter_R.advance(A_r_ext);
			A_r_ext_old=A_r_ext;
			


			A_l_ext= alpha*(L_force_measured(2) + weightbias_L - A_l) +(1-alpha)*A_l_ext_old;
			A_l_ext=weight_filter_L.advance(A_l_ext);
			A_l_ext_old=A_l_ext;

			// std::cout<< std::fixed<<std::setprecision(2)<<"Reading:"<<L_force_measured(2)<<"\tatteso:"<<A_l<<"\tbias:"<<weightbias_L<<std::endl;
			// std::cout<< std::fixed<<std::setprecision(2)<<"Reading_r:"<<R_force_measured(2)<<"\tatteso:"<<A_r<<"\tbias:"<<weightbias<<std::endl;
			// 
		}


		{
		
			// KDL::Vector planarGravity(1000*grav_elbowkdlt.x(),grav_elbowkdl.y(),0);// planarGravity*planarBraccio = momento
			// KDL::Vector planarBraccio(cart_R_hand.p.x(),cart_R_hand.p.y(),0);
			KDL::Vector crossProduct=cart_R_hand.p*grav_elbowkdl;  //moltiplico per 1000 per evitare approssimazioni a 0
			
			// double buffer=fabs((1000*(A_l_ext*(a_s+b_s)))/((1000*crossProduct.z() - 1000*planarGravity[1]*b_s)));
			double numerator = ((A_r_ext * (a_s + b_s)));
			double denom = (-crossProduct.z() + b_s * grav_elbowkdl.y() - c_s * grav_elbowkdl.x());
			double buffer = abs( (numerator * denom ) / ( denom*denom + 0.001 ) );
			//double buffer=fabs((1000*(A_l_ext))/((1000*crossProduct.z() - 1000*planarGravity[1]*b_s))*(a_s+b_s));

			/*
			0 white non stimo
			1 red non attendibile
			2 orange sto stimando
			3 green stima
			*/
			if(firstreading>=30){
				if(buffer>5){
					buffer=weight_r_old;
				}
				if(pub_weight_R){


					weight_R.data[0]=buffer;
					if((arm_R_q_r2s(1)<=-0.35 )||(abs(arm_R_q_r2s(2))>=0.50)|| (arm_R_q_s2h(0)<= 0.3)){

						weight_R.data[1]=1; // I send red = the pose needs to be corrected

					}
					else{
						if(abs(weight_r_old-weight_R.data[0])<0.005){
									to_green_right++; //good pose, stay steady
						}
						else{
						to_green_right=0; // AH ! you moved
						}

							if(to_green_right>=5){ weight_R.data[1]=3;} //green = this estimation is valid
							else {weight_R.data[1]=2;} //try again
						}
					weight_r_old=weight_R.data[0];
					weight_R_pub.publish(weight_R);
					}else{
						weight_R.data[0]=0;
						weight_R.data[1]=0;
						weight_R_pub.publish(weight_R); //hand is open, just ignore everything i did
					}
			}
			
			// std::cout<<"A_r_ext: "<<A_r_ext<<std::endl;
			
		}

		{
			KDL::Vector crossProduct=cart_L_hand.p*grav_elbowkdl_left;  //moltiplico per 1000 per evitare approssimazioni a 0
			
			
			double numerator = ((A_l_ext * (a_s + b_s)));
			double denom = (crossProduct.z() + b_s * grav_elbowkdl_left.y() + c_s * grav_elbowkdl_left.x());
			double buffer =abs( (numerator * denom ) / ( denom*denom + 0.001 ) );
			
			if(firstreading>=30){
				if(buffer>5){
					buffer=weight_l_old;
				}
				if(pub_weight_L){


					weight_L.data[0]=buffer;
					if((arm_L_q_r2s(1)>=0.35 )||(abs(arm_L_q_r2s(2))>=0.50 )|| arm_L_q_s2h(0)>-0.3){

						weight_L.data[1]=1; // I send red = the pose needs to be corrected

					}
					else{
						if(abs(weight_l_old-weight_L.data[0])<0.005){
									to_green_left++; //good pose, stay steady
						}
						else{
						to_green_left=0; // AH ! you moved
						}

							if(to_green_left>=5){ weight_L.data[1]=3;} //green = this estimation is valid
							else {weight_L.data[1]=2;} //try again
						}
					weight_l_old=weight_L.data[0];
					weight_L_pub.publish(weight_L);
					}else{
						weight_L.data[0]=0;
						weight_L.data[1]=0;
						weight_L_pub.publish(weight_L); //hand is open, just ignore everything i did
					}
			}
			


		}

		// std::cout<<"\n A_r_ext :"<<A_r_ext<<"\t weight:"<<weight_R.data<<std::endl;

		if((firstreading>=30)&&DATACOLLECT){
		//          [lxSensor,A_l_ext,A_l,bias,lx_m,lx_bias,    rxSensor,A_r_ext,A_r,rx_m,rx_bias,          gravity_world]
		estimations.data[0]=L_force_measured[2];
		estimations.data[1]=A_l_ext;
		estimations.data[2]=A_l;
		estimations.data[3]=weight_L.data[0];
		estimations.data[4]=weightbias_L;
		estimations.data[5]=R_force_measured[2];
		estimations.data[6]=A_r_ext;
		estimations.data[7]=A_r;
		estimations.data[8]=weight_R.data[0];
		estimations.data[9]=weightbias;
		estimations.data[10]=gravity_v.x();
		estimations.data[11]=gravity_v.y();
		estimations.data[12]=gravity_v.z();
		for (int i=0; i<n_joints_subchain; i++)
		{

			lxArm.data[i]=arm_L_q_r2s(i);
			lxArm.data[i+3]=arm_L_q_s2h(i);
			rxArm.data[i]=arm_R_q_r2s(i);
			rxArm.data[i+3]=arm_R_q_s2h(i);

		}

		KDL::Frame rootToHand = cart_L*cart_L_hand;
		lx_hand_pos.vector.x=rootToHand.p.x();
		lx_hand_pos.vector.y=rootToHand.p.y();
		lx_hand_pos.vector.z=rootToHand.p.z();


		rootToHand = cart_R*cart_R_hand;
		rx_hand_pos.vector.x=rootToHand.p.x();
		rx_hand_pos.vector.y=rootToHand.p.y();
		rx_hand_pos.vector.z=rootToHand.p.z();
		debug_rx_hand.publish(rx_hand_pos);
		debug_lx_hand.publish(lx_hand_pos);
		debug_lx_joints.publish(lxArm);
		debug_rx_joints.publish(rxArm);
		debug_estimation.publish(estimations);

		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}


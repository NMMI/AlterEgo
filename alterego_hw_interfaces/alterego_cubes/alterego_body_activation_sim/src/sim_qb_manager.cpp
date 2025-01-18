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
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>
#include <fstream>

using namespace KDL;
using namespace std;
using namespace Eigen;

#define MAX_PRESET 0.6
#define MIN_PRESET 0
#define MAX_HAND_CL 16000


std_msgs::Float64MultiArray 	msg_meas_arm_m1;
std_msgs::Float64MultiArray 	msg_meas_arm_m2;
std_msgs::Float64MultiArray 	msg_meas_arm_shaft;
std_msgs::Float64			 	msg_meas_neck_shaft;
ros::Publisher    				pub_meas_cubes_m1;
ros::Publisher    				pub_meas_cubes_m2;
ros::Publisher    				pub_meas_cubes_shaft;
ros::Publisher    				pub_meas_neck_shaft;

VectorXd 						ref_c_eq(20);
VectorXd 						ref_c_eq_cb(20);
VectorXd 						ref_c_eq_init(20);		// Cube ref LP filter
VectorXd 						ref_c_eq_prec(20); // Cube ref LP filter
VectorXd 						ref_c_pr(20);
VectorXd 						ref_eq(20);
VectorXd 						ref_pr(20);
double 							ref_hand(0);
bool							ref_hand_changed = false;
bool 							first_callback = true;
double 							ref_neck(0);
int 							arm_cubes_n;
double 							offset_hand = 0.0;
std::vector<bool> 				activatedQB{false,false,false,false,false,false};
bool 							activatedNeck=false;
bool 							activatedHand=false;

// IMU Reading Global Variables
int      						n_imu_;
uint8_t* 						imu_table_;
uint8_t* 						mag_cal_;
std::vector<float> 				imu_values_;

// Analog sensors Global Variables
int 							n_adc_sensors_;
std::vector<short int> 			adc_raw_;

/*---------------------------------------------------------------------*
*                                                                      *
*                                                                      *
*----------------------------------------------------------------------*/
void ref_arm_eq__Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
	
	for(int i=0; i<arm_cubes_n; i++){
		if(first_callback)
		{
			ref_c_eq_init(i) = msg->data[i];
		}
		else{
			ref_c_eq_cb(i) = msg->data[i];			// Cube ref LP filter
			ref_c_eq_cb(1) = ref_c_eq_cb(1) - ref_c_eq_init(1);
			ref_c_eq(i) = ref_c_eq_cb(i); 
		}

	}

	first_callback = false;


	//reale
	// ref_c_eq(i) = msg->data[i];
	// ref_c_eq(i) = 0.93 * msg->data[i] + 0.07 * ref_c_eq_prec(i); // Cube ref LP filter
	// ref_c_eq_prec(i) = ref_c_eq(i);								 // Cube ref LP filter


}

/*---------------------------------------------------------------------*
*                                                *
*                                                                      *
*----------------------------------------------------------------------*/
void ref_arm_preset__Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){

	ref_pr(0)= 0.0;

	for(int i=1; i<arm_cubes_n; i++){
		ref_c_pr(i) = msg->data[i];

		if (ref_c_pr(i) > 1)
			ref_c_pr(i) = 1;
		if (ref_c_pr(i) < 0)
			ref_c_pr(i) = 0;


		ref_c_pr(i) = ref_c_pr(i);
	}
}

/*---------------------------------------------------------------------*
*                                                *
*                                                                      *
*----------------------------------------------------------------------*/
void ref_hand__Callback(const std_msgs::Float64::ConstPtr& msg){
	double ref_hand_old = ref_hand;
	double a = log(1.7);
	double b = -0.7;

  // ref_hand = exp(a*msg->data) + b

  ref_hand = msg->data;
	if(ref_hand>1.0) ref_hand = 1.0;
	ref_hand_changed = true;
}

/*---------------------------------------------------------------------*
*                                                                      *
*                                                                      *
*----------------------------------------------------------------------*/
void ref_neck__Callback(const std_msgs::Float64::ConstPtr& msg){
	ref_neck = msg->data;
}


/*---------------------------------------------------------------------*
*                                                *
*                                                                      *
*----------------------------------------------------------------------*/
void compute_cube_motor_position(double eq, double preset, double& th1, double& th2)
{
	if (preset > MAX_PRESET)
		preset = MAX_PRESET;
	if (preset < MIN_PRESET)
		preset = MIN_PRESET;

	//A differenza del reale
	//Non c'è la moltiplicazione * 32768.0 / (2 * M_PI) 
	//Ovvero la conversione da radianti a tick
	th1 = (eq + preset) ;
	th2 = (eq - preset) ;
}

// /*---------------------------------------------------------------------*
// *           meas_arm_m1 is without the neck
// *----------------------------------------------------------------------*/
void motor_1_state__Callback(const sensor_msgs::JointState & msg){
	
	msg_meas_arm_m1.data.clear();
	for(int i=1; i<=arm_cubes_n ; i++){
		msg_meas_arm_m1.data.push_back(msg.position[i]);
	}
	pub_meas_cubes_m1.publish(msg_meas_arm_m1);
	
}
// /*---------------------------------------------------------------------*
// *                                                                      *
// *                                                                      *
// *----------------------------------------------------------------------*/
void motor_2_state__Callback(const sensor_msgs::JointState &  msg){
	
	msg_meas_arm_m2.data.clear();
	for(int i=1; i<=arm_cubes_n ; i++){
		msg_meas_arm_m2.data.push_back(msg.position[i]);
	}
	pub_meas_cubes_m2.publish(msg_meas_arm_m2);
	
}

// /*---------------------------------------------------------------------*
// *                                                                      *
// *                                                                      *
// *----------------------------------------------------------------------*/
void robot_state__Callback(const sensor_msgs::JointState & msg){
    
	msg_meas_arm_shaft.data.clear();
	
	for(int i=0; i<=arm_cubes_n ; i++){
		if(i==0)
			msg_meas_neck_shaft.data = msg.position[i];
			
		else 
			msg_meas_arm_shaft.data.push_back(msg.position[i]);
			
	}
	pub_meas_neck_shaft.publish(msg_meas_neck_shaft);
	pub_meas_cubes_shaft.publish(msg_meas_arm_shaft);

}




/*---------------------------------------------------------------------*
* MAIN                                                                 *
*                                                                      *
*----------------------------------------------------------------------*/
int main(int argc, char **argv)
{   
	string 				port;
	string 				ref_arm_eq_topic;
	string 				ref_arm_preset_topic;
	string 				ref_hand_topic;
	string 				ref_neck_topic;
	string 				meas_arm_ref1_topic;
	string 				meas_arm_ref2_topic;
	string 				meas_arm_m1_topic;
	string 				meas_arm_m2_topic;
	string 				meas_arm_shaft_topic;
	string 				meas_neck_shaft_topic;
	string 				ns;
	string 				acc_hand_topic;
	string 				curr_m1_topic;
	string 				curr_m2_topic;
	string 				pressure_topic;
	string 				motor_1_state_topic;
	string 				motor_2_state_topic;
	string 				robot_state_topic;
	bool				hand_has_old_board = false;
	bool 				vibrotactile_fb_enabled = false;
	bool				pressure_fb_enabled = false;
	
	double 				main_freq(1000);
	double 				body_freq;
	double        		curr_freq;
	double 				hand_move_freq;
	double 				vibro_fb_freq;
	double				pressure_fb_freq;
	int 				loop_cnt = 0;

	int 				baudrate(2000000);
	int					IDhand;
	int					IDneck;
	int 				waiting_nsec(50);

	double 				inputs[2];
	short int 			measurements[3];
	short int 			curr_meas[2];
	bool rigid_shoulder = false;
	
	std::vector<double>	reference_1;
	std::vector<double>	reference_2;


	ros::Subscriber 	sub_ref_arm_eq;
	ros::Subscriber 	sub_ref_arm_preset;
	ros::Subscriber 	sub_ref_hand;
	ros::Subscriber 	sub_ref_neck;
	ros::Subscriber 	sub_motor_1_state;
	ros::Subscriber 	sub_motor_2_state;
	ros::Subscriber 	sub_robot_state;
	
	ros::Publisher    	pub_meas_cubes_ref1;
	ros::Publisher    	pub_meas_cubes_ref2;
	
	std_msgs::Float64MultiArray 	msg_reference_1;
	std_msgs::Float64MultiArray 	msg_reference_2;

	int AlterEgoVersion;
	// current

	bool 				curr_update=true;

	ref_hand = 			offset_hand;

	// ------------------------------------------------------------------------------------- Init Node
	ros::init(argc, argv, "sim_qb_manager");
  	ros::NodeHandle n;
  	ns = ros::this_node::getNamespace();
    
	std::string robot_name = std::getenv("ROBOT_NAME");

	n.getParam("/"+robot_name+"/arm_cubes_n", arm_cubes_n);
	n.getParam("ref_arm_eq_topic", ref_arm_eq_topic);
	n.getParam("ref_arm_preset_topic", ref_arm_preset_topic);
	n.getParam("ref_neck_topic", ref_neck_topic);
	n.getParam("meas_arm_ref1_topic", meas_arm_ref1_topic);
	n.getParam("meas_arm_ref2_topic", meas_arm_ref2_topic);
	n.getParam("meas_arm_m1_topic", meas_arm_m1_topic);
	n.getParam("meas_arm_m2_topic", meas_arm_m2_topic);
	n.getParam("meas_arm_shaft_topic", meas_arm_shaft_topic);
	n.getParam("meas_neck_shaft_topic", meas_neck_shaft_topic);
	n.getParam("motor_1_state_topic", motor_1_state_topic);
	n.getParam("motor_2_state_topic", motor_2_state_topic);
	n.getParam("robot_state_topic", robot_state_topic);
	n.getParam("/"+robot_name+"/vibrotactile_fb_enabled", vibrotactile_fb_enabled);
	n.getParam("/"+robot_name+"/pressure_fb_enabled", pressure_fb_enabled);
	n.getParam("/"+robot_name+"/rigid_shoulder", rigid_shoulder);
	n.getParam("/"+robot_name+"/AlterEgoVersion", AlterEgoVersion);
	// Frequencies settings
	ros::Rate loop_rate(main_freq);				// Run node at max. frequency [1000 Hz default]

	// All frequencies must be sub-multiples of main_freq
	curr_freq = 50;								// Default
	n.getParam("/current_frequency", curr_freq);	// Override if configured
	curr_freq = (curr_freq > main_freq)?main_freq:curr_freq;	// Limit body frequency to main frequency
	

	body_freq = 50;								// Default
	n.getParam("/body_frequency", body_freq);	// Override if configured
	body_freq = (body_freq > main_freq)?main_freq:body_freq;	// Limit body frequency to main frequency

	hand_move_freq = body_freq;							// Default
	n.getParam("/hand_move_frequency", hand_move_freq);	// Override if configured
	hand_move_freq = (hand_move_freq > body_freq)?body_freq:hand_move_freq;		// Limit hand frequency to body frequency

	vibro_fb_freq = main_freq;								// Default
	n.getParam("/vibrotactile_fb_frequency", vibro_fb_freq);	// Override if configured
	vibro_fb_freq = (vibro_fb_freq > main_freq)?main_freq:vibro_fb_freq;		// Limit vibro_fb frequency to main frequency

	pressure_fb_freq = main_freq;								// Default
	n.getParam("/pressure_fb_frequency", pressure_fb_freq);	// Override if configured
	pressure_fb_freq = (pressure_fb_freq > main_freq)?main_freq:pressure_fb_freq;		// Limit pressure_fb frequency to main frequency


	// --- node param ---
	
	// ------------------------------------------------------------------------------------- Subscribe to topics
	sub_ref_arm_eq				= n.subscribe(ref_arm_eq_topic, 1, ref_arm_eq__Callback);
	sub_ref_arm_preset			= n.subscribe(ref_arm_preset_topic, 1, ref_arm_preset__Callback);
	sub_ref_neck				= n.subscribe("/" + robot_name + ref_neck_topic, 1, ref_neck__Callback);
	sub_motor_1_state			= n.subscribe(motor_1_state_topic, 1, motor_1_state__Callback);
	sub_motor_2_state			= n.subscribe(motor_2_state_topic, 1, motor_2_state__Callback);
	sub_robot_state				= n.subscribe(robot_state_topic, 1, robot_state__Callback);
	


	// ------------------------------------------------------------------------------------- Published topics 
	pub_meas_cubes_ref1			= n.advertise<std_msgs::Float64MultiArray>(meas_arm_ref1_topic, 1);
	pub_meas_cubes_ref2			= n.advertise<std_msgs::Float64MultiArray>(meas_arm_ref2_topic, 1);
	pub_meas_cubes_m1			= n.advertise<std_msgs::Float64MultiArray>(meas_arm_m1_topic, 1);
	pub_meas_cubes_m2			= n.advertise<std_msgs::Float64MultiArray>(meas_arm_m2_topic, 1);
	pub_meas_cubes_shaft		= n.advertise<std_msgs::Float64MultiArray>(meas_arm_shaft_topic, 1);
	pub_meas_neck_shaft			= n.advertise<std_msgs::Float64>(meas_neck_shaft_topic, 1);

	ref_c_eq.resize(arm_cubes_n + 1);
	ref_c_eq_init.resize(arm_cubes_n + 1);
	ref_c_eq_cb.resize(arm_cubes_n + 1);
	for(int i=0; i<arm_cubes_n+1; i++){
		ref_c_eq(i) = 0;
		ref_neck = 0;
		ref_c_eq_init(i) = 0;
		ref_c_eq_cb(i) = 0;
		ref_c_pr(i) = 0;
	}
	
	// std::cout<<"arm_cubes_n: "<<arm_cubes_n<<endl;
	// std::cout<<"robot_state_topic: "<<robot_state_topic<<endl;
	// std::cout<<"AlterEgoVersion: "<<AlterEgoVersion<<endl;

  	// ------------------------------------------------------------------------------------- MAIN LOOP 
  	while (ros::ok())
  	{
  		// BODY
		if (loop_cnt % (int)(main_freq/body_freq) == 0 ) {		// This part is executed at body_freq frequency
			
			// --- Clear old Measurements ---
			msg_reference_1.data.clear();
			msg_reference_2.data.clear();

			// Msg ref1 e ref2 hanno i dati del cubo del collo e dalla spalla addon fino alla mano
			//Set input to the neck 
			compute_cube_motor_position(ref_neck, 0.3, inputs[0], inputs[1]);
			msg_reference_1.data.push_back(inputs[0]);
			msg_reference_2.data.push_back(inputs[1]);


			// -- Set References ---
			ref_eq = ref_c_eq; // Copy ref values from last callback (so that arm SetInputs is atomic and coherent)
			ref_pr = ref_c_pr;

			if((AlterEgoVersion == 2)||(AlterEgoVersion == 4) ){
				// -- Set input for the rest of the arm ---
				for(int i=0; i<arm_cubes_n; i++){
					compute_cube_motor_position(ref_eq(i), ref_pr(i), inputs[0], inputs[1]);
					msg_reference_1.data.push_back(inputs[0]);
					msg_reference_2.data.push_back(inputs[1]);
				}
			}
			else if(AlterEgoVersion == 3 ){
				// -- Set input for rigid shoulder in radianti a diifferenza del reale che li vuole in tick --
				inputs[0] = (short int)(ref_eq(0));
				inputs[1] = 0;
				msg_reference_1.data.push_back(inputs[0]);
				msg_reference_2.data.push_back(inputs[1]);

				// -- Set input for the rest of the arm ---
				for(int i=1; i<arm_cubes_n; i++){
					compute_cube_motor_position(ref_eq(i), ref_pr(i), inputs[0], inputs[1]);
					msg_reference_1.data.push_back(inputs[0]);
					msg_reference_2.data.push_back(inputs[1]);
				}
			}

			
			
		



			//Compute reference_1 and reference_2 
			pub_meas_cubes_ref1.publish(msg_reference_1);
			pub_meas_cubes_ref2.publish(msg_reference_2);

		}

			// --- cycle ---
			
  		ros::spinOnce();
    	loop_rate.sleep();


    	// Update loop counter
    	loop_cnt++;
    	if (loop_cnt > main_freq) {
    		loop_cnt -= main_freq;
    	}


  	}
	
}
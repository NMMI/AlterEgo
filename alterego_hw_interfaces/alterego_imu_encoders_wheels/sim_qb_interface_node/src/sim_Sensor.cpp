
#include "sim_Sensor.h"

//-----------------------------------------------------
//                                         qb_class
//-----------------------------------------------------

/*
/ *****************************************************
/ Costructor of qb_class class
/ *****************************************************
/   parameters:
/				port - usb port tfo activate 
/					   communication
/   return:
/
*/

Sensor::Sensor(){

	// ------------------------------------------------------------------------------------- Init Node
	ns = ros::this_node::getNamespace();

	// ------------------------------------------------------------------------------------- Subscribe to topics
  	node_.getParam("joint_states_topic", joint_states_topic);			 //encoders
	node_.getParam("gazebo_link_state_topic", gazebo_link_state_topic);  //pitch rate
	node_.getParam("gazebo_imu_plugin_topic", gazebo_imu_plugin_topic);  //pitch

	sub_joint_states	 					= node_.subscribe(joint_states_topic, 1, &Sensor::read_joint_states__Callback, this);
	sub_gazebo_imu_plugin					= node_.subscribe(gazebo_imu_plugin_topic, 1, &Sensor::read_gazebo_imu_plugin__Callback, this);
	sub_gazebo_link_state					= node_.subscribe(gazebo_link_state_topic, 1, &Sensor::read_gazebo_link_state__Callback, this);

	// ------------------------------------------------------------------------------------- Published topics 
	node_.getParam("wheels/N_", N_);											//rapporto di riduzione
  	node_.getParam("cube_measurement_topic", cube_measurement_topic);   //encoders
	node_.getParam("gyro_good_topic", gyro_good_topic);					//pitch rate
	node_.getParam("RPY_topic", RPY_topic);								//pitch

	pub_cube_measurement 					= node_.advertise<qb_interface::cubePos>(cube_measurement_topic, 1);
	pub_gyro_good			  				= node_.advertise<geometry_msgs::Vector3>(gyro_good_topic, 1);
	pub_RPY			        				= node_.advertise<geometry_msgs::Vector3>(RPY_topic, 1);

	// ------------------------------------------------------------------------------------- Init params 
	
	
	flag_run1_ = false;
	flag_run2_ = false;
	N_ = 1/N_;

	pos_R_js = 0.0;
	pos_L_js = 0.0;
	

	msg_gyro_good.x = 0.0;
	msg_gyro_good.y = 0.0;
	msg_gyro_good.z = 0.0;

	msg_RPY.x = 0.0;
	msg_RPY.y = 0.0;
	msg_RPY.z = 0.0;

}

//-----------------------------------------------------
//                                            ~qb_class
//-----------------------------------------------------

/*
/ *****************************************************
/ Descructor of qb_class class
/ *****************************************************
/   parameters:
/   return:
/
*/


Sensor::~Sensor(){

}



/*---------------------------------------------------------------------*
*                                                *
*                                                                      *
*----------------------------------------------------------------------*/
void Sensor::read_gazebo_imu_plugin__Callback(const sensor_msgs::Imu& msg){

      
    msg_gyro_good.x = msg.angular_velocity.x;
    msg_gyro_good.y = msg.angular_velocity.y;
    msg_gyro_good.z = msg.angular_velocity.z;


}
/*---------------------------------------------------------------------*
*                                                
*                                                                      
*----------------------------------------------------------------------*/
void Sensor::read_gazebo_link_state__Callback(const gazebo_msgs::LinkStates & msg){
	
  	const std::vector<std::string> &names = msg.name;
	
	for(int i= 0; i<names.size(); i++)
	{
		if(names[i] == "ego_robot::base_link")
		{
		KDL::Rotation::Quaternion(msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z, msg.pose[i].orientation.w).GetRPY(euler_(0), euler_(1), euler_(2));
		}
	}

	msg_RPY.x = euler_(0);
	msg_RPY.y = euler_(1);
	msg_RPY.z = euler_(2);
}

/*
/ *****************************************************
/ Read position of all motors
/ *****************************************************
/   parameters:
/   return:
/               [state]
/
*/

void Sensor::read_joint_states__Callback(const sensor_msgs::JointState & msg){

  	const std::vector<std::string> &names = msg.name;

	//Leggo i joint_states in radianti e li converto in tick
	
	//---------------------------------converto da radianti a tick
	for(int i= 0; i<names.size(); i++)
	{

		if(names[i] == "base_link_to_wheel_R")
		{
		//RAD2TICK
		pos_R_js = (msg.position[i])* N_; 											//moltiplico per il rapporto di riduzione
		
		pos_R_js = int(pos_R_js * 65536 / (4.0 * PI));								//converto in tick
  		// std::cout<< "1 Right wheel rad from js " << msg.position[i] << std::endl;
  		// std::cout<< "2 Right wheel rad from js wrt to N_ " << msg.position[i]* N_ << std::endl;

		}
		if(names[i] == "base_link_to_wheel_L")
		{
		//RAD2TICK	
		pos_L_js = (msg.position[i])* N_;											//moltiplico per il rapporto di riduzione
		pos_L_js = int(pos_L_js * 65536 / (4.0 * PI));								//converto in tick
		}
	}
	//---------------------------------Saturo i tick nel range [-2^15,2^15]
	//Threshold
	th = pow(2.0,15.0);
  	// std::cout<< "3 Right wheel tick from js " << pos_R_js<< std::endl;

	//RIGHT     
	//Sign check
	if(pos_R_js>0) sign = 1;
	if(pos_R_js<0) sign = -1;
	q_R = int(abs(pos_R_js)/th);
	r_R = abs(pos_R_js) - q_R*th;
	//dispari
	if((q_R%2) != 0){
		// pos_R_tk = vector<float>((r_R - th)*sign);
		pos_R_js = (r_R - th)*sign;
	}
	else{
		// pos_R_tk = vector<float>(r_R*sign) ;
		pos_R_js = r_R*sign ;
	}

	//LEFT
	if(pos_L_js>0) sign = 1;
	if(pos_L_js<0) sign = -1;
	q_L = int(abs(pos_L_js)/th);
	r_L = abs(pos_L_js) - q_L*th;

	if((q_L%2) != 0){
		pos_L_js = (r_L - th)*sign;
	}
	else{
		pos_L_js = r_L*sign ;
	}




}

//-----------------------------------------------------
//                                                 spin
//-----------------------------------------------------

/*
/ *****************************************************
/ Read all devices and set position if new ref. is
/ arrived.
/ *****************************************************
/   parameters:
/   return:
/
*/

void Sensor::run(){

	//---------------------------------converto gli encoders rispetto al rapporto di riduzione 
	cube_measurement_msg.p_1.push_back(-pos_L_js); 						//lo spostamento degli'encoder è opposto alla realtà
	cube_measurement_msg.p_2.push_back(-pos_R_js);
	cube_measurement_msg.p_L.push_back(0);
  	// std::cout<< "4 Right wheel wrt to N_ " << pos_R_js<< std::endl;



	//---------------------------------pubblico i valori sul topic

	pub_cube_measurement.publish(cube_measurement_msg);  	//encoders
	pub_RPY.publish(msg_RPY);								//pitch 
    pub_gyro_good.publish(msg_gyro_good);					//pitch rate


	//clean encoders data
	cube_measurement_msg.p_1.clear();
	cube_measurement_msg.p_2.clear();
	cube_measurement_msg.p_L.clear();

}



#include "sim_qb_class_wheels.h"

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

sim_qb_class_wheels::sim_qb_class_wheels(){

	// ------------------------------------------------------------------------------------- Init Node
	ns = ros::this_node::getNamespace();

	// ------------------------------------------------------------------------------------- Subscribe to topics
  	node_.getParam("cube_ref_topic", cube_ref_topic);															//command from LQR

	sub_cube_ref = node_.subscribe(cube_ref_topic, 10, &sim_qb_class_wheels::read_cube_ref__Callback, this);

	// ------------------------------------------------------------------------------------- Published topics 
  	node_.getParam("right_wheel_command_topic", right_wheel_command_topic);										//command to right wheel	
  	node_.getParam("left_wheel_command_topic", left_wheel_command_topic);										//command to left wheel

	pub_right_wheel_command = node_.advertise<std_msgs::Float64>(right_wheel_command_topic, 1);
	pub_left_wheel_command	= node_.advertise<std_msgs::Float64>(left_wheel_command_topic, 1);

	flag_run1_ = false;
	flag_run2_ = false;
	

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


sim_qb_class_wheels::~sim_qb_class_wheels(){

}




/*---------------------------------------------------------------------*
*
*    qb_class/cube_ref is the command topic for the effortcontrollers
*    input: command in voltage wrt to the PWM_RANGE
*    output: command in Nm
*
*----------------------------------------------------------------------*/
void sim_qb_class_wheels::read_cube_ref__Callback(const qb_interface::cubeRef::ConstPtr &msg){


	com_L = (msg->p_1[0] / PWM_EXT_RANGE_K);
	com_R = (msg->p_2[0] / PWM_EXT_RANGE_K);

	command_L = (n*0.24) * Kt/Rm* (com_L )*10.8/68.6;
	command_R = (n*0.24) * Kt/Rm* (com_R )*10.8/68.6;
	
	
	if(command_L > 11)
		command_L = 11;
	else if(command_L< -11)
		command_L = -11;
		
	if(command_R > 11)
		command_R = 11;
	else if(command_R< -11)
		command_R = -11;



    // ROS_INFO_STREAM("Left Torque: " << command_L << " (Nm)" << " Right Torque: " << command_R<< " (Nm) " );




  
}

//-----------------------------------------------------
//                                                 spin
//-----------------------------------------------------


void sim_qb_class_wheels::run(){


	command_L_msg.data = command_L;
	command_R_msg.data = command_R;
	pub_left_wheel_command.publish(command_L_msg);
	pub_right_wheel_command.publish(command_R_msg);

}


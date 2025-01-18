#include <arms_compliant_control.h>


// Constructor
ArmsCompliantControl::ArmsCompliantControl()
{
	std::string robot_name = std::getenv("ROBOT_NAME");
  std::string ns = ros::this_node::getName();   
	nh.getParam("/"+robot_name+"/R_OUT_topic_button_A", R_OUT_topic_button_A);

  if (ns.find("left") != std::string::npos)
  {
	  nh.getParam("/"+robot_name+"/L_OUT_topic_track", IN_topic_track);
    arm_side = "left";
  }
  else if(ns.find("right") != std::string::npos)
  {
	  nh.getParam("/"+robot_name+"/R_OUT_topic_track", IN_topic_track);
    arm_side = "right";
  }

	nh.getParam("/"+robot_name+"/IN_topic_arms_compliant_control", IN_topic_arms_compliant_control);

  pub_posture		= nh.advertise<geometry_msgs::Pose>("/"+robot_name+"/"+IN_topic_track, 1);
  sub_Botton_A_R = nh.subscribe("/"+robot_name+"/"+R_OUT_topic_button_A, 1, &ArmsCompliantControl::R_Botton_A__Callback,this);				// BUTTON 1

  enable_compliant = 1;
}

// Deconstructor:
ArmsCompliantControl::~ArmsCompliantControl()
{

}


// ---------------------------------------------------------------------------------------------------
// Funzione per vedere se c'è un pilota
// ---------------------------------------------------------------------------------------------------

void ArmsCompliantControl::R_Botton_A__Callback(const std_msgs::Bool::ConstPtr &msg)
{
  static int last_msg(0);

    if (msg->data == true)
    {
      enable_compliant = 0;
      // std::cout <<arm_side<< " compliant control disabled" << std::endl;
    }
    else
    {
      enable_compliant = 1;
      // std::cout << arm_side<< " compliant control enabled" << std::endl;
    }


}


// ---------------------------------------------------------------------------------------------------
// Funzione per vedere se viene mosso il braccio destro 
// ---------------------------------------------------------------------------------------------------

geometry_msgs::Pose ArmsCompliantControl::getArmPosition()
{

  try {
    listener.waitForTransform("torso", arm_side+"_hand_curr", ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform("torso", arm_side+"_hand_curr",ros::Time(0), transform);
  } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
  }
  // position = tf::Vector3( transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
  // orientation = transform.getRotation();
  robot_pose.position.x = transform.getOrigin().x();
	robot_pose.position.y = transform.getOrigin().y();
	robot_pose.position.z = transform.getOrigin().z();
	robot_pose.orientation.x = transform.getRotation().x();
	robot_pose.orientation.y = transform.getRotation().y();
	robot_pose.orientation.z = transform.getRotation().z();
	robot_pose.orientation.w = transform.getRotation().w();
  return robot_pose;
}




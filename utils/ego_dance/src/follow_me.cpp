#include <follow_me.h>

// Constructor
FollowMe::FollowMe()
{
	std::string robot_name = std::getenv("ROBOT_NAME");
  std::string ns = ros::this_node::getName();   
  if (ns.find("left") != std::string::npos)
  {
    arm_side = "left";
  }
  else if(ns.find("right") != std::string::npos)
  {
    arm_side = "right";
  }
  // load the gain:
  lambda_1 = 0.0;
  lambda_2 = 0.0;

  nh.getParam("gain_controllers/lambda_1", lambda_1);
  nh.getParam("gain_controllers/lambda_2", lambda_2);
  nh.getParam("/"+robot_name+"/BASE_IN_VEL_topic", BASE_IN_VEL_topic);
  if(!nh.hasParam("gain_controllers/lambda_1") || !nh.hasParam("gain_controllers/lambda_2"))
  {
    ROS_ERROR("Params not correctly loaded");
  }
  // Topic you want to publish:
  pub_des_vel = nh.advertise<geometry_msgs::TwistStamped>("/"+robot_name+"/"+BASE_IN_VEL_topic, 1);


  P0.position.x = 0.0;
  P0.position.y = 0.0;
  P0.position.z = 0.0;
  P0.orientation.x = 0.0;
  P0.orientation.y = 0.0;
  P0.orientation.z = 0.0;
  P0.orientation.w = 0.0;


  Pt.position.x = 0.0;
  Pt.position.y = 0.0;
  Pt.position.z = 0.0;
  Pt.orientation.x = 0.0;
  Pt.orientation.y = 0.0;
  Pt.orientation.z = 0.0;
  Pt.orientation.w = 0.0;
  

  control_law.twist.linear.x =  0;
  control_law.twist.linear.y =  0;
  control_law.twist.linear.z =  0;
  control_law.twist.angular.x = 0;
  control_law.twist.angular.y = 0;


  R_ = 0.125;
  v = 0;
  w = 0;
}

// Deconstructor:

FollowMe::~FollowMe()
{
}


// ---------------------------------------------------------------------------------------------------
// Funzione per calcolare la norma
// ---------------------------------------------------------------------------------------------------

template<typename Iter_T>
long double vectorNorm(Iter_T first, Iter_T last) {
  return sqrt(std::inner_product(first, last, first, 0.0L));
}

// ---------------------------------------------------------------------------------------------------
// Funzione per calcolare la posizione del braccio 
// ---------------------------------------------------------------------------------------------------

geometry_msgs::Pose FollowMe::getArmPosition()
{

  try {
    listener.waitForTransform("torso", arm_side+"_hand_curr", ros::Time(0), ros::Duration(0.5) );
    listener.lookupTransform("torso", arm_side+"_hand_curr",ros::Time(0), transform);
  } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
  }
  hand_pose.position.x = transform.getOrigin().x();
	hand_pose.position.y = transform.getOrigin().y();
	hand_pose.position.z = transform.getOrigin().z();
	hand_pose.orientation.x = transform.getRotation().x();
	hand_pose.orientation.y = transform.getRotation().y();
	hand_pose.orientation.z = transform.getRotation().z();
	hand_pose.orientation.w = transform.getRotation().w();
  return hand_pose;
}


// ---------------------------------------------------------------------------------------------------
// Funzione per calcolare l'errore dalla posizione desiderata a quella iniziale
// ---------------------------------------------------------------------------------------------------
geometry_msgs::Point FollowMe::computeError(geometry_msgs::Point p0, geometry_msgs::Point pt)
{
  geometry_msgs::Point delta_p;

  delta_p.x = pt.x-p0.x;
  delta_p.y = pt.y-p0.y;
  delta_p.z = pt.z-p0.z;

  delta_e_norm = sqrt(pow(delta_p.x,2) + pow(delta_p.y,2) + pow(delta_p.z,2));
  return delta_p;
}


// ---------------------------------------------------------------------------------------------------
// Funzione per calcolare l'angolo di orientamento del braccios
// ---------------------------------------------------------------------------------------------------
double FollowMe::computeOrientation(geometry_msgs::Point delta_p)
{
  double delta_phi;

  delta_phi = atan2(delta_p.y, delta_p.x);

  return delta_phi;
}


// ---------------------------------------------------------------------------------------------------
// Funzione per calcolare la velocita lineare e angolare da imporre al robot
// ---------------------------------------------------------------------------------------------------
void FollowMe::run(double delta_phi, geometry_msgs::Point delta_p )
{
  // Linear Velocity Saturation--------------------------------
  if(delta_e_norm > 0.05){                    //Se norma maggiore di 5cm calcola vel

    v = lambda_1 * delta_p.x;
    v = (1 - 0.005) * v_old + 0.005 * v;
    v_old = v;
  }
  else if(fabs(v)>0.005)
  {
    //devo portare il valore a zero 
    v = v_old - (0.005 * v);
    v_old = v;
  }
  else{
    v_old = 0;
    v = 0;
  }

  // Angular Rate Saturation-----------------------------------------
  if(delta_e_norm > 0.05 && fabs(delta_phi)< 1.57){                      //Se norma maggiore di 5cm calcola vel

    w = lambda_2 * delta_phi;
    w = (1 - 0.005) * w_old + 0.005 * w;
    w_old = w;
  }
  else if(fabs(w)>0.005)                        //Se norma minore di 5cm calcola vel -> porta a 0 gradualmente la vel fino a un valore di 0.005 (trovare unita di misura)
  {
    w = w_old - (0.005 * w);
    w_old = w;
  }
  else{                                         //Altrimenti porta a 0
    w_old = 0;
    w = 0;
  }
  // Publish desired velocities:
  control_law.twist.linear.x =  v;
  control_law.twist.angular.z = w;
  
  pub_des_vel.publish(control_law);
}
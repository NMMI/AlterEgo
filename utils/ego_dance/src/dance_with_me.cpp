#include <dance_with_me.h>

// Constructor
DanceWithMe::DanceWithMe()
{
	std::string robot_name = std::getenv("ROBOT_NAME");
  std::string ns = ros::this_node::getName();   
 
  // load the gain:
  lambda_1 = 0.0;
  lambda_2 = 0.0;

  nh.getParam("gain_controllers/follow_me/lambda_1", lambda_1);
  nh.getParam("gain_controllers/follow_me/lambda_2", lambda_2);
  nh.getParam("/"+robot_name+"/CMD_VEL_IN_topic", CMD_VEL_IN_topic);

  std::vector<std::string> param_names = {"lambda_1", "lambda_2"};
  for (const std::string& param_name : param_names) {
    if (!nh.hasParam("gain_controllers/follow_me/" + param_name)) {
      ROS_ERROR_STREAM("Params not correctly loaded: " << param_name);
      return;
      
    }
  }
  // Topic you want to publish:
  pub_des_vel = nh.advertise<geometry_msgs::Twist>("/"+robot_name+"/"+CMD_VEL_IN_topic, 1);


  P0_right.position.x = 0.0;
  P0_right.position.y = 0.0;
  P0_right.position.z = 0.0;
  P0_right.orientation.x = 0.0;
  P0_right.orientation.y = 0.0;
  P0_right.orientation.z = 0.0;
  P0_right.orientation.w = 0.0;

  P0_left.position.x = 0.0;
  P0_left.position.y = 0.0;
  P0_left.position.z = 0.0;
  P0_left.orientation.x = 0.0;
  P0_left.orientation.y = 0.0;
  P0_left.orientation.z = 0.0;
  P0_left.orientation.w = 0.0;


  Pt_left.position.x = 0.0;
  Pt_left.position.y = 0.0;
  Pt_left.position.z = 0.0;
  Pt_left.orientation.x = 0.0;
  Pt_left.orientation.y = 0.0;
  Pt_left.orientation.z = 0.0;
  Pt_left.orientation.w = 0.0;
  
  Pt_right.position.x = 0.0;
  Pt_right.position.y = 0.0;
  Pt_right.position.z = 0.0;
  Pt_right.orientation.x = 0.0;
  Pt_right.orientation.y = 0.0;
  Pt_right.orientation.z = 0.0;
  Pt_right.orientation.w = 0.0;

  e_left.x = 0;
  e_left.y = 0;
  e_left.z = 0;
  control_law.linear.x =  0;
  control_law.linear.y =  0;
  control_law.linear.z =  0;
  control_law.angular.x = 0;
  control_law.angular.y = 0;

  
  v = 0;
  w = 0;
}

// Deconstructor:

DanceWithMe::~DanceWithMe()
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

geometry_msgs::Pose DanceWithMe::getArmPosition(std::string arm_side)
{

  try {
    listener.waitForTransform("torso", arm_side+"_hand_curr", ros::Time(0), ros::Duration(5) );
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
geometry_msgs::Point DanceWithMe::computeError(geometry_msgs::Point p0, geometry_msgs::Point pt)
{
  geometry_msgs::Point delta_p;

  delta_p.x = pt.x-p0.x;
  delta_p.y = pt.y-p0.y;
  delta_p.z = pt.z-p0.z;

  return delta_p;
}


// ---------------------------------------------------------------------------------------------------
// Funzione per calcolare l'angolo di orientamento del braccio
// ---------------------------------------------------------------------------------------------------
double DanceWithMe::computeOrientation(geometry_msgs::Point delta_p)
{
  double delta_phi;

  delta_phi = atan2(delta_p.y, delta_p.x);

  return delta_phi;
}


// ---------------------------------------------------------------------------------------------------
// Funzione per calcolare la velocita lineare e angolare da imporre al robot
// ---------------------------------------------------------------------------------------------------
void DanceWithMe::run(double delta_phi, geometry_msgs::Point delta_p, double e_norm)
{
  // Linear Velocity Saturation--------------------------------
  if(e_norm > 0.05){                    //Se norma maggiore di 5cm calcola vel

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
  if(e_norm > 0.05 && fabs(delta_phi)< 1.57){                      //Se norma maggiore di 5cm calcola vel

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
  control_law.linear.x =  v;
  control_law.angular.z = w;
  
  pub_des_vel.publish(control_law);
}
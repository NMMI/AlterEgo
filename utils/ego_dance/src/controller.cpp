#include <ego_dance.h>

// ----------------------------------------------------------------------------------------------------------
// Function to calculate everytime the position (x,y) of the unicycle (mobile base)
// ----------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------
// Callback Functions to acquire the element of vector Q, to update it.
// ----------------------------------------------------------------------------------------------------------
void EgoDance::callback_upperbody_state(const alterego_msgs::UpperBodyState::ConstPtr &msg)
{
    for (int i = 0; i < body_names_.size(); i++)
    {
        if (body_names_[i] == "neck")
            Q[i] = (double)msg->left_meas_neck_shaft;

        if (body_names_[i] == "neck_cube")
            Q[i] = (double)msg->right_meas_neck_shaft;

        if (body_names_[i] == "left_shoulder_flange")
        {
            for (int j = 0; j < msg->left_meas_arm_shaft.size(); j++)
            {
                Q[i + j] = (double)msg->left_meas_arm_shaft[j];
            }
        }

        if (body_names_[i] == "right_shoulder_flange")
        {
            for (int j = 0; j < msg->right_meas_arm_shaft.size(); j++)
            {
                Q[i + j] = (double)msg->right_meas_arm_shaft[j];
            }
        }
    }

    acquire_meas_base_real = 1;
    acquire_meas_arm_l = 1;
    acquire_meas_arm_r = 1;
    // Stampa di debug per il vettore Q
    std::cout << "Vettore Q: ";
    for (int i = 0; i < Q.size(); i++)
    {
        std::cout << Q[i] << " ";
    }
    std::cout << std::endl;
}
  // if (AlterEgoVersion == 2)
  // {

  //   for (int i = 0; i < msg->left_meas_arm_shaft.size(); i++)
  //   {
  //     Q[4 + i] = (double)msg->left_meas_arm_shaft[i];
  //   }

  //   for (int i = 0; i < msg->right_meas_arm_shaft.size(); i++)
  //   {
  //     Q[11 + i] = (double)msg->right_meas_arm_shaft[i];
  //   }

  //   acquire_meas_base_real = 1;
  //   acquire_meas_arm_l = 1;
  //   acquire_meas_arm_r = 1;
  // }
  // else if (AlterEgoVersion == 3)
  // {

  //   for (int i = 0; i < msg->left_meas_arm_shaft.size(); i++)
  //   {
  //     Q[8 + i] = (double)msg->left_meas_arm_shaft[i];
  //   }

  //   for (int i = 0; i < msg->right_meas_arm_shaft.size(); i++)
  //   {
  //     Q[16 + i] = (double)msg->right_meas_arm_shaft[i];
  //   }

  //   acquire_meas_base_real = 1;
  //   acquire_meas_arm_l = 1;
  //   acquire_meas_arm_r = 1;
  // }


void EgoDance::callback_lowerbody_state (const alterego_msgs::LowerBodyState::ConstPtr &msg)
{

  
  // Q[0] = (double)msg->mobile_base_pos_x;
  // Q[1] = (double)msg->mobile_base_pos_y;
  // Q[2] = (double)msg->yaw_angle;
  // Q[3] = (double)msg->pitch_angle;

  Q[0] = 0.0;
  Q[1] = 0.0;
  Q[2] = 0.0;
  Q[3] = 0.0;
  acquire_meas_base_real = 1;


}

// Constructor
EgoDance::EgoDance()
{
  std::string robot_name = std::getenv("ROBOT_NAME");
  model = new Model();
  nh.getParam("AlterEgoVersion", AlterEgoVersion);

  // load the gain:

  if (nh.getParam("gain_controllers/dance/lambda_1", lambda_1)) 
  nh.getParam("gain_controllers/dance/lambda_2", lambda_2);
  nh.getParam("gain_controllers/dance/lambda_3", lambda_3);
  nh.getParam("gain_controllers/dance/lambda_4", lambda_4);
  nh.getParam("/"+robot_name+"/CMD_VEL_IN_topic", CMD_VEL_IN_topic);

  std::vector<std::string> param_names = {"lambda_1", "lambda_2", "lambda_3", "lambda_4"};
  for (const std::string& param_name : param_names) {
    if (!nh.hasParam("gain_controllers/dance/" + param_name)) {
      ROS_ERROR_STREAM("Params not correctly loaded: " << param_name);
      return;
      
    }
  }
  // Topic you want to publish:
  controller = nh.advertise<geometry_msgs::Twist>("/"+robot_name+"/"+CMD_VEL_IN_topic, 1);

  // rosbag_1 = nh.advertise<geometry_msgs::Pose>("/data", 1);
  // rosbag_2 = nh.advertise<geometry_msgs::Pose>("/hand_pose", 1);
  // rosbag_3 = nh.advertise<geometry_msgs::Pose>("/unicycle_pose", 1);
  // rosbag_4 = nh.advertise<geometry_msgs::Pose>("/p_medio", 1);

  std::cout<<"AlterEgoVersion: "<<AlterEgoVersion<<std::endl;
  std::string path = ros::package::getPath("ego_dance")+"/urdf/ego_robot_gazebo_v"+std::to_string(AlterEgoVersion)+".urdf";
  std::cout<<path<<std::endl;

  const char *path_char = path.c_str();

  // Topic you want to subscribe

  sub_upperbody_state = nh.subscribe("/"+robot_name+"/alterego_state/upperbody",1, &EgoDance::callback_upperbody_state, this);
  sub_lowerbody_state = nh.subscribe("/"+robot_name+"/alterego_state/lowerbody",1, &EgoDance::callback_lowerbody_state, this);

  // ----------------------------------------------------------------------------------------------------------

  // load the model

  if (!Addons::URDFReadFromFile(path_char, model, false, false))
  {
    std::cerr << "Error loading model ./onearm_ego.xacro" << std::endl;
  }

  // abort();

  // initialize variables:

  // Unicycle's center:

  q = Eigen::VectorXd::Zero(2);   // global frame
  q_l = Eigen::VectorXd::Zero(2); // local frame

  // Hand_Target 1 (DX):

  Pt1 = Eigen::VectorXd::Zero(2);   // global frame
  Pt1_l = Eigen::VectorXd::Zero(2); // local frame

  Pt1_t = Eigen::VectorXd::Zero(2); // shifted target point

  // Hand_Target 2 (SX):

  Pt2 = Eigen::VectorXd::Zero(2);   // global frame
  Pt2_l = Eigen::VectorXd::Zero(2); // local frame

  Pt2_t = Eigen::VectorXd::Zero(2); // shifted target point

  // to store distance from the hands to the unicycle center at time t, in local reference frame :

  delta1_l = Eigen::VectorXd::Zero(2);
  delta2_l = Eigen::VectorXd::Zero(2);
  delta_l = Eigen::VectorXd::Zero(2);

  // to store, at initial time, distance from the hands to the unicycle center at time t, in local reference frame :

  delta1_l_init = Eigen::VectorXd::Zero(2);
  delta2_l_init = Eigen::VectorXd::Zero(2);

  arm_L_q_addon_init = 0.0;
  arm_R_q_addon_init = 0.0;

  R_ = 0.125;
  Pm_l = Eigen::VectorXd::Zero(2);

  // controller: ----------------------------------------------------

  v = 0.0;
  omega = 0.0;
  omega_old = 0.0;
  v_old = 0.0;
  linear_vel_real = 0.0;
  omega_real = 0.0;

  Q = VectorNd::Zero(model->q_size); // vector of the 24 joint variables: x, y, yaw, pitch, front_kickstand[2], back_kickstand[2], L_wheel, R_wheel, neck[2], L_arm[6], R_arm[6]
  // if Alterego version = 3: vector of the 24 joint variables: x, y, yaw, pitch, front_kickstand[2], back_kickstand[2], L_wheel, R_wheel, neck[2], L_arm[6], R_arm[6]
  // if Alterego version = 2: vector of the 18 joint variables: x, y, yaw, pitch, L_wheel, R_wheel, neck[2], L_arm[5], R_arm[5]

  // Urdf Debug -------------------------------------------------------------

  if (VERBOSE)
    std::cout << "Q_size = " << model->q_size << std::endl;
  if (VERBOSE)
    std::cout << Utils::GetModelDOFOverview(*model) << std::endl;
  if (VERBOSE)
    std::cout << Utils::GetModelHierarchy(*model) << std::endl;

  //initialize the body names
  body_names_.clear();

  for (unsigned int body_id = 1; body_id < model->mBodies.size(); ++body_id)
  {    
    body_names_.push_back(model->GetBodyName(body_id));
  }
  if (VERBOSE){
    std::cout << "Body Names: \n";

    for (const auto& name : body_names_) {
        std::cout << name << " ";
        std::cout << std::endl;
    }
  }
    //----------------------------------------------------------------------------------------------------------

  base_pos = Math::Vector3d(0., 0., 0.);   // to store unicycle's base center coordinates (x,y,z) in global frame (world)
  R_hand_pos = Math::Vector3d(0., 0., 0.); // to store right hand coordinates (x,y,z) in global frame (world)
  L_hand_pos = Math::Vector3d(0., 0., 0.); // to store left hand coordinates (x,y,z)in global frame (world)

  // flag for the callback:

  compute_initial_pose = 0;

  acquire_meas_arm_l = 0;
  acquire_meas_arm_r = 0;
  acquire_meas_base_real = 0;
  std::string ns = ros::this_node::getName();

  // kill_node = "rosnode kill "+ns;
  // std::cout<<"\nNODE TO BE KILLED:"<<kill_node<<std::endl;
  stability_checked = true;
  control_law.linear.x =  0;
  control_law.linear.y =  0;
  control_law.linear.z =  0;
  control_law.angular.x = 0;
  control_law.angular.y = 0;
}

// Deconstructor:

EgoDance::~EgoDance()
{
}

int EgoDance::sgn(double d)
{
  return d < 0 ? -1 : d > 0;
}

// ----------------------------------------------------------------------------------------------------------
// Function to calculate the robot hands position and the center of the mobile base, respect to the world reference frame
// ----------------------------------------------------------------------------------------------------------

void EgoDance::compute_points()
{
  base_pos = CalcBodyToBaseCoordinates(*model, Q, model->GetBodyId("base"), Math::Vector3d(0., 0., 0.));
  base_rot_mat = CalcBodyWorldOrientation(*model, Q, model->GetBodyId("base"));
  KDL::Rotation(base_rot_mat(0, 0), base_rot_mat(1, 0), base_rot_mat(2, 0), base_rot_mat(0, 1), base_rot_mat(1, 1), base_rot_mat(2, 1), base_rot_mat(0, 2), base_rot_mat(1, 2), base_rot_mat(2, 2)).GetQuaternion(base_pos_quat.x(), base_pos_quat.y(), base_pos_quat.z(), base_pos_quat.w());

  R_hand_pos = CalcBodyToBaseCoordinates(*model, Q, model->GetBodyId("right_hand"), Math::Vector3d(0., 0., 0.));
  R_hand_rot_mat = CalcBodyWorldOrientation(*model, Q, model->GetBodyId("right_hand"));
  KDL::Rotation(R_hand_rot_mat(0, 0), R_hand_rot_mat(1, 0), R_hand_rot_mat(2, 0), R_hand_rot_mat(0, 1), R_hand_rot_mat(1, 1), R_hand_rot_mat(2, 1), R_hand_rot_mat(0, 2), R_hand_rot_mat(1, 2), R_hand_rot_mat(2, 2)).GetQuaternion(R_hand_pos_quat.x(), R_hand_pos_quat.y(), R_hand_pos_quat.z(), R_hand_pos_quat.w());

  L_hand_pos = CalcBodyToBaseCoordinates(*model, Q, model->GetBodyId("left_hand"), Math::Vector3d(0., 0., 0.));
  L_hand_rot_mat = CalcBodyWorldOrientation(*model, Q, model->GetBodyId("left_hand"));
  KDL::Rotation(L_hand_rot_mat(0, 0), L_hand_rot_mat(1, 0), L_hand_rot_mat(2, 0), L_hand_rot_mat(0, 1), L_hand_rot_mat(1, 1), L_hand_rot_mat(2, 1), L_hand_rot_mat(0, 2), L_hand_rot_mat(1, 2), L_hand_rot_mat(2, 2)).GetQuaternion(L_hand_pos_quat.x(), L_hand_pos_quat.y(), L_hand_pos_quat.z(), L_hand_pos_quat.w());

  q << base_pos(0), base_pos(1);
  Pt1 << R_hand_pos(0), R_hand_pos(1);
  Pt2 << L_hand_pos(0), L_hand_pos(1);
}

// -----------------------------------------------------------------------------------------------

// The person who dances with the robot holds it by the hands so a certain distance has to be kept between the center of the robot’s mobile base
// and the hands of the person who holds the hands of the robot in the act of dancing.
// the centre of the unicycle should not converge to the centre of the two target hands, when the distance between
// the center of the mobile base and the hands is the same as the one at the beginning, the robot's mobile base has to stop.
// so the controller has to work now looking at the new shifted target points:

void EgoDance::new_target_points()
{

  double xm_i = (Pt1(0) + Pt2(0)) / 2.0;
  double ym_i = (Pt1(1) + Pt2(1)) / 2.0;

  Eigen::Vector2d Pt1_i(Pt1(0) - xm_i, Pt1(1) - ym_i);
  double epsilon = atan2(Pt1_i(0), Pt1_i(1));
  // std::cout << "epsilon= "<< epsilon <<  std::endl;

  Pt1_t << Pt1(0) + delta1_l_init(0) * cos(epsilon), Pt1(1) - delta1_l_init(0) * sin(epsilon);
  Pt2_t << Pt2(0) + delta2_l_init(0) * cos(epsilon), Pt2(1) - delta2_l_init(0) * sin(epsilon);
}

// -----------------------------------------------------------------------------------------------
// 2° CONTROL:
// -----------------------------------------------------------------------------------------------

void EgoDance::transformation_2()
{
  // To calculate the robot hands position respect to the local reference frame

  Eigen::Vector2d q_g(q(0), q(1));

  Eigen::Matrix<double, 2, 2> Rgl;
  Rgl(0, 0) = cos(Q(2));
  Rgl(0, 1) = -sin(Q(2));
  Rgl(1, 0) = sin(Q(2));
  Rgl(1, 1) = cos(Q(2));

  Eigen::Vector2d dgl(q(0), q(1));

  Eigen::VectorXd Pt1_l_1(Rgl.transpose() * Pt1 - Rgl.transpose() * dgl);
  Pt1_l << Pt1_l_1(0), Pt1_l_1(1);

  Eigen::VectorXd Pt2_l_1(Rgl.transpose() * Pt2 - Rgl.transpose() * dgl);
  Pt2_l << Pt2_l_1(0), Pt2_l_1(1);

  Eigen::VectorXd q_l_1(Rgl.transpose() * q_g - Rgl.transpose() * dgl);
  q_l << q_l_1(0), q_l_1(1);
}

void EgoDance::transformation_2_new() // works with the new shifted target points
{
  // To calculate the robot hands position respect to the local reference frame:

  Eigen::Vector2d q_g(q(0), q(1));

  Eigen::Matrix<double, 2, 2> Rgl;
  Rgl(0, 0) = cos(Q(2));
  Rgl(0, 1) = -sin(Q(2));
  Rgl(1, 0) = sin(Q(2));
  Rgl(1, 1) = cos(Q(2));

  Eigen::Vector2d dgl(q(0), q(1));

  Eigen::VectorXd Pt1_l_1(Rgl.transpose() * Pt1_t - Rgl.transpose() * dgl);
  Pt1_l << Pt1_l_1(0), Pt1_l_1(1);

  Eigen::VectorXd Pt2_l_1(Rgl.transpose() * Pt2_t - Rgl.transpose() * dgl);
  Pt2_l << Pt2_l_1(0), Pt2_l_1(1);

  Eigen::VectorXd q_l_1(Rgl.transpose() * q_g - Rgl.transpose() * dgl);
  q_l << q_l_1(0), q_l_1(1);

  Pm_l << (Pt1_l(0) + Pt2_l(0)) / 2, (Pt1_l(1) + Pt2_l(1)) / 2;
}

void EgoDance::compute_error()
{
  delta1_l = Pt1_l - q_l;
  delta2_l = Pt2_l - q_l;
  delta_l = Pm_l - q_l;
}

void EgoDance::controller_2()
{

  double phi1 = atan2(delta1_l(1), delta1_l(0));
  double phi2 = atan2(delta2_l(1), delta2_l(0));

  v = lambda_1 * delta1_l(0) + lambda_3 * delta2_l(0);
  omega = (lambda_2 * phi1 + lambda_4 * phi2) + k * (fabs(delta1_l(1) - delta2_l(1)) / (delta1_l(1) - delta2_l(1))) * (delta1_l(0) - delta2_l(0));

  if (VERBOSE)
  {
    std::cout << "v = " << v << std::endl;
    std::cout << std::endl;
    std::cout << "omega = " << omega << std::endl;
    std::cout << std::endl;
  }
}

// ----------------------------------------------------------------------------------------------------------
//
//
// Function to calculate the initial distance of each hand from the center of the unicycle,
// when the robot is grabbed by the hands before starting to dance.
//
//
// ----------------------------------------------------------------------------------------------------------

void EgoDance::compute_initial_hand_pose()
{
  compute_points();
  // std::cout << "---------------*************************-------------------- " << std::endl;
  // std::cout << "---------------Compute initial hand pose-------------------- " << std::endl;
  // std::cout << "---------------*************************-------------------- " << std::endl;
  // std::cout << "Base_0: " << std::endl;
  // std::cout << "x: " << q[0] << " y: " << q[1] << std::endl;

  // std::cout << "\nRight hand global_0: " << std::endl;
  // std::cout << "x: " << Pt1[0] << " y: " << Pt1[1] << std::endl;

  // std::cout << "\nLeft hand global_0: " << std::endl;
  // std::cout << "x: " << Pt2[0] << " y: " << Pt2[1] << std::endl;

  transformation_2();

  // std::cout << "\nRight hand local_0: " << std::endl;
  // std::cout << "x: " << Pt1_l[0] << " y: " << Pt1_l[1] << std::endl;

  // std::cout << "\nLeft hand local_0: " << std::endl;
  // std::cout << "x: " << Pt2_l[0] << " y: " << Pt2_l[1] << std::endl;

  // std::cout << "\nq_l_0 = " << std::endl;
  // std::cout << "x: " << q_l[0] << " y: " << q_l[1] << std::endl;

  compute_error();

  // std::cout << "\ndelta1_l_0: " << std::endl;
  // std::cout << "x: " << delta1_l[0] << " y: " << delta1_l[1] << std::endl;

  // std::cout << "\ndelta2_l_0: " << std::endl;
  // std::cout << "x: " << delta2_l[0] << " y: " << delta2_l[1] << std::endl;
  // std::cout << "----------------------------------- " << std::endl;

  delta1_l_init = delta1_l;
  delta2_l_init = delta2_l;

  compute_initial_pose = 1;
}

// ----------------------------------------------------------------------------------------------------------
//
//
// Functions to built the desired controlled linear velocity and angular rate:
//
//
// ----------------------------------------------------------------------------------------------------------

void EgoDance::run()
{
  compute_points();
  new_target_points();
  transformation_2_new();
  compute_error();
  controller_2();

  // Linear Velocity and Angular Rate saturation--------------------------------
  omega = (1 - 0.005) * omega_old + 0.005 * omega;
  omega_old = omega;

  v = (1 - 0.005) * v_old + 0.005 * v;
  v_old = v;

  if (fabs(omega) > 2)
  {
    omega = ((fabs(omega)) / omega) * 2;
  }
  else if (fabs(omega) < 0.1)
  {
    omega = 0;
  }

  if (fabs(v) > 3 * R_)
  {
    v = ((fabs(v)) / v) * 3 * R_;
  }
  else if (fabs(v) < 0.2 * R_)
  {
    v = 0;
  }
  

  //-----------------------------------------

  // Publish desired velocities:

  control_law.linear.x = v / R_;
  control_law.angular.z = omega;

  // se commento queste liee va meglio, perchè Gianni Giovanni le hai aggiunte, già ci sono quelle sopra, sono uguali?

  controller.publish(control_law);
}

// ----------------------------------------------------------------------------------------------------------
//
//
// Function to check the stability of the pitch
//
//
// ----------------------------------------------------------------------------------------------------------
void EgoDance::stability_check(){

  // Check the pitch inclination and kill the node if it is too high
  if (fabs(Q[3]) > 0.25 && stability_checked)
  {
    ROS_ERROR("Pitch out of bound: set velocity to 0");
    // azzero le velocità
    control_law.linear.x = 0.0;
    control_law.angular.z = 0.0;
    stability_checked = false;
    ;
    // controller.publish(control_law);
  }
}

// ----------------------------------------------------------------------------------------------------------
//
//
// Function to print the data for debugging
//
//
// ----------------------------------------------------------------------------------------------------------

void EgoDance::print_data(){

  std::cout << "---------------*************************-------------------- " << std::endl;
  std::cout << "---------------Compute initial hand pose-------------------- " << std::endl;
  std::cout << "---------------*************************-------------------- " << std::endl;

  std::cout << "\nDelta Right hand initial pose: " << std::endl;
  std::cout << "x: " << delta1_l_init[0] << " y: " << delta1_l_init[1] << std::endl;

  std::cout << "\nDelta Left hand initial pose: " << std::endl;
  std::cout << "x: " << delta2_l_init[0] << " y: " << delta2_l_init[1] << std::endl;

  std::cout << "---------------Compute points-------------------- " << std::endl;
  std::cout << "Base: " << std::endl;
  std::cout << "x: " << q[0] << " y: " << q[1] << " z: " << base_pos[2] << std::endl;

  std::cout << "\nRight hand global: " << std::endl;
  std::cout << "x: " << Pt1[0] << " y: " << Pt1[1] << " z: " << R_hand_pos[2] << std::endl;

  std::cout << "\nLeft hand global: " << std::endl;
  std::cout << "x: " << Pt2[0] << " y: " << Pt2[1] << " z: " << L_hand_pos[2] << std::endl;

  std::cout << "---------------new_target_points-------------------- " << std::endl;

  std::cout << "\nRight hand Pt1_t: " << std::endl;
  std::cout << "x: " << Pt1_t[0] << " y: " << Pt1_t[1] << std::endl;

  std::cout << "\nLeft hand Pt2_t: " << std::endl;
  std::cout << "x: " << Pt2_t[0] << " y: " << Pt2_t[1] << std::endl;

  std::cout << "---------------transformated points-------------------- " << std::endl;

  std::cout << "\nRight hand Pt1_l: " << std::endl;
  std::cout << "x: " << Pt1_l[0] << " y: " << Pt1_l[1] << std::endl;

  std::cout << "\nLeft hand Pt2_l: " << std::endl;
  std::cout << "x: " << Pt2_l[0] << " y: " << Pt2_l[1] << std::endl;

  std::cout << "\nMean point: " << std::endl;
  std::cout << "x: " << Pm_l[0] << " y: " << Pm_l[1] << std::endl;

  std::cout << "---------------compute error-------------------- " << std::endl;

  std::cout << "\nDelta Right hand : " << std::endl;
  std::cout << "x: " << delta1_l[0] << " y: " << delta1_l[1] << std::endl;

  std::cout << "\nDelta Left hand: " << std::endl;
  std::cout << "x: " << delta2_l[0] << " y: " << delta2_l[1] << std::endl;

  std::cout << "\nDelta Mean point: " << std::endl;
  std::cout << "x: " << delta_l[0] << " y: " << delta_l[1] << std::endl;

  std::cout << "-------------------------Control----------------------------" << std::endl;
  std::cout << std::endl;

  std::cout << "v = " << v / R_ << std::endl;
  std::cout << std::endl;
  std::cout << "omega  = " << omega << std::endl;
  std::cout << std::endl;
}
// ----------------------------------------------------------------------------------------------------------
//
//
// Function to matlab debug
//
//
// ----------------------------------------------------------------------------------------------------------
void EgoDance::matlab_debug()
{

  // publish for rosbag om topic /data:

  geometry_msgs::Pose dist;
  dist.position.x = delta_l(0);
  dist.position.y = delta_l(1);

  dist.orientation.w = linear_vel_real;
  dist.orientation.x = omega_real;
  dist.orientation.y = v;
  dist.orientation.z = omega;
  rosbag_1.publish(dist);

  // Publish hand_pose rosbag su topic /hand_pose

  geometry_msgs::Pose vec;
  vec.position.x = Q(0);
  vec.position.y = Q(1);
  vec.position.z = Q(2);

  vec.orientation.w = Pt1(0);
  vec.orientation.x = Pt1(1);
  vec.orientation.y = Pt2(0);
  vec.orientation.z = Pt2(1);
  rosbag_2.publish(vec);

  // Publish hand_pose rosbag su topic /unicycle_pose

  geometry_msgs::Pose vec1;
  vec1.position.x = q(0);
  vec1.position.y = q(1);

  vec1.orientation.w = Pt1_t(0);
  vec1.orientation.x = Pt1_t(1);
  vec1.orientation.y = Pt2_t(0);
  vec1.orientation.z = Pt2_t(1);
  rosbag_3.publish(vec1);

  geometry_msgs::Pose vec2;
  vec2.position.x = 0.5 * (Pt1_t(0) + Pt2_t(0));
  vec2.position.y = 0.5 * (Pt1_t(1) + Pt2_t(1));

  rosbag_4.publish(vec2);
}
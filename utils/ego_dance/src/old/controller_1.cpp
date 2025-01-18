#include <ego_dance.h>

ego_dance::ego_dance()

{

  model = new Model();

  nh.getParam("/AlterEgoVersion", AlterEgoVersion);

  // load the gain:

  nh.getParam("/gain_controllers/K", K);
  nh.getParam("/gain_controllers/Kb", Kb);
  nh.getParam("/gain_controllers/Kv", Kv);

  // Topic you want to publish

  controller = nh.advertise<geometry_msgs::Vector3>("/segway_des_vel", 1);
  rosbag_1 = nh.advertise<geometry_msgs::Pose>("/data", 1);
  rosbag_2 = nh.advertise<geometry_msgs::Pose>("/hand_pose", 1);
  rosbag_3 = nh.advertise<geometry_msgs::Pose>("/unicycle_pose", 1);
  rosbag_4 = nh.advertise<geometry_msgs::Pose>("/p_medio", 1);

  // std::string path;

  //   if (!nh.getParam("/path", path))
  //   {
  //     ROS_ERROR("Specify the path of model.urdf");
  //     exit(1);
  //   }

  // std::string path = ros::package::getPath("alterego_robot")+"/alterego_description/urdf/ego_robot_gazebo_v"+std::to_string(AlterEgoVersion)+".urdf";
  std::string path = ros::package::getPath("ego_dance") + "/urdf/ego_robot_gazebo_v" + std::to_string(AlterEgoVersion) + ".urdf";
  std::cout << path << std::endl;

  const char *path_char = path.c_str();

  // Topic you want to subscribe

  robot_state = nh.subscribe("/state_info", 1, &ego_dance::callback_robot_state, this);

  // ----------------------------------------------------------------------------------------------------------

  // load the model

  if (!Addons::URDFReadFromFile(path_char, model, false, false))
  {
    std::cerr << "Error loading model ./onearm_ego.xacro" << std::endl;
  }
  ROS_INFO("Parsing URDF file ok");

  // abort();

  // initialize variables:

  // Unicycle's center:

  q = Eigen::VectorXd::Zero(2);   // global frame
  q_l = Eigen::VectorXd::Zero(2); // local frame

  // Hand_Target 1:

  Pt1 = Eigen::VectorXd::Zero(2);   // global frame
  Pt1_l = Eigen::VectorXd::Zero(2); // local frame

  Pt1_t = Eigen::VectorXd::Zero(2); // shifted target point

  // Hand_Target 2:

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

  R_ = 0.125;
  Pm_l = Eigen::VectorXd::Zero(2);

  eta = 0.0;

  // controller: ----------------------------------------------------

  v = 0.0;
  omega = 0.0;
  v_old = 0.0;
  omega_old = 0.0;
  linear_vel_real = 0.0;
  omega_real = 0.0;

  Q = VectorNd::Zero(model->q_size); // vector of the 24 joint variables: x, y, yaw, pitch, front_kickstand[2], back_kickstand[2], L_wheel, R_wheel, neck[2], L_arm[6], R_arm[6]
  // if Alterego version = 3: vector of the 24 joint variables: x, y, yaw, pitch, front_kickstand[2], back_kickstand[2], L_wheel, R_wheel, neck[2], L_arm[6], R_arm[6]
  // if Alterego version = 2: vector of the 18 joint variables: x, y, yaw, pitch, L_wheel, R_wheel, neck[2], L_arm[5], R_arm[5]

  // Urdf Debug -------------------------------------------------------------
  std::cout << "Q_size = " << model->q_size << std::endl;
  // std::cout << "path_char" << path_char << std::endl;
  //  std::cout << "Q_joint_variable_order = " << std::endl;

  std::cout << Utils::GetModelDOFOverview(*model) << std::endl;
  std::cout << Utils::GetModelHierarchy(*model) << std::endl;

  // std::cout << " Q = " << Q << std::endl;
  // std::cout << "left_id_hand = " << model->GetBodyId("left_hand") <<std::endl;
  // std::cout << "right_id_hand = " << model->GetBodyId("right_hand") <<std::endl;
  // std::cout << "root_link_id = " << model->GetBodyId("base") <<std::endl;

  for (unsigned int body_id = 1; body_id < model->mBodies.size(); ++body_id)
  {
    std::string body_name = model->GetBodyName(body_id);
    std::cout << "Body ID: " << body_id << ", Name: " << body_name << std::endl;
  }

  // abort();

  //----------------------------------------------------------------------------------------------------------

  base_pos = Math::Vector3d(0., 0., 0.);   // to store unicycle's base center coordinates (x,y,z) in global frame (world)
  R_hand_pos = Math::Vector3d(0., 0., 0.); // to store right hand coordinates (x,y,z) in global frame (world)
  L_hand_pos = Math::Vector3d(0., 0., 0.); // to store left hand coordinates (x,y,z)in global frame (world)

  // flag for the callback:

  compute_initial_pose = 0;

  acquire_meas_arm_l = 0;
  acquire_meas_arm_r = 0;
  acquire_meas_base_real = 0;
}

// Deconstructor:

ego_dance::~ego_dance()
{
}

// ----------------------------------------------------------------------------------------------------------
// Function to calculate the robot hands position and the center of the mobile base, respect to the world reference frame
// ----------------------------------------------------------------------------------------------------------

void ego_dance::compute_points()
{

  base_pos = CalcBodyToBaseCoordinates(*model, Q, model->GetBodyId("base"), Math::Vector3d(0., 0., 0.));
  R_hand_pos = CalcBodyToBaseCoordinates(*model, Q, model->GetBodyId("right_hand"), Math::Vector3d(0., 0., 0.));
  L_hand_pos = CalcBodyToBaseCoordinates(*model, Q, model->GetBodyId("left_hand"), Math::Vector3d(0., 0., 0.));

  q << base_pos(0), base_pos(1);
  Pt1 << R_hand_pos(0), R_hand_pos(1);
  Pt2 << L_hand_pos(0), L_hand_pos(1);

  // Debug -----------------------------------------

  //  std::cout << " Q = " << Q << std::endl;

  std::cout << "q = " << q << std::endl;
  std::cout << std::endl;
  std::cout << "Pt1= " << Pt1 << std::endl;
  std::cout << std::endl;
  std::cout << "Pt2= " << Pt2 << std::endl;
  std::cout << std::endl;

  std::cout << std::endl;
  std::cout << "delta1_l_init = " << delta1_l_init << std::endl;
  std::cout << std::endl;
  std::cout << "delta2_l_init = " << delta2_l_init << std::endl;
  std::cout << std::endl;

  std::cout << std::endl;
  std::cout << "Pt1_t = " << Pt1_t << std::endl;
  std::cout << std::endl;
  std::cout << "Pt2_t = " << Pt2_t << std::endl;
  std::cout << std::endl;

  std::cout << std::endl;
  std::cout << "Pt1_l= " << Pt1_l << std::endl;
  std::cout << std::endl;
  std::cout << "Pt2_l= " << Pt2_l << std::endl;
  std::cout << std::endl;
  std::cout << "q_l = " << q_l << std::endl;
  std::cout << std::endl;

  std::cout << "delta1_l = " << delta1_l << std::endl;
  std::cout << std::endl;
  std::cout << "delta2_l = " << delta2_l << std::endl;
  std::cout << std::endl;
}

// -----------------------------------------------------------------------------------------------
// function to calculate the initial distance of each hand from the center of the unicycle,
// when the robot is grabbed by the hands before starting to dance.
// ----------------------------------------------------------------------------------------------------------

void ego_dance::compute_initial_hand_pose()
{
  compute_points();
  transformation_1();
  compute_error();

  delta1_l_init = delta1_l;
  delta2_l_init = delta2_l;

  compute_initial_pose = 1;

  // std::cout << "delta1_l_init = "<< delta1_l_init <<  std::endl;
  // std::cout<<std::endl;
  // std::cout << "delta2_l_init = "<< delta2_l_init <<  std::endl;
  // std::cout<<std::endl;
}

// -----------------------------------------------------------------------------------------------

// The person who dances with the robot holds it by the hands so a certain distance has to be kept between the center of the robot’s mobile base
// and the hands of the person who holds the hands of the robot in the act of dancing.
// the centre of the unicycle should not converge to the centre of the two target hands, when the distance between
// the center of the mobile base and the hands is the same as the one at the beginning, the robot's mobile base has to stop.
// so the controller has to work now looking at the new shifted target points:

void ego_dance::new_target_points()
{

  double xm_i = (Pt1(0) + Pt2(0)) / 2.0;
  double ym_i = (Pt1(1) + Pt2(1)) / 2.0;

  Eigen::Vector2d Pt1_i(Pt1(0) - xm_i, Pt1(1) - ym_i);
  double epsilon = atan2(Pt1_i(0), Pt1_i(1));

  // std::cout << "epsilon= "<< epsilon <<  std::endl;

  double ym = 0; //(delta1_l_init(1) + delta2_l_init(1))/2.0;

  Pt1_t << Pt1(0) + delta1_l_init(0) * cos(epsilon) + ym * sin(epsilon), Pt1(1) - delta1_l_init(0) * sin(epsilon) + ym * cos(epsilon);
  Pt2_t << Pt2(0) + delta2_l_init(0) * cos(epsilon) + ym * sin(epsilon), Pt2(1) - delta2_l_init(0) * sin(epsilon) + ym * cos(epsilon);

  // std::cout << "Pt1_t = "<< Pt1_t <<  std::endl;
  // std::cout<<std::endl;
  // // std::cout << "Pt2_t = "<< Pt2_t <<  std::endl;
  // std::cout<<std::endl;
}

// -----------------------------------------------------------------------------------------------
// 1° CONTROL
// -----------------------------------------------------------------------------------------------

void ego_dance::transformation_1()
{

  // 1° transformation:
  // Roto-traslation of the robot hands so that they lie along the global y-axis.

  double xm = (Pt1(0) + Pt2(0)) / 2.0;
  double ym = (Pt1(1) + Pt2(1)) / 2.0;

  // std::cout << "xm = "<< xm <<  std::endl;
  // std::cout << "ym = "<< ym <<  std::endl;

  Eigen::Vector2d q_new(q(0) - xm, q(1) - ym);

  Eigen::Vector2d Pt1_new(Pt1(0) - xm, Pt1(1) - ym);
  Eigen::Vector2d Pt2_new(Pt2(0) - xm, Pt2(1) - ym);

  double gamma = atan2(Pt1_new(0), Pt1_new(1));

  // std::cout << "q_new = "<< q_new <<  std::endl;
  // std::cout << "Pt1_new = "<< Pt1_new <<  std::endl;
  // std::cout << "Pt2_new = "<< Pt2_new <<  std::endl;
  // std::cout << "gamma = "<< gamma <<  std::endl;

  Eigen::Matrix<double, 2, 2> R_new;
  R_new(0, 0) = cos(gamma);
  R_new(0, 1) = -sin(gamma);
  R_new(1, 0) = sin(gamma);
  R_new(1, 1) = cos(gamma);

  // std::cout << "R_new = "<< R_new <<  std::endl;

  Eigen::VectorXd Pt1_f(R_new * Pt1_new);
  Eigen::VectorXd Pt2_f(R_new * Pt2_new);

  Eigen::VectorXd q_f1(R_new * q_new);

  Eigen::VectorXd q_f = Eigen::VectorXd::Zero(3);
  q_f << q_f1, Q(2) + gamma;

  // std::cout << "Pt1_f = "<< Pt1_f <<  std::endl;
  // std::cout << "Pt2_f = "<< Pt2_f <<  std::endl;
  // std::cout << "q_f1 = "<< q_f1 <<  std::endl;
  // std::cout << "q_f = "<< q_f <<  std::endl;

  // 2° transformation:
  // Calculation of the hands position respect to the local reference frame

  Eigen::Vector2d q_g(q_f(0), q_f(1));

  // std::cout << "q_g = "<< q_g <<  std::endl;

  Eigen::Matrix<double, 2, 2> Rgl;
  Rgl(0, 0) = cos(q_f(2));
  Rgl(0, 1) = -sin(q_f(2));
  Rgl(1, 0) = sin(q_f(2));
  Rgl(1, 1) = cos(q_f(2));

  // std::cout << "Rgl = "<< Rgl <<  std::endl;

  Eigen::Vector2d dgl(q_f(0), q_f(1));

  // std::cout << "dgl = "<< dgl <<  std::endl;

  Eigen::VectorXd Pt1_l_1(Rgl.transpose() * Pt1_f - Rgl.transpose() * dgl);
  Pt1_l << Pt1_l_1(0), Pt1_l_1(1);

  Eigen::VectorXd Pt2_l_1(Rgl.transpose() * Pt2_f - Rgl.transpose() * dgl);
  Pt2_l << Pt2_l_1(0), Pt2_l_1(1);

  std::cout << "Pt1_l= " << Pt1_l << std::endl;
  std::cout << "Pt2_l= " << Pt2_l << std::endl;

  Eigen::VectorXd q_l_1(Rgl.transpose() * q_g - Rgl.transpose() * dgl);
  q_l << q_l_1(0), q_l_1(1);

  // std::cout << "q_l = "<< q_l <<  std::endl;

  std::cout << std::endl;
}

void ego_dance::transformation_1_new() // works with the new shifted target points
{

  // 1° transformation:
  // Roto-traslation of the robot hands so that they lie along the global y-axis.

  double xm = (Pt1_t(0) + Pt2_t(0)) / 2.0;
  double ym = (Pt1_t(1) + Pt2_t(1)) / 2.0;

  // std::cout << "xm = "<< xm <<  std::endl;
  // std::cout << "ym = "<< ym <<  std::endl;

  Eigen::Vector2d q_new(q(0) - xm, q(1) - ym);

  Eigen::Vector2d Pt1_new(Pt1_t(0) - xm, Pt1_t(1) - ym);
  Eigen::Vector2d Pt2_new(Pt2_t(0) - xm, Pt2_t(1) - ym);

  double gamma = atan2(Pt1_new(0), Pt1_new(1));

  eta = Q(2) + gamma;

  // std::cout << "q_new = "<< q_new <<  std::endl;
  // std::cout << "Pt1_new = "<< Pt1_new <<  std::endl;
  // std::cout << "Pt2_new = "<< Pt2_new <<  std::endl;
  // std::cout << "gamma = "<< gamma <<  std::endl;

  Eigen::Matrix<double, 2, 2> R_new;
  R_new(0, 0) = cos(gamma);
  R_new(0, 1) = -sin(gamma);
  R_new(1, 0) = sin(gamma);
  R_new(1, 1) = cos(gamma);

  // std::cout << "R_new = "<< R_new <<  std::endl;

  Eigen::VectorXd Pt1_f(R_new * Pt1_new);
  Eigen::VectorXd Pt2_f(R_new * Pt2_new);
  Eigen::VectorXd q_f1(R_new * q_new);

  Eigen::VectorXd q_f = Eigen::VectorXd::Zero(3);
  q_f << q_f1, Q(2) + gamma;

  // std::cout << "Pt1_f = "<< Pt1_f <<  std::endl;
  // std::cout << "Pt2_f = "<< Pt2_f <<  std::endl;
  // std::cout << "q_f1 = "<< q_f1 <<  std::endl;
  // std::cout << "q_f = "<< q_f <<  std::endl;

  // 2° transformation:
  // Calculation of the hands position respect to the local reference frame

  Eigen::Vector2d q_g(q_f(0), q_f(1));

  // std::cout << "q_g = "<< q_g <<  std::endl;

  Eigen::Matrix<double, 2, 2> Rgl;
  Rgl(0, 0) = cos(q_f(2));
  Rgl(0, 1) = -sin(q_f(2));
  Rgl(1, 0) = sin(q_f(2));
  Rgl(1, 1) = cos(q_f(2));

  // std::cout << "Rgl = "<< Rgl <<  std::endl;

  Eigen::Vector2d dgl(q_f(0), q_f(1));

  // std::cout << "dgl = "<< dgl <<  std::endl;

  Eigen::VectorXd Pt1_l_1(Rgl.transpose() * Pt1_f - Rgl.transpose() * dgl);
  Pt1_l << Pt1_l_1(0), Pt1_l_1(1);

  Eigen::VectorXd Pt2_l_1(Rgl.transpose() * Pt2_f - Rgl.transpose() * dgl);
  Pt2_l << Pt2_l_1(0), Pt2_l_1(1);

  // std::cout << "Pt1_l= "<< Pt1_l <<  std::endl;
  // std::cout << "Pt2_l= "<< Pt2_l <<  std::endl;

  Eigen::VectorXd q_l_1(Rgl.transpose() * q_g - Rgl.transpose() * dgl);
  q_l << q_l_1(0), q_l_1(1);
  Pm_l << (Pt1_l(0) + Pt2_l(0)) / 2, (Pt1_l(1) + Pt2_l(1)) / 2;

  // std::cout << "q_l = "<< q_l <<  std::endl;
}

void ego_dance::compute_error()
{
  delta1_l = Pt1_l - q_l;
  delta2_l = Pt2_l - q_l;
  delta_l = Pm_l - q_l;

  // std::cout << "delta1_l = "<< delta1_l <<  std::endl;
  // std::cout<<std::endl;
  // std::cout << "delta2_l = "<< delta2_l <<  std::endl;
  // std::cout<<std::endl;
}

void ego_dance::controller_1()
{

  double phi1 = atan2(delta1_l(1), delta1_l(0));
  double phi2 = atan2(delta2_l(1), delta2_l(0));
  double d1 = sqrt(pow(delta1_l(0), 2) + pow(delta1_l(1), 2));
  double d2 = sqrt(pow(delta2_l(0), 2) + pow(delta2_l(1), 2));

  //  std::cout << "phi1 = "<< phi1 <<  std::endl;
  //  std::cout<<std::endl;
  //  std::cout << "phi2 = "<< phi2 <<  std::endl;
  //  std::cout<<std::endl;
  // std::cout << "d1 = "<< d1 <<  std::endl;
  // std::cout<<std::endl;
  // std::cout << "d2 = "<< d2 <<  std::endl;
  // std::cout<<std::endl;

  double tau = log(d2 / d1);
  double alpha = phi2 - phi1;

  // std::cout << "tau = "<< tau <<  std::endl;
  // std::cout<<std::endl;
  //  std::cout << "alpha = "<< alpha <<  std::endl;
  //  std::cout<<std::endl;

  double a = sqrt(pow(d1, 2) + pow(d2, 2) - 2 * d1 * d2 * cos(alpha)) / 2.0;

  // std::cout << "a = "<< a <<  std::endl;
  // std::cout<<std::endl;

  double arg = (sin(alpha) * sinh(tau)) / (1 - cos(alpha) * cosh(tau));
  double beta_b = atan(arg) - eta + pi;

  // std::cout << "beta_b = "<< beta_b <<  std::endl;
  // std::cout<<std::endl;

  // limitation of beta_b: ----------------------------------------------------------------------------------------
  // (angle between the heading of the vehicle and the tangent to the circle passing through the vehicle position)

  while (beta_b >= pi || beta_b <= -pi)
  {
    beta_b = beta_b - ((fabs(beta_b)) / beta_b) * 2 * pi;

    //  std::cout << "beta_b sempre qui"<< std::endl;
    //  std::cout << "beta_b_new = "<< beta_b <<  std::endl;
    //  std::cout<<std::endl;
  }

  double alpha_s = pi - alpha;

  // std::cout << "alpha_s  = "<< alpha_s <<  std::endl;
  // std::cout<<std::endl;

  double phi = -atan2(K * sinh(tau), sin(alpha_s));

  // std::cout << "phi  = "<< phi <<  std::endl;
  // std::cout<<std::endl;

  double sigma = beta_b - phi;

  //  std::cout << "sigma  = "<< sigma <<  std::endl;
  //  std::cout<<std::endl;

  // std::cout << "alpha_s = "<< alpha_s <<  std::endl;
  // std::cout << "phi = "<< phi <<  std::endl;
  // std::cout << "sigma = "<< sigma <<  std::endl;

  double den1 = (cos(alpha_s) - cosh(tau));
  double den2 = 1 + pow(((cos(alpha_s) / sin(alpha_s)) * (cosh(tau) / sinh(tau)) + (1 / sin(alpha_s)) * (1 / sinh(tau))), 2);

  double arg2 = (pow(cos(alpha_s) + cosh(tau), 2) * (cos(beta_b) * (1 / sin(alpha_s)) + (1 / sinh(tau)) * sin(beta_b))) / (den1 * sqrt(den2));

  double arg3 = (cos(alpha_s) + cosh(tau)) * (1 / sin(alpha_s)) * (cosh(tau) * sin(beta_b) + cos(beta_b) * (cos(alpha_s) / sin(alpha_s)) * sinh(tau)) / (a + a * pow(K, 2) * pow(1 / sin(alpha_s), 2) * pow(sinh(tau), 2));

  // std::cout << "den1 = "<< den1 <<  std::endl;
  // std::cout<<std::endl;
  // std::cout << "den2  = "<< den2 <<  std::endl;
  // std::cout<<std::endl;
  //  std::cout << "arg2  = "<< arg2 <<  std::endl;
  //  std::cout<<std::endl;
  // std::cout << "arg3  = "<< arg3 <<  std::endl;
  // std::cout<<std::endl;

  // std::cout << "t1  = "<< Kb*sigma <<  std::endl;
  // std::cout<<std::endl;

  v = Kv * ((alpha_s)*cos(beta_b) - tau * sin(beta_b));

  // std::cout << "t2  = "<< (v/a)*arg2 <<  std::endl;
  // std::cout<<std::endl;

  // std::cout << "t3  = "<< K*v*arg3 <<  std::endl;
  // std::cout<<std::endl;

  if (fabs(tau) <= 1e-2)
  {
    std::cout << "sto eseguendo omega semplificata" << std::endl;
    omega = Kb * sigma + v * ((1 / a) * (1 + K + cos(alpha_s)) * (cos((0.5) * alpha_s) / (sin((0.5) * alpha_s))) * sin(beta_b));
  }
  else
  {
    omega = Kb * (sigma) - (v / a) * arg2 + v * K * arg3;
  }

  if (fabs((delta1_l(1) + delta2_l(1))) <= 0.1 && fabs(delta1_l(0)) <= 1e-2 && fabs(delta2_l(0)) <= 1e-2)
  {
    v = 0;
    omega = 0;
    std::cout << "sono nell' if " << std::endl;
  }

  std::cout << "v  = " << v << std::endl;
  std::cout << std::endl;
  std::cout << "omega  = " << omega << std::endl;
  std::cout << std::endl;
  std::cout << "Q(2)  = " << Q(2) << std::endl;
  std::cout << std::endl;
}

// ----------------------------------------------------------------------------------------------------------

void ego_dance::run()
{

  compute_points();
  new_target_points();
  transformation_1_new();
  compute_error();
  controller_1();

  std::cout << "Executing Control 1" << std::endl;
  std::cout << "-----------------------------------------------------" << std::endl;
  std::cout << std::endl;

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
  else if (fabs(v) < 0.1 * R_)
  {
    v = 0;
  }

  // std::cout << "v_sat = "<< v <<  std::endl;
  // std::cout<<std::endl;
  // std::cout << "omega_sat  = "<< omega <<  std::endl;
  // std::cout<<std::endl;
  //-----------------------------------------
  // publish desired velocities:

  geometry_msgs::Vector3 control_law;
  control_law.y = v / R_;
  control_law.x = omega;
  controller.publish(control_law);

  // geometry_msgs::Vector3 control_law;
  // control_law.y = v/R_;
  // control_law.x = omega;
  // if(!std::isnan(control_law.x))
  //   {

  //     controller.publish(control_law);
  //   }

  //   else {
  //     std::cout <<  "vedo nan "<<  std::endl;
  //   }

  // ----------------------------------------------------------------------------------------------------------
  // Matlab Debug:
  // ----------------------------------------------------------------------------------------------------------
  // publish for rosbag om topic /data:

  geometry_msgs::Pose dist;
  dist.position.x = delta_l(0);
  dist.position.y = delta_l(1);

  dist.orientation.w = linear_vel_real;
  dist.orientation.x = omega_real;
  dist.orientation.y = v;
  dist.orientation.z = omega;
  rosbag_1.publish(dist);

  // publish hand_pose rosbag su topic /hand_pose

  geometry_msgs::Pose vec;
  vec.position.x = Q(0);
  vec.position.y = Q(1);
  vec.position.z = Q(2);

  vec.orientation.w = Pt1(0);
  vec.orientation.x = Pt1(1);
  vec.orientation.y = Pt2(0);
  vec.orientation.z = Pt2(1);
  rosbag_2.publish(vec);

  // publish hand_pose rosbag su topic /unicycle_pose

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

// ----------------------------------------------------------------------------------------------------------
// Function to calculate everytime the position (x,y) of the unicycle (mobile base)
// ----------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------
// Callback Functions to acquire the element of vector Q, to update it.
// ----------------------------------------------------------------------------------------------------------

// Joint order ego_robot_gazebo_v3:

// std::cout << Utils::GetModelHierarchy(*model)<< std::endl;

//   Q_size = 24

//   Q_joint_variable_order =

//   0: trans_x_TX
//   1: trans_y_TY
//   2: rot_z_RZ
//   3: rot_y_RY
//   4: back_kick_stand_RY
//   5: back_wheel_RZ
//   6: front_kick_stand_RY
//   7: front_wheel_RZ
//   8: left_shoulder_flange_RZ
//   9: left_arm_flange_RZ
//  10: left_elbow_flange_sensor_RZ
//  11: left_forearm_flange_RZ
//  12: left_wrist_flange_RZ
//  13: left_hand_RZ
//  14: neck_RZ
//  15: neck_cube_custom ( 0  0 -1  0  0  0)
//  16: right_shoulder_flange_RZ
//  17: right_arm_flange_RZ
//  18: right_elbow_flange_sensor_RZ
//  19: right_forearm_flange_RZ
//  20: right_wrist_flange_RZ
//  21: right_hand_RZ
//  22: wheel_L_RZ
//  23: wheel_R_RZ

// ----------------------------------------------------------------------------------------------------------

// Joint order ego_robot_gazebo_v2:

// Q_size = 18

// Q_joint_variable_order =

//   0: trans_x_TX
//   1: trans_y_TY
//   2: rot_z_RZ
//   3: rot_y_RY
//   4: left_shoulder_flange_RZ
//   5: left_arm_flange_RZ
//   6: left_elbow_flange_RZ
//   7: left_forearm_flange_RZ
//   8: left_wrist_flange_RZ
//   9: neck_RZ
//  10: neck_cube_custom ( 0  0 -1  0  0  0)
//  11: right_shoulder_flange_RZ
//  12: right_arm_flange_RZ
//  13: right_elbow_flange_RZ
//  14: right_forearm_flange_RZ
//  15: right_wrist_flange_RZ
//  16: wheel_L_RZ
//  17: wheel_R_RZ

void ego_dance::callback_robot_state(const alterego_msgs::AlteregoState::ConstPtr &msg)
{
  Q[0] = (double)msg->mobile_base_pos_x;
  Q[1] = (double)msg->mobile_base_pos_y;
  Q[2] = (double)msg->yaw_angle;
  Q[3] = (double)msg->pitch_angle;

  if (AlterEgoVersion == 2)
  {

    for (int i = 0; i < msg->left_meas_arm_shaft.size(); i++)
    {
      Q[4 + i] = (double)msg->left_meas_arm_shaft[i];
    }

    for (int i = 0; i < msg->right_meas_arm_shaft.size(); i++)
    {
      Q[11 + i] = (double)msg->right_meas_arm_shaft[i];
    }

    acquire_meas_base_real = 1;
    acquire_meas_arm_l = 1;
    acquire_meas_arm_r = 1;
  }
  else if (AlterEgoVersion == 3)
  {

    for (int i = 0; i < msg->left_meas_arm_shaft.size(); i++)
    {
      Q[8 + i] = (double)msg->left_meas_arm_shaft[i];
    }

    for (int i = 0; i < msg->right_meas_arm_shaft.size(); i++)
    {
      Q[16 + i] = (double)msg->right_meas_arm_shaft[i];
    }

    acquire_meas_base_real = 1;
    acquire_meas_arm_l = 1;
    acquire_meas_arm_r = 1;
  }
}

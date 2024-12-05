#include <controller_test.h>
#include <urdf/model.h>


ego_dance_new::ego_dance_new()

{
  std::cout << "step1" << std::endl;

  std::cout << "step2" << std::endl;

  // Topic you want to publish

  // controller = nh.advertise<geometry_msgs::Vector3>("/segway_des_vel", 1);
  // rosbag_1 = nh.advertise<geometry_msgs::Pose>("/data", 1);
  // rosbag_2 = nh.advertise<geometry_msgs::Pose>("/hand_pose", 1);
  // rosbag_3 = nh.advertise<geometry_msgs::Pose>("/unicycle_pose", 1);
  // rosbag_4 = nh.advertise<geometry_msgs::Pose>("/p_medio", 1);

  std::string path;

  std::cout << "step3" << std::endl;

  if (!nh.getParam("/path", path))
  {
    ROS_ERROR("Specify the path of model.urdf");
    exit(1);
  }

  std::cout << "step4" << std::endl;

  const char *path_char = path.c_str();

  // Topic you want to subscribe

  // base_link_Q = nh.subscribe("/gazebo/link_states", 1, &ego_dance::callback_to_update_base, this);
  // wheel_Q = nh.subscribe("/ego/joint_states", 1, &ego_dance::callback_to_update_wheel, this);
  // arm_Q = nh.subscribe("/ego/robot_state", 1, &ego_dance::callback_to_update_arm, this);
  // base_link_Q = nh.subscribe("/state", 1, &ego_dance::callback_to_update_real_base, this);
  // arm_Q_left = nh.subscribe("/left/meas_arm_shaft", 1, &ego_dance::callback_to_update_left_arm, this);
  // arm_Q_right = nh.subscribe("/right/meas_arm_shaft", 1, &ego_dance::callback_to_update_right_arm, this);

  // ----------------------------------------------------------------------------------------------------------

  // load the model

  // if (!Addons::URDFReadFromFile(path_char, model, true, false))
  // {
  //   std::cerr << "Error loading model ./onearm_ego.xacro" << std::endl;
  // }
  // ROS_INFO("Parsing URDF file ok");

  
  // std::string pathURDF = "/home/alterego-base/catkin_ws/src/AlterEGO/ego_dance/urdf/model.urdf";
  nh.param("/path", pathURDF, std::string());
  // if (!kdl_parser::treeFromUrdfModel(pathURDF, kdl_tree))
  // {
  //   ROS_ERROR("Failed to construct kdl tree for the URDF of Ego loaded as parameter");
  // }
  // urdf::Model modelURDF;
	// if (!modelURDF.initFile(pathURDF)){
	// 	ROS_ERROR("Failed to parse urdf file");
	// 	// return -1;
	// }

  std::cout << "step7" << std::endl;

  // KDL::Tree my_tree;
	if (!kdl_parser::treeFromFile(pathURDF, kdl_tree)){
		ROS_ERROR("Failed to construct kdl tree");
		// return -1;
	}
  std::cout << "step8" << std::endl;




  std::cout << "step9" << std::endl;

  kdl_tree.getChain("torso", "right_hand", KDLChain);
  qChain.resize(KDLChain.getNrOfJoints());
  for (int i = 0; i < KDLChain.getNrOfJoints(); i++)
  {
    qChain(i) = 0;
  }

  std::cout << "step10" << std::endl;
  std::cout << KDLChain.getNrOfJoints() << std::endl;


  KDL::ChainFkSolverPos_recursive fksolverC_des(KDLChain);
  
  fksolverC_des.JntToCart(qChain, Frame);        
  xc << Frame.p[0], Frame.p[1], Frame.p[2]; 
  R_target = Frame.M;
  R_target.GetRPY(roll, pitch, yaw);
  std::cout << "xc target = " << xc[0] << ", " << xc[1] << ", " << xc[2] << std::endl;
  std::cout << "R target = " << roll << ", " << pitch << ", " << yaw << std::endl;

  
  // load the gain:

  // nh.getParam("/gain_controllers/lambda_1", lambda_1);
  // nh.getParam("/gain_controllers/lambda_2", lambda_2);
  // nh.getParam("/gain_controllers/lambda_3", lambda_3);
  // nh.getParam("/gain_controllers/lambda_4", lambda_4);
  // nh.getParam("/gain_controllers/lambda_k", k);

  // initialize variables:

  // Unicycle's center:

  // q = Eigen::VectorXd::Zero(2);   // global frame
  // q_l = Eigen::VectorXd::Zero(2); // local frame

  // // Hand_Target 1:

  // Pt1 = Eigen::VectorXd::Zero(2);   // global frame
  // Pt1_l = Eigen::VectorXd::Zero(2); // local frame

  // Pt1_t = Eigen::VectorXd::Zero(2); // shifted target point

  // // Hand_Target 2:

  // Pt2 = Eigen::VectorXd::Zero(2);   // global frame
  // Pt2_l = Eigen::VectorXd::Zero(2); // local frame

  // Pt2_t = Eigen::VectorXd::Zero(2); // shifted target point

  // // to store distance from the hands to the unicycle center at time t, in local reference frame :

  // delta1_l = Eigen::VectorXd::Zero(2);
  // delta2_l = Eigen::VectorXd::Zero(2);
  // delta_l = Eigen::VectorXd::Zero(2);

  // // to store, at initial time, distance from the hands to the unicycle center at time t, in local reference frame :

  // delta1_l_init = Eigen::VectorXd::Zero(2);
  // delta2_l_init = Eigen::VectorXd::Zero(2);

  // arm_L_q_addon_init = 0.0;
  // arm_R_q_addon_init = 0.0;

  // // arm_L_q_addon_init =  VectorNd::Zero(n_joints_addon);
  // // arm_R_q_addon_init =  VectorNd::Zero(n_joints_addon);

  // R_ = 0.125;
  // Pm_l = Eigen::VectorXd::Zero(2);

  // // controller: ----------------------------------------------------

  // v = 0.0;
  // omega = 0.0;
  // omega_old = 0.0;
  // v_old = 0.0;
  // linear_vel_real = 0.0;
  // omega_real = 0.0;

  // //-----------------------------------------------------------------

  // Q = VectorNd::Zero(model->q_size); // vector of the 16 joint variables: x,y,yaw,pitch,L_wheel,R_wheel,L_arm[5],R_arm[5]
  // Q1 = VectorNd::Zero(model->q_size);
  
  // // std::cout << "Q_size = " << model->q_size << std::endl;
  // // std::cout << "path_char" << path_char << std::endl;
  // // std::cout << "Q_joint_variable_order = " << std::endl;
  // //std::cout << Utils::GetModelDOFOverview(*model)<< std::endl;
  // //std::cout << Utils::GetModelHierarchy(*model)<< std::endl;
  // // std::cout << " Q = " << Q << std::endl;
  // // std::cout << "left_id_hand = " << model->GetBodyId("left_hand") <<std::endl;
  // // std::cout << "right_id_hand = " << model->GetBodyId("right_hand") <<std::endl;
  // // std::cout << "torso= " << model->GetBodyId("torso") <<std::endl;
  //  //abort();
 


  // //--------------------------------------------

  // base_pos = Math::Vector3d(0., 0., 0.);   // to store unicycle's base center coordinates (x,y,z) in global frame (world)
  // R_hand_pos = Math::Vector3d(0., 0., 0.); // to store right hand coordinates (x,y,z) in global frame (world)
  // L_hand_pos = Math::Vector3d(0., 0., 0.); // to store left hand coordinates (x,y,z)in global frame (world)

  // // flag for the callback:

  // // acquire_meas_arm = 0;
  // // acquire_meas_wheel = 0;
  // // acquire_meas_base = 0;

  // compute_initial_pose = 0;

  // acquire_meas_arm_l = 0;
  // acquire_meas_arm_r = 0;
  // acquire_meas_base_real = 0;

  // use_addon = 1;

  // // move arm:

  // status = 1;
  // msg_button_A.data = false;
  // el_cycl = 0;
  // run_freq = 400;
  // enabling_sec = 1;
  // t_slerp = 0;
  // starting_sec = 4;

  // flag_arms_ready  = 0;
  // // pose e orientation delle braccia
  // // Rest position braccio destro
  // rest_R_p << 0.0, -1, 0.0;
  // rest_R_q.x() = -0.7;
  // rest_R_q.y() = 0;
  // rest_R_q.z() = 0;
  // rest_R_q.w() = 0.7;

  // //goal position braccio destro
  // // idle_R_p << 0.0, -0.31, -0.6;
  // // idle_R_q.x() = 0;
  // // idle_R_q.y() = 0.1961161;
  // // idle_R_q.z() = 0;
  // // idle_R_q.w() = 0.9805807;

  // idle_R_p << 0.0, -0.50, -0.6;
  // idle_R_q.x() = 0;
  // idle_R_q.y() = 0;
  // idle_R_q.z() = 0;
  // idle_R_q.w() = 1;

  // //messaggio pubblicato di default a riposo del braccio destro
  // msg_post_R.position.x = rest_R_p.x();
  // msg_post_R.position.y = rest_R_p.y();
  // msg_post_R.position.z = rest_R_p.z();
  // msg_post_R.orientation.x = rest_R_q.x();
  // msg_post_R.orientation.y = rest_R_q.y();
  // msg_post_R.orientation.z = rest_R_q.z();
  // msg_post_R.orientation.w = rest_R_q.w();
  
  // //Rest position braccio sinistro
  // rest_L_p << 0.0, -1, 0.0;
  // rest_L_q.x() = -0.7;
  // rest_L_q.y() = 0;
  // rest_L_q.z() = 0;
  // rest_L_q.w() = 0.7;

  // //goal position braccio sinistro
  // // idle_L_p << 0.0, -0.31, -0.6;
  // // idle_L_q.x() = 0;
  // // idle_L_q.y() = 0.1961161;
  // // idle_L_q.z() = 0;
  // // idle_L_q.w() = 0.9805807;
  
  // idle_L_p << 0.0, -0.50, -0.6;
  // idle_L_q.x() = 0;
  // idle_L_q.y() = 0;
  // idle_L_q.z() = 0;
  // idle_L_q.w() = 1;
  
  // //messaggio pubblicato di default a riposo del braccio sinistro
  // msg_post_L.position.x = rest_L_p.x();
  // msg_post_L.position.y = rest_L_p.y();
  // msg_post_L.position.z = rest_L_p.z();
  // msg_post_L.orientation.x = rest_L_q.x();
  // msg_post_L.orientation.y = rest_L_q.y();
  // msg_post_L.orientation.z = rest_L_q.z();
  // msg_post_L.orientation.w = rest_L_q.w();

  
  // pub_button_A	= nh.advertise<std_msgs::Bool>("/Button_A", 1);
  // pub_post_R		= nh.advertise<geometry_msgs::Pose>("/RH_track", 1);
  // pub_post_L		= nh.advertise<geometry_msgs::Pose>("/LH_track", 1);

}


// deconstructor:

ego_dance_new::~ego_dance_new()
{
}


// -----------------------------------------------------------------------------------------------
// function to calculate the robot hands position and the center of the mobile base, respect to the world reference frame


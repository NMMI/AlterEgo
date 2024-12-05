#include <move_arm.h>

// ----------------------------------------------------------------------------------------------------------
// Function to calculate everytime the position (x,y) of the unicycle (mobile base)
// ----------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------
// Callback Functions to acquire the element of vector Q, to update it.
// ----------------------------------------------------------------------------------------------------------

void MoveArm::callback_robot_state(const ego_msgs::AlteregoState::ConstPtr &msg)
{
  pitch_angle = (double)msg->pitch_angle;

  for (int i = 0; i < msg->left_meas_arm_shaft.size(); i++)
  {
    meas_cube_shaft_left(i) = (double)msg->left_meas_arm_shaft[i];
  }

  for (int i = 0; i < msg->right_meas_arm_shaft.size(); i++)
  {
    meas_cube_shaft_right(i) = (double)msg->right_meas_arm_shaft[i];
  }

  acquire_meas_arm_l = true;
  acquire_meas_arm_r = true;
}

// Constructor
MoveArm::MoveArm()
{
  std::string robot_name = std::getenv("ROBOT_NAME");
  std::string ns = ros::this_node::getName();
  nh.getParam("AlterEgoVersion", AlterEgoVersion);
  nh.getParam("arm_cubes_n", n_joints);
  nh.getParam("left/stiffness_vec", stiffn_vec_left);
  nh.getParam("right/stiffness_vec", stiffn_vec_right);
  nh.getParam(ns + "/degree", degree);
  // ---------------------------------------------------------------------------------------------------subscriber:
  robot_state = nh.subscribe("alterego_state", 1, &MoveArm::callback_robot_state, this);
  // ---------------------------------------------------------------------------------------------------publisher:
  pub_ref_eq_arm_left = nh.advertise<std_msgs::Float64MultiArray>("left/ref_cubes_eq", 1);
  pub_ref_pr_arm_left = nh.advertise<std_msgs::Float64MultiArray>("left/ref_cubes_preset", 1);
  pub_ref_eq_arm_right = nh.advertise<std_msgs::Float64MultiArray>("right/ref_cubes_eq", 1);
  pub_ref_pr_arm_right = nh.advertise<std_msgs::Float64MultiArray>("right/ref_cubes_preset", 1);

  // -------------------------------------------------------------------------------------------------------------

  meas_cube_shaft_left.resize(n_joints);
  meas_cube_shaft_right.resize(n_joints);

  meas_cube_shaft_left = Eigen::VectorXd::Zero(n_joints);
  meas_cube_shaft_right = Eigen::VectorXd::Zero(n_joints);
  q_send_left = Eigen::VectorXd::Zero(n_joints);
  q_send_right = Eigen::VectorXd::Zero(n_joints);
  q_send = Eigen::VectorXd::Zero(n_joints);
  q_send_left_old = Eigen::VectorXd::Zero(n_joints);
  q_send_right_old = Eigen::VectorXd::Zero(n_joints);

  k = 0;
  index_move_arm = 1;
  sec = 10;

  position_reached = false;
  acquire_meas_arm_l = false;
  acquire_meas_arm_r = false;

  use_addon = true;
}

// Deconstructor:

MoveArm::~MoveArm()
{
}

// ----------------------------------------------------------------------------------------------------------
//
// Function to set the position of the arm by defining the side from the main
//
// ----------------------------------------------------------------------------------------------------------

void MoveArm::set_arm_position(std::string side)
{
  if (side == "right")
    bias = 1;
  else
    bias = -1;

  if (index_move_arm != degree * sec)
  {
    q_send(3) = bias * (index_move_arm / sec) * (3.14 / 180);
    index_move_arm++;
  }
  else
  {
    q_send(3) = bias * (index_move_arm / sec) * (3.14 / 180);
    position_reached = true;
  }

  if (degree == 0)
  {
    q_send(3) = 0;
  }
  arm_eq_ref_right_msg.data.clear();
  arm_pr_ref_right_msg.data.clear();
  arm_eq_ref_left_msg.data.clear();
  arm_pr_ref_left_msg.data.clear();
  if (side == "right")
  {
    for (int i = 0; i < n_joints; i++)
    {
      q_send_right(i) = q_send(i);
      arm_eq_ref_right_msg.data.push_back(q_send(i));
      arm_pr_ref_right_msg.data.push_back(stiffn_vec_right[i]);
    }

    pub_ref_eq_arm_right.publish(arm_eq_ref_right_msg);
    pub_ref_pr_arm_right.publish(arm_pr_ref_right_msg);
  }
  else
  {
    for (int i = 0; i < n_joints; i++)
    {
      q_send_left(i) = q_send(i);
      arm_eq_ref_left_msg.data.push_back(q_send(i));
      arm_pr_ref_left_msg.data.push_back(stiffn_vec_left[i]);
    }

    pub_ref_eq_arm_left.publish(arm_eq_ref_left_msg);
    pub_ref_pr_arm_left.publish(arm_pr_ref_left_msg);
  }
}
// ----------------------------------------------------------------------------------------------------------
//
// Check the variation of the pitch angle if out of bound set the arm to zero
//
// ----------------------------------------------------------------------------------------------------------
void MoveArm::reset_arm()
{

  degree = 0;
  sec = 10;
  if (index_move_arm != 0)
  {
    q_send_right(3) = (index_move_arm / sec) * (3.14 / 180);
    q_send_left(3) = -(index_move_arm / sec) * (3.14 / 180);
    index_move_arm--;
  }
  else
  {
    q_send_left(3) = 0;
    q_send_right(3) = 0;
    reset_done = true;
    ROS_ERROR("Pitch out of bound: set arm position to 0");
  }

  arm_eq_ref_right_msg.data.clear();
  arm_pr_ref_right_msg.data.clear();
  arm_eq_ref_left_msg.data.clear();
  arm_pr_ref_left_msg.data.clear();

  for (int i = 0; i < n_joints; i++)
  {
    arm_eq_ref_right_msg.data.push_back(q_send_right(i));
    arm_pr_ref_right_msg.data.push_back(stiffn_vec_right[i]);
  }

  for (int i = 0; i < n_joints; i++)
  {
    arm_eq_ref_left_msg.data.push_back(q_send_left(i));
    arm_pr_ref_left_msg.data.push_back(stiffn_vec_left[i]);
  }

  pub_ref_eq_arm_left.publish(arm_eq_ref_left_msg);
  pub_ref_pr_arm_left.publish(arm_pr_ref_left_msg);
  pub_ref_eq_arm_right.publish(arm_eq_ref_right_msg);
  pub_ref_pr_arm_right.publish(arm_pr_ref_right_msg);
}
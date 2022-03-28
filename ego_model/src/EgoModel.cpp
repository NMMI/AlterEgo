/***
 *  Software License Agreement: BSD 3-Clause License
 *
 * Copyright (c) 2021, NMMI
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// ----------------------------------------------------------------------------

/**
 * \file      EgoModel.cpp
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
 **/
// ----------------------------------------------------------------------------

#include "EgoModel.h"

EgoModel::EgoModel(ros::NodeHandle nh)
{
  model = new Model();

  std::string path;

  if (!nh.getParam("/path", path))
  {
    ROS_ERROR("Specify the path of model.urdf");
    exit(1);
  }

  const char *path_char = path.c_str();

  if (!Addons::URDFReadFromFile(path_char, model, false, false))
  {
    std::cerr << "Error loading model ./onearm_ego.xacro" << std::endl;
    // abort();
  }

  ROS_INFO("Parsing URDF file ok");
  model->gravity = Vector3d(0., 0., -9.81);

  // Initialize Lagrangian coordinates and generalized velocities

  joints_n_ = 16;

  bodies_n_ = 13;
  constraints_n_ = 3;

  q_ = Eigen::VectorXd::Zero(joints_n_);
  q_sent_ = Eigen::VectorXd::Zero(joints_n_);
  q_old_ = Eigen::VectorXd::Zero(joints_n_);

  qd_ = Eigen::VectorXd::Zero(joints_n_);
  qd_sent_ = Eigen::VectorXd::Zero(joints_n_);

  q_linear_ = 0;
  qd_linear_ = 0;

  q_lim_low_ = Eigen::VectorXd::Zero(10);
  q_lim_up_ = Eigen::VectorXd::Zero(10);

  q_lim_low_ << -PI, -PI / 18, -PI, -PI / 2, -PI, -PI, -PI, -PI, -PI / 2, -PI;
  q_lim_up_ << PI, PI, PI, PI / 2, PI, PI, PI / 18, PI, PI / 2, PI;

  // Initialize constraints jacobian
  Jc_ = Eigen::MatrixXd::Zero(constraints_n_, joints_n_);

  // initialize derived constraints jacobian
  Jc_dot_ = Eigen::MatrixXd::Zero(constraints_n_, joints_n_);

  // initialize pseudoinverse of the constraint matrix (weighted with dynamic matrix)
  Jc_pinv_ = Eigen::MatrixXd::Zero(joints_n_, constraints_n_);

  // Initialize dynamics variables and matrices

  masses_ = Eigen::VectorXd::Zero(bodies_n_); // Vector of masses
  masses_ << 13.3260, 1.22, 1.22, 0.6010, 0.6430, 0.5910, 0.3530, 0.3810, 0.6010, 0.6430, 0.5910, 0.3530, 0.3810;

  M_tot_ = 0;
  for (int i = 0; i < masses_.size(); i++)
  {
    M_tot_ = M_tot_ + masses_(i);
  }

  M_ = Eigen::MatrixXd::Zero(joints_n_, joints_n_); // Inertia matrix M

  c_ = Eigen::VectorXd::Zero(joints_n_); // Vector c = C*qd + g

  g_ = Eigen::VectorXd::Zero(joints_n_); // Gravity vector

  cor_ = Eigen::VectorXd::Zero(joints_n_); // Corilis Vector cor = C*qd

  tau_w_ = Eigen::VectorXd::Zero(2);   // Wheels torques
  tau_ub_ = Eigen::VectorXd::Zero(10); // Upper body Torques

  // Initialize flag for Callback
  torque_acquired_ = 0;
  ra_torque_acquired_ = 0;
  la_torque_acquired_ = 0;
  state_acquired_ = 0;

  // Sampling time
  Ts_ = 0.0025;
  Ts_arms_ = 0.005;

  // simulated_robot_ == 0 -> real robot
  // simulated_robot_ == 1 -> the robot is simulated on RVIZ

  if (!nh.getParam("simulated_robot", simulated_robot_))
  {
    ROS_ERROR("EgoModel: Specify simulated parameter");
    exit(1);
  }

  if (!nh.getParam("simulated_on_rviz", sim_rviz_))
  {
    ROS_ERROR("EgoModel: Specify rviz parameter");
    exit(1);
  }

  if (!nh.getParam("simulated_on_gazebo", sim_gazebo_))
  {
    ROS_ERROR("EgoModel: Specify simulated parameter");
    exit(1);
  }

  // acquire_base_ == 1 only when we consider the real robot to take information from the sensors and motors

  if (!nh.getParam("acquire_base", acquire_base_))
  {
    ROS_ERROR("EgoModel: Specify acquire_base_ parameter");
    exit(1);
  }

  if (simulated_robot_ == 0)
  {
    robot_state_pub_ = nh.advertise<sensor_msgs::JointState>("/ego_model/robot_state", 1);
    // info odometria x,y,yaw,pitch

    ra_control_sub_ = nh.subscribe("/ego_model/ra_control", 10, &EgoModel::raTorquesAcquireCallback, this);
    la_control_sub_ = nh.subscribe("/ego_model/la_control", 10, &EgoModel::laTorquesAcquireCallback, this);
    control_sub_ = nh.subscribe("/ego_model/control", 10, &EgoModel::torquesAcquireCallback, this);
  }
  else
  {
    robot_state_pub_ = nh.advertise<sensor_msgs::JointState>("/robot_state", 1);

    ra_control_sub_ = nh.subscribe("/ra_control", 10, &EgoModel::raTorquesAcquireCallback, this);
    la_control_sub_ = nh.subscribe("/la_control", 10, &EgoModel::laTorquesAcquireCallback, this);
    control_sub_ = nh.subscribe("/control", 10, &EgoModel::torquesAcquireCallback, this);

    if (sim_gazebo_ == 1)
    {
      gazebo_base_link_sub_ = nh.subscribe("/gazebo/link_states", 10, &EgoModel::PitchAcquireCallback, this);
      gazebo_joints_sub_ = nh.subscribe("/ego/joint_states", 10, &EgoModel::EncoderAcquireCallback, this);
      gazebo_gyro_sub_ = nh.subscribe("/imu", 10, &EgoModel::PitchRateAcquireCallback, this);
    }

    if (sim_rviz_ == 1)
    {
      // Initialize the joint message to visualize the robot in RVIZ

      joints_pub = nh.advertise<sensor_msgs::JointState>("/ego1/joint_states", 1);

      joint_state_msg_.name.resize(20);
      joint_state_msg_.position.resize(20);

      // HEAD
      joint_state_msg_.name[0] = "world_to_trans_x";
      joint_state_msg_.name[1] = "trans_x_to_trans_y";
      joint_state_msg_.name[2] = "trans_y_to_rot_z";
      joint_state_msg_.name[3] = "rot_z_to_rot_y";

      joint_state_msg_.name[4] = "base_to_neck";
      joint_state_msg_.name[5] = "neck_to_head";

      // LEFT ARM
      joint_state_msg_.name[6] = "base_to_left_shoulder";
      joint_state_msg_.name[7] = "shoulder_to_left_arm";
      joint_state_msg_.name[8] = "arm_to_left_elbow";
      joint_state_msg_.name[9] = "elbow_to_left_forearm";
      joint_state_msg_.name[10] = "forearm_to_left_EE";
      joint_state_msg_.name[11] = "left_hand_synergy_joint";

      // RIGHT ARM
      joint_state_msg_.name[12] = "base_to_right_shoulder";
      joint_state_msg_.name[13] = "shoulder_to_right_arm";
      joint_state_msg_.name[14] = "arm_to_right_elbow";
      joint_state_msg_.name[15] = "elbow_to_right_forearm";
      joint_state_msg_.name[16] = "forearm_to_right_EE";
      joint_state_msg_.name[17] = "right_hand_synergy_joint";

      // Wheels
      joint_state_msg_.name[18] = "L_joint_baseW";
      joint_state_msg_.name[19] = "R_joint_baseW";
    }
  }

  // publishers definitions
  ra_reference_pub_ = nh.advertise<geometry_msgs::Pose>("/frank_q_des_right_no_g", 1);
  la_reference_pub_ = nh.advertise<geometry_msgs::Pose>("/frank_q_des_left_no_g", 1);

  pfaffian_pub_ = nh.advertise<geometry_msgs::Vector3>("/pfaffian", 1);

  if (acquire_base_ == 1)
  {
    state_sub_ = nh.subscribe("/state", 10, &EgoModel::acquireStateCallback, this);
    sub_euler_ = nh.subscribe("/RPY", 1, &EgoModel::callback_imu_euler, this);
    sub_gyro_ = nh.subscribe("/gyro_good", 1, &EgoModel::callback_gyro, this);

    sub_rarm_ = nh.subscribe("/measure_R", 1, &EgoModel::callback_rarm, this);
    sub_larm_ = nh.subscribe("/measure_L", 1, &EgoModel::callback_larm, this);
  }

  ROS_INFO("simulated robot has started");

  // Initialize robot state message

  ego_state_.position.resize(joints_n_);
  ego_state_.velocity.resize(joints_n_);
}

EgoModel::~EgoModel()
{
  ROS_INFO_STREAM("Egomodel deleted");
  delete model;
}

//------------------------------------------------------------------------
// Callback Functions to update joints position and velocity. These
// functions work only when acquire_base_ == 1
//------------------------------------------------------------------------

void EgoModel::acquireStateCallback(const geometry_msgs::Pose &msg)
{
  state_acquired_ = 1; // set this flag to 1 to signal that callback has been executed

  ego_state_.position[2] = msg.orientation.y;
  ego_state_.position[4] = msg.position.y;
  ego_state_.position[5] = msg.position.x;

  ego_state_.velocity[2] = msg.orientation.z;
  ego_state_.velocity[4] = msg.orientation.w;
  ego_state_.velocity[5] = msg.position.z;
}

void EgoModel::callback_gyro(const geometry_msgs::Vector3::ConstPtr &msg)
{
  ego_state_.velocity[3] = msg->y;
}

void EgoModel::callback_imu_euler(const geometry_msgs::Vector3::ConstPtr &msg)
{
  ego_state_.position[3] = msg->y;
}

void EgoModel::callback_larm(const geometry_msgs::Pose &msg)
{
  ego_state_.position[6] = msg.position.x;
  ego_state_.position[7] = msg.position.y;
  ego_state_.position[8] = msg.position.z;
  ego_state_.position[9] = -msg.orientation.x;
  ego_state_.position[10] = msg.orientation.y;
}

void EgoModel::callback_rarm(const geometry_msgs::Pose &msg)
{
  ego_state_.position[11] = msg.position.x;
  ego_state_.position[12] = msg.position.y;
  ego_state_.position[13] = -msg.position.z;
  ego_state_.position[14] = -msg.orientation.x;
  ego_state_.position[15] = -msg.orientation.y;
}

//------------------------------------------------------------------------
// Callback to take measurements from Gazebo
//------------------------------------------------------------------------

void EgoModel::PitchAcquireCallback(const gazebo_msgs::LinkStates &msg)
{
  const std::vector<std::string> &names = msg.name;

  for (int i = 0; i < names.size(); i++)
  {
    if (names[i] == "ego_robot::base_link")
    {
      double roll;
      KDL::Rotation::Quaternion(msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z, msg.pose[i].orientation.w).GetRPY(roll, ego_state_.position[3], ego_state_.position[2]);

      //       std::cout<< "Pitch "<< ego_state_.position[3]<<std::endl;
      //       ROS_INFO_STREAM("PITCH angle: " << ego_state_.position[3] << " (rad)" << " " << ego_state_.position[3]*RAD2DEG << " (deg)");
      //        ROS_INFO_STREAM("YAW angle: " << ego_state_.position[2] << " (rad)" << " " << ego_state_.position[2]*RAD2DEG << " (deg)");
    }
  }
}

void EgoModel::PitchRateAcquireCallback(const sensor_msgs::Imu &msg)
{
  ego_state_.velocity[3] = msg.angular_velocity.y;
}

void EgoModel::EncoderAcquireCallback(const sensor_msgs::JointState &msg)
{
  state_acquired_ = 1;

  const std::vector<std::string> &names = msg.name;

  for (int i = 0; i < names.size(); i++)
  {
    if (names[i] == "L_joint_baseW")
    {
      ego_state_.position[4] = msg.position[i];
      ego_state_.velocity[4] = msg.velocity[i];
    }

    if (names[i] == "R_joint_baseW")
    {
      ego_state_.position[5] = msg.position[i];
      ego_state_.velocity[5] = msg.velocity[i];
    }
  }
}

//------------------------------------------------------------------------
// Callback Functions to acquire torque from the dynamic whole-body control
//------------------------------------------------------------------------

void EgoModel::torquesAcquireCallback(const ego_msgs::Wheel_Torque &msg)
{
  torque_acquired_ = 1;

  // acquire wheels torques
  tau_w_(0) = msg.left_wheel_torque;
  tau_w_(1) = msg.right_wheel_torque;
}

void EgoModel::laTorquesAcquireCallback(const ego_msgs::Arm_Torque &msg)
{
  la_torque_acquired_ = 1;

  // acquire left arm torques
  tau_ub_(0) = msg.ArmTau_1;
  tau_ub_(1) = msg.ArmTau_2;
  tau_ub_(2) = msg.ArmTau_3;
  tau_ub_(3) = msg.ArmTau_4;
  tau_ub_(4) = msg.ArmTau_5;
}

void EgoModel::raTorquesAcquireCallback(const ego_msgs::Arm_Torque &msg)
{
  ra_torque_acquired_ = 1;

  // acquire right arm torques
  tau_ub_(5) = msg.ArmTau_1;
  tau_ub_(6) = msg.ArmTau_2;
  tau_ub_(7) = msg.ArmTau_3;
  tau_ub_(8) = msg.ArmTau_4;
  tau_ub_(9) = msg.ArmTau_5;
}

//------------------------------------------------------------------------
// Function to publish the joint reference position to the arms motors
//------------------------------------------------------------------------

void EgoModel::publishArmsReferences()
{
  // define motors references
  Eigen::VectorXd q_motor;
  int arms_n = 10; // number of arms joints

  q_motor = Eigen::VectorXd::Zero(arms_n);

  for (int i = 0; i < arms_n; i++)
  {
    q_motor(i) = q_(6 + i);
  }

  // saturate q_motor vector (every joint angle is saturated between -PI/2 and PI/2)
  for (int i = 0; i < arms_n; i++)
  {
    if (q_motor(i) > q_lim_up_(i))
    {
      q_motor(i) = q_lim_up_(i);
    }
    else if (q_motor(i) < q_lim_low_(i))
    {
      q_motor(i) = q_lim_low_(i);
    }
  }

  // define messages for right arm reference positions and left arm reference positions
  geometry_msgs::Pose rarm_msg;
  geometry_msgs::Pose larm_msg;

  larm_msg.position.x = q_motor(0);
  larm_msg.position.y = q_motor(1);
  larm_msg.position.z = q_motor(2);
  larm_msg.orientation.x = -q_motor(3);
  larm_msg.orientation.y = q_motor(4);

  rarm_msg.position.x = q_motor(5);
  rarm_msg.position.y = q_motor(6);
  rarm_msg.position.z = -q_motor(7);
  rarm_msg.orientation.x = -q_motor(8);
  rarm_msg.orientation.y = -q_motor(9);

  la_reference_pub_.publish(larm_msg);
  ra_reference_pub_.publish(rarm_msg);
}

//------------------------------------------------------------------------
// In order to set the initial conditions, the pitch angle in simulation
// is computed taking into account the pitch offset, that allow to align
// the CoM forward coordinate with the Base forward coordinate
//------------------------------------------------------------------------

void EgoModel::setInitialConditions()
{
  int base_n_ = 6; // number of mobile base variables

  // compute the pitch offset
  VectorNd Q = VectorNd::Zero(model->q_size);
  VectorNd QDot = VectorNd::Zero(model->qdot_size);
  VectorNd QDDot = VectorNd::Zero(model->qdot_size);

  Math::Vector3d com_pos = Math::Vector3d(0., 0., 0.);
  Math::Vector3d base_pos = Math::Vector3d(0., 0., 0.);
  Math::Vector3d com_vel = Math::Vector3d(0., 0., 0.);
  Math::Vector3d delta = Math::Vector3d(0., 0., 0.);

  Q = q_;
  QDot = qd_;

  Utils::CalcCenterOfMass(*model, Q, QDot, &QDDot, M_tot_, com_pos, &com_vel);
  base_pos = CalcBodyToBaseCoordinates(*model, Q, model->GetBodyId("torso"), Math::Vector3d(0., 0., 0.));
  delta = com_pos - base_pos;

  offsetpitch_ = atan(-delta(0) / delta(2));

  // Initialize the mobile base joints position and velocity.
  // If we consider the real robot, we initialize these variable
  // with the ones measured by the sensors. Otherwise, we initialize
  // it  with all 0 and the pitch offset

  if (acquire_base_ == 1)
  {
    for (int i = 2; i < base_n_; i++)
    {
      q_(i) = ego_state_.position[i];
      qd_(i) = 0 * ego_state_.velocity[i];
    }
  }

  if (acquire_base_ == 0)
  {
    if (sim_rviz_ == 1)
    {
      q_(0) = 0;
      q_(1) = 0;
      q_(2) = 0;
      q_(3) = offsetpitch_;
      q_(4) = 0;
      q_(5) = 0;
    }

    if (sim_gazebo_ == 1)
    {
      q_(3) = ego_state_.position[3];
      q_(4) = ego_state_.position[4];
      q_(5) = ego_state_.position[5];
    }
  }

  q_sent_ = q_;
}

//------------------------------------------------------------------------
// Compute Pfaffian Matrix Jc_
//------------------------------------------------------------------------

void EgoModel::computeConstraintJacobian()
{
  double a = W / 2; // semilength of the axle

  // pure rolling constraints
  Jc_(0, x_coordinate_id) = 1;
  Jc_(0, yaw_angle_id) = -a * cos(q_(yaw_angle_id));
  Jc_(0, pitch_angle_id) = -Rw * cos(q_(yaw_angle_id));
  Jc_(0, lw_angle_id) = -Rw * cos(q_(yaw_angle_id));

  Jc_(1, y_coordinate_id) = 1;
  Jc_(1, yaw_angle_id) = -a * sin(q_(yaw_angle_id));
  Jc_(1, pitch_angle_id) = -Rw * sin(q_(yaw_angle_id));
  Jc_(1, lw_angle_id) = -Rw * sin(q_(yaw_angle_id));

  Jc_(2, x_coordinate_id) = 1;
  Jc_(2, yaw_angle_id) = a * cos(q_(yaw_angle_id));
  Jc_(2, pitch_angle_id) = -Rw * cos(q_(yaw_angle_id));
  Jc_(2, rw_angle_id) = -Rw * cos(q_(yaw_angle_id));
}

//------------------------------------------------------------------------
// compute derivative of the Pfaffian Matrix (Jc_dot_)
//------------------------------------------------------------------------

void EgoModel::computeDerivedConstraintJacobian()
{
  double a = W / 2; // semilength of the axle

  // derivative of the constraint matrix
  Jc_dot_(0, yaw_angle_id) = a * sin(q_(yaw_angle_id)) * qd_(yaw_angle_id);
  Jc_dot_(0, pitch_angle_id) = Rw * sin(q_(yaw_angle_id)) * qd_(yaw_angle_id);
  Jc_dot_(0, lw_angle_id) = Rw * sin(q_(yaw_angle_id)) * qd_(yaw_angle_id);

  Jc_dot_(1, yaw_angle_id) = -a * cos(q_(yaw_angle_id)) * qd_(yaw_angle_id);
  Jc_dot_(1, pitch_angle_id) = -Rw * cos(q_(yaw_angle_id)) * qd_(yaw_angle_id);
  Jc_dot_(1, lw_angle_id) = -Rw * cos(q_(yaw_angle_id)) * qd_(yaw_angle_id);

  Jc_dot_(2, yaw_angle_id) = -a * sin(q_(yaw_angle_id)) * qd_(yaw_angle_id);
  Jc_dot_(2, pitch_angle_id) = Rw * sin(q_(yaw_angle_id)) * qd_(yaw_angle_id);
  Jc_dot_(2, rw_angle_id) = Rw * sin(q_(yaw_angle_id)) * qd_(yaw_angle_id);
}

//-------------------------------------------------------------------------------------------
// compute dynamic matrices which appears in the robot motion equations:
// M(q)ddot(q) + c = tau
// c = cor + g ---> c is the sum of coriolis and centrifugal terms and gravity vector
// Moreover, compute also the pseudo inverse of the Pfaffian Matrix
//-------------------------------------------------------------------------------------------

void EgoModel::computeDynamicMatrices(VectorNd q, VectorNd qd)
{
  VectorNd QDDot = VectorNd::Zero(model->qdot_size);

  VectorNd C = VectorNd::Zero(model->qdot_size);
  MatrixNd M = MatrixNd::Zero(model->q_size, model->q_size);

  Eigen::MatrixXd Mw = Eigen::MatrixXd::Zero(joints_n_, joints_n_);

  CompositeRigidBodyAlgorithm(*model, q, M);
  M_ = M;

  //     std::cout << "Inertia matrix:" << std::endl;
  //     std::cout << M_<< std::endl;

  NonlinearEffects(*model, q, qd, C);
  c_ = C;

  qd = VectorNd::Zero(model->qdot_size);

  NonlinearEffects(*model, q, qd, C);
  g_ = C;

  //     std::cout << "Gravity vector:" << std::endl;
  //     std::cout << g_ << std::endl;

  cor_ = c_ - g_;

  //     std::cout << "Coriolis vector:" << std::endl;
  //     std::cout << cor_ << std::endl;

  Mw = Jc_ * M_.inverse() * Jc_.transpose();
  Jc_pinv_ = M_.inverse() * Jc_.transpose() * Mw.inverse();
}

//----------------------------------------------------------------------------------------------
// integrate motion equations according to the Aumented Formulation and using Ode1
//----------------------------------------------------------------------------------------------

void EgoModel::integrateMotionEquations()
{
  Eigen::VectorXd tau, f;
  Eigen::MatrixXd Id;

  tau = Eigen::VectorXd::Zero(joints_n_);
  f = Eigen::VectorXd::Zero(joints_n_);
  Id = Eigen::MatrixXd::Identity(joints_n_, joints_n_);

  if (torque_acquired_ == 1)
  {
    tau(4) = tau_w_(0);
    tau(5) = tau_w_(1);
  }

  if (ra_torque_acquired_ == 1 && la_torque_acquired_ == 1)
  {
    for (int i = 0; i < 10; i++)
    {
      tau(6 + i) = tau_ub_(i);
    }
  }

  f = (Id - Jc_pinv_ * Jc_) * M_.inverse() * (tau - c_) - Jc_pinv_ * Jc_dot_ * qd_sent_;

  // update joints angles and velocities

  qd_ = qd_sent_ + Ts_ * f;
  q_ = q_sent_ + Ts_ * qd_;

  for (int i = 0; i < 10; i++)
  {
    if (q_(i + 6) > q_lim_up_(i))
    {
      q_(i + 6) = q_lim_up_(i);
    }
    else if (q_(i + 6) < q_lim_low_(i))
    {
      q_(i + 6) = q_lim_low_(i);
    }
  }
}

//----------------------------------------------------------------------------------------------
// Sent robot State
//----------------------------------------------------------------------------------------------

void EgoModel::sentRobotState()
{
  sensor_msgs::JointState robot_state_msg; // define a msg of type sensor_msgs::JointState

  for (int i = 0; i < joints_n_; i++)
  {
    robot_state_msg.position.push_back(q_sent_(i));
    robot_state_msg.velocity.push_back(qd_sent_(i));
  }

  robot_state_msg.position.push_back(q_linear_);
  robot_state_msg.velocity.push_back(qd_linear_);

  robot_state_pub_.publish(robot_state_msg);
}

//----------------------------------------------------------------------------------------------
// Visualize robot State
//----------------------------------------------------------------------------------------------

void EgoModel::visualizeRobot()
{
  joint_state_msg_.position[0] = q_sent_(0);
  joint_state_msg_.position[1] = q_sent_(1);
  joint_state_msg_.position[2] = q_sent_(2);
  joint_state_msg_.position[3] = q_sent_(3);

  joint_state_msg_.position[4] = 0;
  joint_state_msg_.position[5] = 0;

  for (int i = 0; i < 5; i++)
  {
    joint_state_msg_.position[i + 6] = q_sent_(6 + i);
    joint_state_msg_.position[i + 12] = q_sent_(11 + i);
  }

  joint_state_msg_.position[11] = 0;
  joint_state_msg_.position[17] = 0;

  joint_state_msg_.position[18] = q_sent_(4);
  joint_state_msg_.position[19] = q_sent_(5);

  joint_state_msg_.header.stamp = ros::Time::now();
  joints_pub.publish(joint_state_msg_);
}

//----------------------------------------------------------------------------------------------
// Update robot State
//----------------------------------------------------------------------------------------------

void EgoModel::updateRobotState()
{
  float a;                // axle lenght and semilength
  float v, theta, thetad; // forward velocity, yaw angle and its derivative
  float R;
  float x, y, x_old, y_old, q_linear_old; // x and y position of the axle
  float xd, yd;                           // derivative of x and y position of the axle

  VectorNd Q = VectorNd::Zero(model->q_size);
  VectorNd Qd = VectorNd::Zero(model->q_size);
  Q = q_;
  Qd = qd_;

  computeConstraintJacobian();
  computeDerivedConstraintJacobian();
  computeDynamicMatrices(Q, Qd);
  integrateMotionEquations();

  a = W / 2;
  R = Rw;

  // x and y position are computed by integration
  x_old = q_sent_(0);
  y_old = q_sent_(1);
  q_linear_old = q_linear_;

  q_old_ = q_sent_;
  q_sent_ = q_;
  qd_sent_ = qd_;

  if (acquire_base_ == 1 || (simulated_robot_ == 1 && sim_gazebo_ == 1))
  {
    for (int i = 2; i < 6; i++)
    {
      q_sent_(i) = ego_state_.position[i];
      qd_sent_(i) = ego_state_.velocity[i];
    }
  }

  //   ROS_INFO_STREAM("q_left_wheel: " << q_sent_(4) << " (rad)    q_right_wheel" << " " << q_sent_(5));

  theta = R / (2 * a) * (q_sent_(5) - q_sent_(4));    // compute yaw angle
  thetad = R / (2 * a) * (qd_sent_(5) - qd_sent_(4)); // compute yaw rate

  v = R / 2 * (qd_sent_(4) + qd_sent_(5)) + R * qd_sent_(3); // compute forward velocity

  qd_linear_ = v;

  if (simulated_robot_ == 1 && sim_rviz_ == 1)
  {
    q_linear_ = q_linear_old + Ts_ * v;
  }
  else
  {
    q_linear_ = R / 2 * (q_sent_(4) + q_sent_(5)) + R * q_sent_(3);
  }
  //   ROS_INFO_STREAM("Position " << q_linear_ << " (m)");

  xd = v * cos(theta); // compute velocity in the x direction
  yd = v * sin(theta); // compute velocity in the y direction

  x = x_old + Ts_ * v * cos(theta);
  y = y_old + Ts_ * v * sin(theta);

  // compute mobile base joint angles
  q_sent_(0) = x;
  q_sent_(1) = y;
  q_sent_(2) = theta;

  //   ROS_INFO_STREAM("Yaw angle estimate: " << theta << " (rad)" << " " << theta*RAD2DEG << " (deg)");

  // compute mobile base joint velocities
  qd_sent_(0) = xd;
  qd_sent_(1) = yd;
  qd_sent_(2) = thetad;

  if (simulated_robot_ == 0 || (simulated_robot_ == 1 && sim_gazebo_ == 1))
  {
    for (int i = 0; i < 10; i++)
    {
      qd_sent_(6 + i) = 1 / Ts_arms_ * (q_sent_(6 + i) - q_old_(6 + i));
    }
  }

  //   for (int i = 0; i < 10; i ++)
  //   {
  //     qd_sent_(6+i) = 0;
  //     q_sent_(6+i) = 0;
  //   }
  //
  sentRobotState();

  if (sim_rviz_ == 1)
  {
    visualizeRobot();
  }

  if (simulated_robot_ == 0 || (simulated_robot_ == 1 && sim_gazebo_ == 1))
  {
    publishArmsReferences();
  }

  q_ = q_sent_;
  qd_ = qd_sent_;
}
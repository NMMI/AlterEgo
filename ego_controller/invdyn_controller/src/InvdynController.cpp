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
 * \file      InvdynController.cpp
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
 **/
// ----------------------------------------------------------------------------

#include "InvdynController.h"

InvdynController::InvdynController(ros::NodeHandle nh)
{
  ROS_INFO("InvdynController");
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

  model->gravity = Vector3d(0., 0., -9.81);

  // Initialize all the parameters that describe the robot (number of joints, number of actuators, ... )

  joints_n_ = 16;
  act_n_ = 12;
  bodies_n_ = 13;
  constraints_n_ = 3;
  quasi_vel_n_ = 13;

  // Initialize the desired quasi-velocities vector and its integral and derivative

  qb_des_ = Eigen::VectorXd::Zero(quasi_vel_n_);
  qb_des_(2) = -0.01313;
  qb_dot_des_ = Eigen::VectorXd::Zero(quasi_vel_n_);
  qb_ddot_des_ = Eigen::VectorXd::Zero(quasi_vel_n_);

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

  tau_act_ = Eigen::VectorXd::Zero(act_n_);

  // Initialize projector into the nullspace of the constraints jacobian and its derivative
  S_ = Eigen::MatrixXd::Zero(joints_n_, quasi_vel_n_);
  S_dot_ = Eigen::MatrixXd::Zero(joints_n_, quasi_vel_n_);

  // Initialize flag for callback
  state_acquired_ = 0;
  reference_acquired_ = 0;

  // Initialize controller sampling time
  Ts_ = 0.0025;

  // simulated_robot_ == 0 -> real robot
  // simulated_robot_ == 1 -> the robot is simulated on RVIZ

  if (!nh.getParam("simulated_robot", simulated_robot_))
  {
    ROS_ERROR("Invdyn controller: Specify simulated parameter");
    exit(1);
  }

  if (!nh.getParam("simulated_on_gazebo", sim_gazebo_))
  {
    ROS_ERROR("EgoModel: Specify simulated parameter");
    exit(1);
  }

  ROS_INFO_STREAM("sim_gazebo_ == " << sim_gazebo_);

  maximum_torque = 68.0;
  maximum_torque_gazebo = 11.0;

  // define subscribers and publisher
  if (simulated_robot_ == 0)
  {
    // define subscriber for joints state
    state_sub_ = nh.subscribe("/ego_model/robot_state", 10, &InvdynController::acquireStateCallback, this);

    // publisher for arms torques
    wheels_command_pub_ = nh.advertise<ego_msgs::Wheel_Torque>("/ego_model/control", 1);
    rarm_model_torque_pub_ = nh.advertise<ego_msgs::Arm_Torque>("/ego_model/ra_control", 1);
    larm_model_torque_pub_ = nh.advertise<ego_msgs::Arm_Torque>("/ego_model/la_control", 1);

    // publisher for the wheels voltage
    wheels_volt_command_pub_ = nh.advertise<geometry_msgs::Vector3>("/qb_class/cube_ref", 1);

    // define subscriber for joints references
    reference_sub_ = nh.subscribe("/ego_model/references", 10, &InvdynController::acquireReferenceCallback, this);
  }
  else
  {
    // define subscriber for joints state
    state_sub_ = nh.subscribe("/robot_state", 10, &InvdynController::acquireStateCallback, this);

    // publisher for torques
    wheels_command_pub_ = nh.advertise<ego_msgs::Wheel_Torque>("/control", 1);
    rarm_model_torque_pub_ = nh.advertise<ego_msgs::Arm_Torque>("/ra_control", 1);
    larm_model_torque_pub_ = nh.advertise<ego_msgs::Arm_Torque>("/la_control", 1);

    left_torque_pub_ = nh.advertise<std_msgs::Float64>("/ego/left_wheel_controller/command", 1);
    right_torque_pub_ = nh.advertise<std_msgs::Float64>("/ego/right_wheel_controller/command", 1);

    // define subscriber for joints references
    reference_sub_ = nh.subscribe("/references", 10, &InvdynController::acquireReferenceCallback, this);

    ref_base_sub_ = nh.subscribe("/ref_q_base", 10, &InvdynController::acquireBaseQRefCallback, this);
    ref_qd_base_sub_ = nh.subscribe("/ref_qd_base", 10, &InvdynController::acquireBaseQdRefCallback, this);
    ref_rarm_pos_sub_ = nh.subscribe("/ref_rarm_q", 10, &InvdynController::acquireRarmQRefCallback, this);
    ref_rarm_vel_sub_ = nh.subscribe("/ref_rarm_qd", 10, &InvdynController::acquireRarmQdRefCallback, this);
    ref_larm_pos_sub_ = nh.subscribe("/ref_larm_q", 10, &InvdynController::acquireLarmQRefCallback, this);
    ref_larm_vel_sub_ = nh.subscribe("/ref_larm_qd", 10, &InvdynController::acquireLarmQdRefCallback, this);
  }

  // Initialize the gain matrix Kp

  Kp_ = Eigen::MatrixXd::Zero(quasi_vel_n_, quasi_vel_n_);

  if (!nh.getParam("Kp_linear_", Kp_(0, 0)))
  {
    ROS_ERROR("Specify linear position gain Kp_linear_");
    exit(1);
  }

  if (!nh.getParam("Kp_yaw_", Kp_(1, 1)))
  {
    ROS_ERROR("Specify yaw position gain Kp_yaw_");
    exit(1);
  }

  if (!nh.getParam("Kp_pitch_", Kp_(2, 2)))
  {
    ROS_ERROR("Specify pitch position gain Kp_pitch_");
    exit(1);
  }

  std::string larm_prop_gain_string[5];
  for (int i = 0; i < 5; i++)
  {
    larm_prop_gain_string[i] = std::to_string(i + 1);

    if (!nh.getParam("Kp_larm" + larm_prop_gain_string[i] + "_", Kp_(3 + i, 3 + i)))
    {
      ROS_ERROR("Specify gain for right arm control");
      exit(1);
    }
  }

  std::string rarm_prop_gain_string[5];
  for (int i = 0; i < 5; i++)
  {
    rarm_prop_gain_string[i] = std::to_string(i + 1);

    if (!nh.getParam("Kp_rarm" + rarm_prop_gain_string[i] + "_", Kp_(8 + i, 8 + i)))
    {
      ROS_ERROR("Specify gain for right arm control");
      exit(1);
    }
  }

  if (!nh.getParam("Kp_larm_pitch_shoulder_", Kp_(3, 2)))
  {
    ROS_ERROR("Specify gain for left shoulder coupling control");
    exit(1);
  }

  if (!nh.getParam("Kp_rarm_pitch_shoulder_", Kp_(8, 2)))
  {
    ROS_ERROR("Specify gain for right shoulder coupling control");
    exit(1);
  }

  if (!nh.getParam("Kp_larm_pitch_elbow_", Kp_(6, 2)))
  {
    ROS_ERROR("Specify gain for left elbow coupling control");
    exit(1);
  }

  if (!nh.getParam("Kp_rarm_pitch_elbow_", Kp_(11, 2)))
  {
    ROS_ERROR("Specify gain for right elbow coupling control");
    exit(1);
  }

  // Initialize the gain matrix Kd

  Kd_ = Eigen::MatrixXd::Zero(quasi_vel_n_, quasi_vel_n_);

  if (!nh.getParam("Kd_linear_", Kd_(0, 0)))
  {
    ROS_ERROR("Specify linear derivative gain Kd_linear_");
    exit(1);
  }

  if (!nh.getParam("Kd_yaw_", Kd_(1, 1)))
  {
    ROS_ERROR("Specify yaw derivative gain Kd_yaw_");
    exit(1);
  }

  if (!nh.getParam("Kd_pitch_", Kd_(2, 2)))
  {
    ROS_ERROR("Specify pitch derivative gain Kd_pitch_");
    exit(1);
  }

  std::string larm_der_gain_string[5];
  for (int i = 0; i < 5; i++)
  {
    larm_der_gain_string[i] = std::to_string(i + 1);

    if (!nh.getParam("Kd_larm" + larm_der_gain_string[i] + "_", Kd_(3 + i, 3 + i)))
    {
      ROS_ERROR("Specify gain for right arm control");
      exit(1);
    }
  }

  std::string rarm_der_gain_string[5];
  for (int i = 0; i < 5; i++)
  {
    rarm_der_gain_string[i] = std::to_string(i + 1);

    if (!nh.getParam("Kd_rarm" + rarm_der_gain_string[i] + "_", Kd_(8 + i, 8 + i)))
    {
      ROS_ERROR("Specify gain for right arm control");
      exit(1);
    }
  }

  if (!nh.getParam("Kd_larm_pitch_shoulder_", Kd_(3, 2)))
  {
    ROS_ERROR("Specify gain for left shoulder coupling control");
    exit(1);
  }

  if (!nh.getParam("Kd_rarm_pitch_shoulder_", Kd_(8, 2)))
  {
    ROS_ERROR("Specify gain for right shoulder coupling control");
    exit(1);
  }

  if (!nh.getParam("Kd_larm_pitch_elbow_", Kd_(6, 2)))
  {
    ROS_ERROR("Specify gain for left elbow coupling control");
    exit(1);
  }

  if (!nh.getParam("Kd_rarm_pitch_elbow_", Kd_(11, 2)))
  {
    ROS_ERROR("Specify gain for right elbow coupling control");
    exit(1);
  }
}

InvdynController::~InvdynController()
{
  ROS_INFO("InvdynController deleted");
  delete model;
}

//------------------------------------------------------------------------------------------
// Callback to update robot state
//------------------------------------------------------------------------------------------

void InvdynController::acquireStateCallback(const sensor_msgs::JointState &msg)
{
  sensor_msgs::JointState ego_state_temp; // temporary variable to store joints state

  for (int i = 0; i < 17; i++)
  {
    ego_state_temp.position.push_back(msg.position[i]);
    ego_state_temp.velocity.push_back(msg.velocity[i]);
  }

  ego_state_ = ego_state_temp;

  state_acquired_ = 1; // set this flag to 1 to signal that callback has been executed the first time
}

//------------------------------------------------------------------------------------------
// Callback to update the reference of the quasi-velocity and their integral and derivative
//------------------------------------------------------------------------------------------

void InvdynController::acquireReferenceCallback(const sensor_msgs::JointState &msg)
{
  reference_acquired_ = 1;

  double qb_linear_old = qb_des_(0);

  for (int i = 0; i < quasi_vel_n_; i++)
  {
    qb_des_(i) = msg.position[i];
    qb_dot_des_(i) = msg.velocity[i];
    qb_ddot_des_(i) = msg.effort[i];
  }

  qb_des_(0) = qb_linear_old + qb_dot_des_(0) * Ts_;
}

//------------------------------------------------------------------------------------------
// Callback to give by topic the reference to the integral of the quasi-velocities of the
// mobile acquire_base and to the quasi-velocities and their integral of the upper body.
// This option is implemented only for the case of simulated_robot_ == 1.
//------------------------------------------------------------------------------------------

void InvdynController::acquireBaseQRefCallback(const ego_msgs::EgoPose2DUnicycle &msg)
{
  double qb_linear_old = qb_des_(0);
  double yaw_old = qb_des_(1);

  qb_des_(0) = msg.deltaForward + qb_linear_old;
  qb_des_(1) = msg.deltaYaw + yaw_old;
}

void InvdynController::acquireBaseQdRefCallback(const ego_msgs::EgoTwist2DUnicycle &msg)
{
  reference_acquired_ = 1;

  qb_dot_des_(0) = msg.ForwardVelocity;
  qb_dot_des_(1) = msg.YawRate;
}

void InvdynController::acquireLarmQRefCallback(const invdyn_controller::Arm_Des_Pos &msg)
{
  qb_des_(3) = msg.ArmPosDes_1;
  qb_des_(4) = msg.ArmPosDes_2;
  qb_des_(5) = msg.ArmPosDes_3;
  qb_des_(6) = msg.ArmPosDes_4;
  qb_des_(7) = msg.ArmPosDes_5;

  //   qb_des_(6) = -msg.orientation.x;
}

void InvdynController::acquireRarmQRefCallback(const invdyn_controller::Arm_Des_Pos &msg)
{
  qb_des_(8) = msg.ArmPosDes_1;
  qb_des_(9) = msg.ArmPosDes_2;
  qb_des_(10) = msg.ArmPosDes_3;
  qb_des_(11) = msg.ArmPosDes_4;
  qb_des_(12) = msg.ArmPosDes_5;

  //   qb_des_(10) = -msg.position.z;
  //   qb_des_(11) = -msg.orientation.x;
  //   qb_des_(12) = -msg.orientation.y;
}

void InvdynController::acquireLarmQdRefCallback(const invdyn_controller::Arm_Des_Vel &msg)
{
  qb_dot_des_(3) = msg.ArmVelDes_1;
  qb_dot_des_(4) = msg.ArmVelDes_2;
  qb_dot_des_(5) = msg.ArmVelDes_3;
  qb_dot_des_(6) = msg.ArmVelDes_4;
  qb_dot_des_(7) = msg.ArmVelDes_5;
}

void InvdynController::acquireRarmQdRefCallback(const invdyn_controller::Arm_Des_Vel &msg)
{
  qb_dot_des_(8) = msg.ArmVelDes_1;
  qb_dot_des_(9) = msg.ArmVelDes_2;
  qb_dot_des_(10) = msg.ArmVelDes_3;
  qb_dot_des_(11) = msg.ArmVelDes_4;
  qb_dot_des_(12) = msg.ArmVelDes_5;
}

//------------------------------------------------------------------------------------------
// Sign function
//------------------------------------------------------------------------------------------

double InvdynController::sign(double x)
{
  double sign;

  if (x >= 0)
  {
    sign = 1;
  }
  else
  {
    sign = -1;
  }

  return sign;
}

//-------------------------------------------------------------------------------------------
// Function to saturate motor voltage between -100 and 100
//-------------------------------------------------------------------------------------------

Eigen::VectorXd InvdynController::saturateMotorVoltage(Eigen::VectorXd voltage)
{
  Eigen::VectorXd voltage_sat = Eigen::VectorXd::Zero(voltage.size());

  for (int i = 0; i < voltage.size(); i++)
  {
    if (voltage(i) > 100)
      voltage_sat(i) = 100;
    else if (voltage(i) < -100)
      voltage_sat(i) = -100;
    else
      voltage_sat = voltage;
  }

  return voltage_sat;
}

//-------------------------------------------------------------------------------------------
// Function to sent the torques of the mobile base
//-------------------------------------------------------------------------------------------

void InvdynController::publishMbTorques()
{
  Eigen::VectorXd tau_wheels = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd tau_sent = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd vel_wheels = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd voltage = Eigen::VectorXd::Zero(2); // define motors voltage
  Eigen::VectorXd voltage_sat = Eigen::VectorXd::Zero(2);
  double sign_rw, sign_lw;

  tau_wheels(0) = tau_act_(0);
  tau_wheels(1) = tau_act_(1);

  ego_msgs::Wheel_Torque msg;

  for (int i = 0; i < 2; i++)
  {
    if (tau_wheels(i) > maximum_torque)
      tau_wheels(i) = maximum_torque;
    else if (tau_wheels(i) < -maximum_torque)
      tau_wheels(i) = -maximum_torque;
  }

  //   ROS_INFO_STREAM("Right torque (saturated): " << tau_wheels(1) << " (Nm)");

  msg.left_wheel_torque = tau_wheels(0);
  msg.right_wheel_torque = tau_wheels(1);

  wheels_command_pub_.publish(msg);
}

void InvdynController::publishMbVoltage()
{
  Eigen::VectorXd tau_sent = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd vel_wheels = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd voltage = Eigen::VectorXd::Zero(2); // define motors voltage
  Eigen::VectorXd voltage_sat = Eigen::VectorXd::Zero(2);
  double sign_rw, sign_lw;

  tau_sent(0) = tau_act_(0);
  tau_sent(1) = tau_act_(1);

  vel_wheels(0) = ego_state_.velocity[4];
  vel_wheels(1) = ego_state_.velocity[5];

  voltage = (Rm / Kt * tau_sent) * 100 / (n * 24);
  ;

  sign_lw = sign(voltage(0));
  sign_rw = sign(voltage(1));

  voltage(0) = voltage(0) + 20 * sign_lw;
  voltage(1) = voltage(1) + 20 * sign_rw;

  voltage_sat = saturateMotorVoltage(voltage);

  if (simulated_robot_ == 0)
  {
    comm_pub_.x = (voltage_sat(1));
    comm_pub_.y = (voltage_sat(0));

    wheels_volt_command_pub_.publish(comm_pub_);
  }
  else
  {
    tau_sent = (n * 0.24) * Kt / Rm * (voltage_sat)*10.8 / 68.6;

    for (int i = 0; i < 2; i++)
    {
      if (tau_sent(i) > maximum_torque_gazebo)
        tau_sent(i) = maximum_torque_gazebo;
      else if (tau_sent(i) < -maximum_torque_gazebo)
        tau_sent(i) = -maximum_torque_gazebo;
    }

    std_msgs::Float64 msg_lw;
    msg_lw.data = tau_sent(0);

    left_torque_pub_.publish(msg_lw);

    std_msgs::Float64 msg_rw;
    msg_rw.data = tau_sent(1);

    right_torque_pub_.publish(msg_rw);

    ROS_INFO_STREAM("Left torque (saturated): " << tau_sent(0) << " (Nm) Right torque (saturated) " << tau_sent(1) << " (Nm) ");
  }
}

//-------------------------------------------------------------------------------------------
// Function to sent the torques of the upper body
//-------------------------------------------------------------------------------------------

void InvdynController::publishUbTorques()
{
  ego_msgs::Arm_Torque rarm_msg; // define message for right arm torques
  ego_msgs::Arm_Torque larm_msg; // define message for left arm torques

  larm_msg.ArmTau_1 = tau_act_(2);
  larm_msg.ArmTau_2 = tau_act_(3);
  larm_msg.ArmTau_3 = tau_act_(4);
  larm_msg.ArmTau_4 = tau_act_(5);
  larm_msg.ArmTau_5 = tau_act_(6);

  rarm_msg.ArmTau_1 = tau_act_(7);
  rarm_msg.ArmTau_2 = tau_act_(8);
  rarm_msg.ArmTau_3 = tau_act_(9);
  rarm_msg.ArmTau_4 = tau_act_(10);
  rarm_msg.ArmTau_5 = tau_act_(11);

  rarm_model_torque_pub_.publish(rarm_msg);
  larm_model_torque_pub_.publish(larm_msg);
}

//-------------------------------------------------------------------------------------------
// compute dynamic matrixes which appears in the robot motion equations:
// M(q)ddot(q) + c = tau
// c = cor + g ---> c is the sum of coriolis and centrifugal terms and gravity vector
//-------------------------------------------------------------------------------------------

void InvdynController::computeDynamicMatrices()
{

  VectorNd Q = VectorNd::Zero(model->q_size);
  VectorNd QDot = VectorNd::Zero(model->qdot_size);
  VectorNd QDDot = VectorNd::Zero(model->qdot_size);
  VectorNd C = VectorNd::Zero(model->qdot_size);
  MatrixNd M = MatrixNd::Zero(model->q_size, model->q_size);

  for (int i = 0; i < joints_n_; i++)
  {
    Q(i) = ego_state_.position[i];
    QDot(i) = ego_state_.velocity[i];
    QDDot(i) = 0;
  }

  CompositeRigidBodyAlgorithm(*model, Q, M);
  M_ = M;

  //     std::cout << "Inertia matrix:" << std::endl;
  //     std::cout << M << std::endl;

  NonlinearEffects(*model, Q, QDot, C);
  c_ = C;

  QDot = VectorNd::Zero(model->qdot_size);

  NonlinearEffects(*model, Q, QDot, C);
  g_ = C;

  //     std::cout << "Gravity vector:" << std::endl;
  //     std::cout << g_ << std::endl;

  cor_ = c_ - g_;

  //     std::cout << "Coriolis vector:" << std::endl;
  //     std::cout << cor_ << std::endl;
}

//---------------------------------------------------------------------------
// Compute the matrix S such that Jc_*S = 0
//---------------------------------------------------------------------------

void InvdynController::computeNullSpaceProjector()
{
  double a = W / 2; // semilength of the axle
  double theta = ego_state_.position[2];

  S_(0, 0) = cos(theta);

  S_(1, 0) = sin(theta);

  S_(2, 1) = 1;

  S_(3, 2) = 1;

  S_(4, 0) = 1 / Rw;
  S_(4, 1) = -a / Rw;
  S_(4, 2) = -1;

  S_(5, 0) = 1 / Rw;
  S_(5, 1) = a / Rw;
  S_(5, 2) = -1;

  S_.block<10, 10>(6, 3) << Eigen::MatrixXd::Identity(10, 10);
}

//---------------------------------------------------------------------------
// Compute the derivative of the matrix S (S_dot_)
//---------------------------------------------------------------------------

void InvdynController::computeNullSpaceProjectorDerivate()
{
  double theta = ego_state_.position[2];
  double theta_dot = ego_state_.velocity[2];

  S_dot_(0, 0) = -sin(theta) * theta_dot;

  S_dot_(1, 0) = cos(theta) * theta_dot;
}

//----------------------------------------------------------------------------
// compute joint torques according to an inverse dynamics method
// M_tilde = S'*M*S
// c_tilde = S'*(c+g)
// U_tilde = S'*U
// solve in tau: c_tilde + M_tilde*(qb_ddot_des + Kd*(qb_dot_des - qb_dot) +
// + Kp*(qb_des - qb)) = U_tilde*tau
// where qb_dot = quasi-velocities, qb = quasi-velocities integrals
//----------------------------------------------------------------------------

void InvdynController::computeJointsTorques()
{
  Eigen::MatrixXd M_tilde = Eigen::MatrixXd::Identity(quasi_vel_n_, quasi_vel_n_);
  Eigen::VectorXd c_tilde = Eigen::VectorXd::Zero(quasi_vel_n_);
  Eigen::MatrixXd U_tilde = Eigen::MatrixXd::Zero(quasi_vel_n_, act_n_);

  Eigen::MatrixXd S_transposed = Eigen::MatrixXd::Identity(quasi_vel_n_, joints_n_);
  Eigen::MatrixXd U = Eigen::MatrixXd::Zero(joints_n_, act_n_);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(joints_n_);
  Eigen::VectorXd qd = Eigen::VectorXd::Zero(joints_n_);

  Eigen::VectorXd qb = Eigen::VectorXd::Zero(quasi_vel_n_);     // integral of quasi-velocities vector
  Eigen::VectorXd qb_dot = Eigen::VectorXd::Zero(quasi_vel_n_); // quasi-velocities vector

  Eigen::VectorXd tau_tilde = Eigen::VectorXd::Zero(quasi_vel_n_);   // torque to control the quasi-velocities
  Eigen::VectorXd tau_tilde_reduced = Eigen::VectorXd::Zero(act_n_); // tau_tilde without the one element to control the forward velocity

  // update joints angles and velocities
  for (int i = 0; i < joints_n_; i++)
  {
    q[i] = ego_state_.position[i];
    qd[i] = ego_state_.velocity[i];
  }

  // update quasi velocities and their integrals
  qb(0) = ego_state_.position[16]; // linear dispacement
  qb_dot(0) = ego_state_.velocity[16];

  qb(1) = q(2); // yaw angle
  qb_dot(1) = qd(2);

  qb(2) = q(3); // pitch angle
  qb_dot(2) = qd(3);

  for (int i = 0; i < 10; i++) // upper body
  {
    qb(3 + i) = q(6 + i);
    qb_dot(3 + i) = qd(6 + i);
  }

  if (reference_acquired_ == 1)
  {
    double qb_linear_old = qb_des_(0);
    double yaw_old = qb_des_(1);
    qb_des_(0) = qb_linear_old + Ts_ * qb_dot_des_(0);
    qb_des_(1) = yaw_old + Ts_ * qb_dot_des_(1);
  }

  S_transposed = S_.transpose();
  U.block<12, 12>(4, 0) << Eigen::MatrixXd::Identity(act_n_, act_n_);

  M_tilde = S_transposed * M_ * S_;
  c_tilde = S_transposed * (c_ + M_ * S_dot_ * qb_dot);
  U_tilde = S_transposed * U;

  tau_tilde = M_tilde * (qb_ddot_des_ + Kp_ * (qb_des_ - qb) + Kd_ * (qb_dot_des_ - qb_dot)) + c_tilde;

  //   ROS_INFO_STREAM("Pitch angle: " << qb(2) << " (rad)" << " " << qb(2)*RAD2DEG << " (deg)");
  //   ROS_INFO_STREAM("Pitch rate: " << qb_dot(2) << " (rad/s)");

  //   ROS_INFO_STREAM("Des_linear_disp " << qb_des_(0) << " m" );

  // compute the torque of the actuators
  for (int i = 0; i < act_n_; i++)
  {
    tau_tilde_reduced(i) = tau_tilde(1 + i);
  }
  Eigen::MatrixXd U_tilde_reduced = Eigen::MatrixXd::Zero(act_n_, act_n_);
  U_tilde_reduced << U_tilde.block<12, 12>(1, 0);

  tau_act_ = U_tilde_reduced.inverse() * tau_tilde_reduced;

  //   ROS_INFO_STREAM("Left torque: " << tau_act_(0) << " (Nm)");
  //   ROS_INFO_STREAM("Right torque: " << tau_act_(1) << " (Nm)");
}

//-----------------------------------------------------------------------------
// main control routine
//-----------------------------------------------------------------------------
void InvdynController::control()
{
  computeDynamicMatrices();
  computeNullSpaceProjector();
  computeNullSpaceProjectorDerivate();

  computeJointsTorques();

  publishMbTorques();
  publishUbTorques();

  if (simulated_robot_ == 0 || sim_gazebo_ == 1)
  {
    publishMbVoltage();
  }
}

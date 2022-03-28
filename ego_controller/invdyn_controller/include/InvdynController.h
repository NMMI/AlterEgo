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
 * \file      InvdynController.h
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
 **/
// ----------------------------------------------------------------------------

#ifndef INVDYN_CONTROLLER_H
#define INVDYN_CONTROLLER_H

#include <ros/ros.h>
#include <ros/time.h>
#include <urdf/model.h>

#include <string>
#include <cstdio>
#include <unistd.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include <array>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>

#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/SVD>

// Libraries for KDL
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>

// Libraries for RBDL
#include <rbdl/rbdl.h>
#include <rbdl/Constraints.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

// #include <qb_interface/cubeRef.h>

#include <ego_msgs/EgoPose2DUnicycle.h>
#include <ego_msgs/EgoTwist2DUnicycle.h>
#include <invdyn_controller/Arm_Des_Pos.h>
#include <invdyn_controller/Arm_Des_Vel.h>
#include <ego_msgs/Arm_Torque.h>
#include <ego_msgs/Wheel_Torque.h>

#define PI 3.1415926535
#define Rw 0.13 // wheels radius
#define W 0.496 // axle length

// constats to convert couple into voltage
#define Rm 0.84278
#define n 160
#define Kt 0.015056
#define Kb 0.048365
#define RAD2DEG 180 / M_PI

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class InvdynController
{
public:
  InvdynController(ros::NodeHandle nh);
  ~InvdynController();

  Model *model;

  void acquireStateCallback(const sensor_msgs::JointState &msg);
  void acquireReferenceCallback(const sensor_msgs::JointState &msg);

  void acquireBaseQRefCallback(const ego_msgs::EgoPose2DUnicycle &msg);
  void acquireBaseQdRefCallback(const ego_msgs::EgoTwist2DUnicycle &msg);
  void acquireLarmQRefCallback(const invdyn_controller::Arm_Des_Pos &msg);
  void acquireRarmQRefCallback(const invdyn_controller::Arm_Des_Pos &msg);
  void acquireLarmQdRefCallback(const invdyn_controller::Arm_Des_Vel &msg);
  void acquireRarmQdRefCallback(const invdyn_controller::Arm_Des_Vel &msg);

  double sign(double x);
  Eigen::VectorXd saturateMotorVoltage(Eigen::VectorXd voltage);

  void publishMbTorques();
  void publishMbVoltage();
  void publishUbTorques();

  void computeDynamicMatrices();
  void computeNullSpaceProjector();
  void computeNullSpaceProjectorDerivate();
  void computeJointsTorques();
  void control();

  ros::Subscriber state_sub_; // subscriber for the robot state
  ros::Subscriber reference_sub_;

  ros::Subscriber ref_base_sub_;
  ros::Subscriber ref_qd_base_sub_;
  ros::Subscriber ref_rarm_pos_sub_;
  ros::Subscriber ref_rarm_vel_sub_;
  ros::Subscriber ref_larm_pos_sub_;
  ros::Subscriber ref_larm_vel_sub_;

  ros::Publisher wheels_command_pub_;
  ros::Publisher wheels_volt_command_pub_;
  ros::Publisher rarm_model_torque_pub_;
  ros::Publisher larm_model_torque_pub_;
  ros::Publisher left_torque_pub_;
  ros::Publisher right_torque_pub_;

  sensor_msgs::JointState ego_state_;
  geometry_msgs::Vector3 comm_pub_;

  Eigen::MatrixXd S_;
  Eigen::MatrixXd S_dot_;

  Eigen::MatrixXd M_;
  Eigen::VectorXd c_;
  Eigen::VectorXd cor_;
  Eigen::VectorXd g_;
  Eigen::VectorXd tau_act_;
  double maximum_torque;
  double maximum_torque_gazebo;

  Eigen::MatrixXd Kp_, Kd_;
  Eigen::VectorXd qb_des_, qb_dot_des_, qb_ddot_des_;

  Eigen::VectorXd masses_;
  double M_tot_;

  int simulated_robot_;
  int sim_gazebo_;
  int state_acquired_;
  int reference_acquired_;

  int joints_n_;
  int act_n_;
  int bodies_n_;
  int constraints_n_;
  int quasi_vel_n_;
  double Ts_;
};

#endif
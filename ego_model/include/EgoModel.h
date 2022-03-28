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
 * \file      EgoModel.h
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
 **/
// ----------------------------------------------------------------------------

#ifndef EGO_MODEL_H
#define EGO_MODEL_H

#include <ros/ros.h>
#include <ros/time.h>
#include <urdf/model.h>

#include <string>
#include <cstring>
#include <iostream>
#include <cstdio>
#include <cstdlib>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/LinkStates.h>

#include <ego_msgs/Arm_Torque.h>
#include <ego_msgs/Wheel_Torque.h>

#include <unistd.h>
#include <serial/serial.h>
#include <cmath>
#include <math.h>
#include <array>

#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/Geometry>

#include "visualization_msgs/Marker.h"

// Libraries for KDL
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

// Libraries for RBDL
#include <rbdl/rbdl.h>
#include <rbdl/Constraints.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#define Rw 0.13 // wheels radius
#define W 0.496 // axle length
#define PI 3.14

#define Rm 0.84278
#define n 160
#define Kt 0.015056
#define Kb 0.048365

#define RAD2DEG 180 / M_PI

#define x_coordinate_id 0 // index corresponding to the x coordinate of the main body
#define y_coordinate_id 1 // index corresponding to the y coordinate of the main body
#define yaw_angle_id 2    // index corresponding to the yaw angle of the main body
#define pitch_angle_id 3  // index corresponding to the pitch angle of the main body
#define lw_angle_id 4     // index corresponding to the left wheel angle
#define rw_angle_id 5     // index corresponding to the right wheel angle

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class EgoModel
{
public:
  EgoModel(ros::NodeHandle nh);
  ~EgoModel();

  Model *model;

  // callbacks for data acquisition
  void acquireStateCallback(const geometry_msgs::Pose &msg);
  void callback_imu_euler(const geometry_msgs::Vector3::ConstPtr &msg);
  void callback_gyro(const geometry_msgs::Vector3::ConstPtr &msg);
  void callback_larm(const geometry_msgs::Pose &msg);
  void callback_rarm(const geometry_msgs::Pose &msg);

  void PitchAcquireCallback(const gazebo_msgs::LinkStates &msg);
  void EncoderAcquireCallback(const sensor_msgs::JointState &msg);
  void PitchRateAcquireCallback(const sensor_msgs::Imu &msg);

  void torquesAcquireCallback(const ego_msgs::Wheel_Torque &msg);
  void raTorquesAcquireCallback(const ego_msgs::Arm_Torque &msg); // right arm torque callback
  void laTorquesAcquireCallback(const ego_msgs::Arm_Torque &msg); // left arm torque callback

  void publishArmsReferences();
  void setInitialConditions();

  void computeDynamicMatrices(VectorNd q, VectorNd qd);
  void computeConstraintJacobian();
  void computeDerivedConstraintJacobian();
  void integrateMotionEquations();
  void sentRobotState();
  void visualizeRobot();
  void updateRobotState();

  ros::Publisher robot_state_pub_;
  ros::Publisher ra_reference_pub_, la_reference_pub_; // publishers for right arm and left arm reference positions
  ros::Publisher pfaffian_pub_;
  ros::Publisher joints_pub;

  ros::Subscriber control_sub_;
  ros::Subscriber ra_control_sub_, la_control_sub_; // subscribers for right arm torques and left arm torques
  ros::Subscriber state_sub_;
  ros::Subscriber gazebo_base_link_sub_;
  ros::Subscriber gazebo_joints_sub_;
  ros::Subscriber gazebo_gyro_sub_;

  ros::Subscriber sub_euler_;
  ros::Subscriber sub_gyro_;
  ros::Subscriber sub_rarm_;
  ros::Subscriber sub_larm_;

  sensor_msgs::JointState ego_state_; // joints state (acquired from real Robot)
  sensor_msgs::JointState joint_state_msg_;

  int state_acquired_;
  int torque_acquired_;
  int ra_torque_acquired_, la_torque_acquired_;

  int simulated_robot_;
  int acquire_base_; // if acquire_base == 0, just simulate the model, else acquire feedback variables from the mobile base
  int sim_rviz_;
  int sim_gazebo_;

  int joints_n_;
  int constraints_n_;
  int bodies_n_;

  Eigen::VectorXd q_, qd_, q_sent_, qd_sent_, q_old_;
  Eigen::VectorXd q_lim_up_, q_lim_low_;
  double q_linear_;
  double qd_linear_;
  double offsetpitch_;

  Eigen::VectorXd masses_; // vector of masses
  double M_tot_;           // mass of the robot

  Eigen::MatrixXd Jc_pinv_;
  Eigen::MatrixXd Jc_, Jc_dot_;

  Eigen::MatrixXd M_;
  Eigen::VectorXd c_, g_, cor_;

  Eigen::VectorXd tau_w_;
  Eigen::VectorXd tau_ub_;

  double Ts_;
  double Ts_arms_;
};

#endif
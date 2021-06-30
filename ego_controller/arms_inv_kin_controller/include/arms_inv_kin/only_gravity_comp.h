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
 * \file      only_gravity_comp.h
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
**/
// ----------------------------------------------------------------------------


#include <ros/ros.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>

#include <eigen3/Eigen/Eigen>
#include <math.h>

#include <boost/scoped_ptr.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

#include <pseudo_inversion.h>
#include <skew_symmetric.h>


# define PI 3.1416
# define PI2 6.2832


class only_gravity_comp
{
public:
  only_gravity_comp();
  ~only_gravity_comp();



  void run();
  double dt;



private:
  void callback_right_q(const geometry_msgs::Pose::ConstPtr& msg);
  void callback_left_q(const geometry_msgs::Pose::ConstPtr& msg);
  void run_R();
  void run_L();
  int sgn(double d);


  Eigen::MatrixXd W_;
  KDL::Chain right_arm_chain_, left_arm_chain_;
  double a_motor_, k_motor_;


//Right arm
  // Eigen::Vector3d T_bt_R_;          //traslazione rigida dal sistema posto sul primo cubo del braccio destro al sistema torso
  // Eigen::Matrix3d R_bt_R_;          //rotazione rigida dal sistema posto sul primo cubo del braccio destro al sistema torso
  // Eigen::Quaterniond Q_bt_R_;       //rotazione rigida dal sistema posto sul primo cubo del braccio destro al sistema torso
  Eigen::Quaterniond Q_zmin90_;        //rotazione di -90 intorno a z
  Eigen::VectorXd defl_R_;                                                                                                                             
  KDL::JntArray  q_R_;            // Joint positions
  boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_R_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_R_;
  boost::scoped_ptr<KDL::ChainDynParam> id_solver_R_;
  KDL::JntArray G_comp_R_; // gravity compensation
  KDL::RigidBodyInertia inert_R_0_, inert_R_1_, inert_R_2_, inert_R_3_, inert_R_4_, inert_R_5_;
  double qpreset_R_;
  KDL::Vector gravity_R_;

//Left arm
  Eigen::Quaterniond Q_zplus90_;        //rotazione di -90 intorno a z
  Eigen::VectorXd defl_L_;                                                                                                                             
  KDL::JntArray  q_L_;            // Joint positions

  boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_L_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_L_;
  boost::scoped_ptr<KDL::ChainDynParam> id_solver_L_;  
  KDL::JntArray G_comp_L_; // gravity compensation
  KDL::RigidBodyInertia inert_L_0_, inert_L_1_, inert_L_2_, inert_L_3_, inert_L_4_, inert_L_5_;
  double qpreset_L_;
  KDL::Vector gravity_L_;


  ros::NodeHandle n_;

  ros::Publisher pub_ref1_, pub_ref2_;
  ros::Subscriber sub_right_q_, sub_left_q_;
  
  std_msgs::Float64MultiArray ref1, ref2;
  
  Eigen::VectorXd q_Eq_R_;
  Eigen::VectorXd q_Eq_L_;

};



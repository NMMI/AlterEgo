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
 * \file      only_gravity_comp.cpp
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
**/
// ----------------------------------------------------------------------------


#include <only_gravity_comp.h>

only_gravity_comp::only_gravity_comp()
{
  double d1 = 0.0;
  double d2 = 0.090;
  double d3 = 0.078;
  double d4 = 0.090;
  double d5 = 0.090;

  Q_zmin90_.w() = 0.7071;
  Q_zmin90_.vec() << 0.0, 0.0, -0.7071;

  Q_zplus90_.w() = 0.7071;
  Q_zplus90_.vec() << 0.0, 0.0, 0.7071;

  // rotazione rigida  da braccio destro a torso 
  Eigen::Matrix3d Rot_R2t;
  Rot_R2t << 0.173648, 	  0.0,  0.984808,
		    -0.171010, 0.984808, 0.0301537,
		    -0.969846,-0.173648,  0.171010;


  Eigen::Vector3d grav_R(0.0, 0.0, -9.81);
  grav_R = Rot_R2t * grav_R;
  // gravità (kdl) riportata nel sistema braccio destro per ricavare G
  gravity_R_ = KDL::Vector(grav_R(0), grav_R(1), grav_R(2));
  // gravity_R_ = KDL::Vector(-9.81, 0.0, 0.0); //non riportata


  // rotazione rigida  da braccio sinistro a torso 
  Eigen::Matrix3d Rot_L2t, Rot_z_pi;
  Rot_z_pi << -1.0,  0.0, 0.0,
			    0.0, -1.0, 0.0,
			    0.0,  0.0, 1.0;

  Rot_L2t << 0.173648, 	  0.0, -0.984808,
		      0.171010, 0.984808, 0.0301537,
		      0.969846,-0.173648,  0.171010;

  Eigen::Vector3d grav_L(0.0, 0.0, -9.81);
  grav_L = Rot_z_pi * Rot_L2t * grav_L;
  // gravità (kdl) riportata nel sistema braccio destro per ricavare G
  gravity_L_ = KDL::Vector(grav_L(0), grav_L(1), grav_L(2));
  // gravity_L_ = KDL::Vector(-9.81, 0.0, 0.0); //non riportata


  //costanti derivate dal cubo datasheet
  a_motor_ = 2.0;
  k_motor_ = 0.5;

  inert_R_1_ = KDL::RigidBodyInertia(0.6, KDL::Vector(0.0, 0.0, 0.0)); // secondo cubo
  inert_R_2_ = KDL::RigidBodyInertia(0.643, KDL::Vector(0.0, 0.0, (0.09 - 0.007)));
  inert_R_3_ = KDL::RigidBodyInertia(0.59, KDL::Vector(0.0, 0.0, 0.0));
  inert_R_4_ = KDL::RigidBodyInertia(0.35, KDL::Vector(0.0, 0.0, (0.09 - 0.013)));
  inert_R_5_ = KDL::RigidBodyInertia(0.38, KDL::Vector(0.0, 0.0, 0.0));//mano

  inert_L_1_ = KDL::RigidBodyInertia(0.6, KDL::Vector(0.0, 0.0, 0.0)); // secondo cubo
  inert_L_2_ = KDL::RigidBodyInertia(0.643, KDL::Vector(0.0, 0.0, (0.09 - 0.007)));
  inert_L_3_ = KDL::RigidBodyInertia(0.59, KDL::Vector(0.0, 0.0, 0.0));
  inert_L_4_ = KDL::RigidBodyInertia(0.35, KDL::Vector(0.0, 0.0, (0.09 - 0.013)));
  inert_L_5_ = KDL::RigidBodyInertia(0.38, KDL::Vector(0.0, 0.0, 0.0));//mano


  //DH del braccio destro(libreria KDL)
  right_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, PI/2.0, d1, 0.0), inert_R_1_));
  right_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, -PI/2.0, 0.0, PI/2.0), inert_R_2_));
  right_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, -PI/2.0, d2 + d3, -PI/2.0), inert_R_3_));
  right_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, PI/2.0, 0.0, 0.0), inert_R_4_));
  right_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, -PI/2.0, d4 + d5, 0.0), inert_R_5_));


  //DH del braccio sinistro(libreria KDL)
  left_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, -PI/2.0, d1, 0.0), inert_L_1_));
  left_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, PI/2.0, 0.0, -PI/2.0), inert_L_2_));
  left_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, PI/2.0, d2 + d3, PI/2.0), inert_L_3_));
  left_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, -PI/2.0, 0.0, 0.0), inert_L_4_));
  left_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, PI/2.0, d4 + d5, 0.0), inert_L_5_));

  // chain.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(double a, double alpha, double d, double theta)));
  
  // Construct the kdl solvers in non-realtime.
  jnt_to_pose_solver_R_.reset(new KDL::ChainFkSolverPos_recursive(right_arm_chain_));
  jnt_to_jac_solver_R_.reset(new KDL::ChainJntToJacSolver(right_arm_chain_));
  id_solver_R_.reset(new KDL::ChainDynParam(right_arm_chain_,gravity_R_));


  jnt_to_pose_solver_L_.reset(new KDL::ChainFkSolverPos_recursive(left_arm_chain_));
  jnt_to_jac_solver_L_.reset(new KDL::ChainJntToJacSolver(left_arm_chain_));
  id_solver_L_.reset(new KDL::ChainDynParam(left_arm_chain_,gravity_L_));



  // Resize (pre-allocate) the variables in non-realtime.                                                                                                                         
  q_R_.resize(right_arm_chain_.getNrOfJoints());	
  KDL::SetToZero(q_R_);
  defl_R_.resize(right_arm_chain_.getNrOfJoints());
  defl_R_ = Eigen::VectorXd::Zero(right_arm_chain_.getNrOfJoints());
  G_comp_R_.resize(right_arm_chain_.getNrOfJoints());
  KDL::SetToZero(G_comp_R_);
  qpreset_R_ = 0.3;
  q_Eq_R_ = Eigen::VectorXd::Zero(right_arm_chain_.getNrOfJoints());



  q_L_.resize(left_arm_chain_.getNrOfJoints());
  KDL::SetToZero(q_L_);
  defl_L_.resize(left_arm_chain_.getNrOfJoints());
  defl_L_ = Eigen::VectorXd::Zero(left_arm_chain_.getNrOfJoints());
  G_comp_L_.resize(left_arm_chain_.getNrOfJoints());
  KDL::SetToZero(G_comp_L_);
  qpreset_L_ = 0.3;
  q_Eq_L_ = Eigen::VectorXd::Zero(right_arm_chain_.getNrOfJoints());


  // //Topic you want to publish
  //Publisher of the references of the motors
  pub_ref1_ = n_.advertise<std_msgs::Float64MultiArray >("/AlterEgo/reference_1", 10);
  ref1.data.resize(10);
  pub_ref2_ = n_.advertise<std_msgs::Float64MultiArray >("/AlterEgo/reference_2", 10);
  ref2.data.resize(10);

  //Topic you want to subscribe
  sub_right_q_ = n_.subscribe("/frank_q_des_right_no_g", 10, &only_gravity_comp::callback_right_q, this);
  sub_left_q_ = n_.subscribe("/frank_q_des_left_no_g", 10, &only_gravity_comp::callback_left_q, this);

}

only_gravity_comp::~only_gravity_comp()
{

}

void only_gravity_comp::callback_right_q(const geometry_msgs::Pose::ConstPtr& msg)
{
  q_R_(0) = msg->position.x;
  q_R_(1) = msg->position.y;
  q_R_(2) = msg->position.z;
  q_R_(3) = msg->orientation.x;
  q_R_(4) = msg->orientation.y;

}

void only_gravity_comp::callback_left_q(const geometry_msgs::Pose::ConstPtr& msg)
{
  q_L_(0) = msg->position.x;
  q_L_(1) = msg->position.y;
  q_L_(2) = msg->position.z;
  q_L_(3) = msg->orientation.x;
  q_L_(4) = msg->orientation.y;

}

void only_gravity_comp::run_R()
{	

  id_solver_R_->JntToGravity(q_R_, G_comp_R_);

  for(int i = 0; i < right_arm_chain_.getNrOfJoints(); i++)
  {
    if(fabs(G_comp_R_(i)) < 0.0000001)
    {
	    defl_R_(i) = 0.0; 
    }
    else
    {				
	    defl_R_(i) = -atan((k_motor_ + k_motor_ * pow(tan(a_motor_ * qpreset_R_),2) - sqrt(pow(-G_comp_R_(i),2) * pow(tan(a_motor_ * qpreset_R_),2) + 2 * pow(k_motor_,2) * pow(tan(a_motor_ * qpreset_R_),2) + pow(k_motor_,2) * pow(tan(a_motor_ * qpreset_R_),4) + pow(k_motor_,2))) / (-G_comp_R_(i) * pow(tan(a_motor_ * qpreset_R_),2))) / a_motor_;	
    }
  }

//   q_right_pub_.QbAdvancePosDes_1 = q_R_(0) - defl_R_(0);
//   q_right_pub_.QbAdvancePosDes_2 = q_R_(1) - defl_R_(1);
//   q_right_pub_.QbAdvancePosDes_3 = -(q_R_(2) - defl_R_(2));
//   q_right_pub_.QbAdvancePosDes_4 = q_R_(3) - defl_R_(3); 
//   q_right_pub_.QbAdvancePosDes_5 = -(q_R_(4) - defl_R_(4));

  for(int i=0; i<5; i++)
  {
    q_Eq_R_(i) = q_R_(i) - defl_R_(i);
    
    if(i == 2 || i == 4)
    {
      q_Eq_R_(i) = -(q_R_(i) - defl_R_(i));
    }
    
    ref1.data[i+5] =  q_Eq_R_(i) + qpreset_R_;
    ref2.data[i+5] =  q_Eq_R_(i) - qpreset_R_;

  }
}


void only_gravity_comp::run_L()
{	

  id_solver_L_->JntToGravity(q_L_, G_comp_L_);

  for(int i = 0; i < left_arm_chain_.getNrOfJoints(); i++)
  {
    if(fabs(G_comp_L_(i)) < 0.0000001)
    {
	    defl_L_(i) = 0.0; 
    }
    else
    {				
	    defl_L_(i) = -atan((k_motor_ + k_motor_ * pow(tan(a_motor_ * qpreset_L_),2) - sqrt(pow(-G_comp_L_(i),2) * pow(tan(a_motor_ * qpreset_L_),2) + 2 * pow(k_motor_,2) * pow(tan(a_motor_ * qpreset_L_),2) + pow(k_motor_,2) * pow(tan(a_motor_ * qpreset_L_),4) + pow(k_motor_,2))) / (-G_comp_L_(i) * pow(tan(a_motor_ * qpreset_L_),2))) / a_motor_;	
    }
  }

//   q_left_pub_.QbAdvancePosDes_1 = q_L_(0) - defl_L_(0);
//   q_left_pub_.QbAdvancePosDes_2 = q_L_(1) - defl_L_(1);
//   q_left_pub_.QbAdvancePosDes_3 = q_L_(2) - defl_L_(2);
//   q_left_pub_.QbAdvancePosDes_4 = q_L_(3) - defl_L_(3); // il meno è dovuto al fatto che il giunto reale ha verso opposto a quello di DH
//   q_left_pub_.QbAdvancePosDes_5 = q_L_(4) - defl_L_(4);
  
  for(int i=0; i<5; i++)
  {
    q_Eq_L_(i) = q_L_(i) - defl_L_(i);
    
    ref1.data[i] =  q_Eq_L_(i) + qpreset_L_;
    ref2.data[i] =  q_Eq_L_(i) - qpreset_L_;
  }


}

void only_gravity_comp::run()
{
  run_R();
  run_L();

  pub_ref1_.publish(ref1);
  pub_ref2_.publish(ref2);

}

int only_gravity_comp::sgn(double d)
{
    return d<0? -1 : d>0; 
}
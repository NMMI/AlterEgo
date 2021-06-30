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
 * \file      lqr.cpp
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
**/
// ----------------------------------------------------------------------------

#include <lqr.h>

lqr::lqr()
{
  //Topic you want to publish
  
  left_torque_pub_ = n_.advertise<std_msgs::Float64>("/ego/left_wheel_controller/command",1);
  right_torque_pub_ = n_.advertise<std_msgs::Float64>("/ego/right_wheel_controller/command",1);

  //Topic you want to subscribe
  sub_gyro_= n_.subscribe("/imu", 1, &lqr::callback_gyro, this);				//gyro measurements 
  sub_euler_ = n_.subscribe("/gazebo/link_states", 1, &lqr::callback_imu_euler, this);		//we use only pitch for self balance
  
  sub_enc = n_.subscribe("/ego/joint_states", 1, &lqr::callback_meas, this);			//mase encoder motor
  sub_pitch_off_ = n_.subscribe("/offset_phi", 1, &lqr::callback_offset_phi, this);		//offset pitch due to COM movement
  sub_des_vel = n_.subscribe("/segway_des_vel", 10, &lqr::callback_des_vel, this);		//reference linear and angular velocity

  flag_run1_ = flag_run2_ = flag_run3_ =  false;	

  pos1_des = 0.0;
  pos2_des = 0.0;
  vel1_des = 0.0;
  vel2_des = 0.0;

  offset_pitch_ = 0.01313;//-0.09;
  th_des_ = 0.0;
  dth_des_ = 0.0;
  phi_des_ = 0.0;
  dphi_des_ = 0.0;

  vel1 = 0;
  vel2 = 0;
  pos1 = 0; 
  pos2 = 0;
  
  R_ = 0.13;
  W_ = 0.496;
  N_ = 1/0.2727;
  
  std::cout<<"Raggio: "<< R_ << ", Distanza ruote: "<< W_<<", Rapporto di riduzione: "<< N_<<std::endl;
  std::cout<<"Il nodo che pubblica la vel_des e il sensore IR devono andare alla stessa freq del controllo, il filtraggio del rif vieni fatto in questo nodo "<<std::endl;

  dth_des_filt_ = 0.0;
  dth_des_filt_old_ = 0.0;
  
  int version = 0;
  if (!n_.getParam("AlterEgoVersion", version)) 
  {
    ROS_ERROR("Specify AlterEgo version");
    exit(1);
  }
  
  
  k_feed = Eigen::MatrixXd::Zero(2,6);
  if (version == 1)
  { 
    k_feed << -20.5,   -1816.9, -95.1, -250.8,  60.2236, 50.2114, 
	      -20.5,   -1816.9, -95.1, -250.8, -60.2236, -50.2114;
    
    trsh_L_ = 20;
    trsh_R_ = 20;
  }
  else if (version == 2)
  {
    k_feed <<  -8.0,   -909.8, -60, -168,  40.2236, 50.2114, 
	      -8.0,   -909.8, -60, -168, -40.2236, -50.2114;
    trsh_L_ = 25;
    trsh_R_ = 25;
  }

  max_dur_cmd_vel_ = ros::Duration(0.1);
  time_cmd_vel_ = ros::Time::now();
}

lqr::~lqr()
{

}

void lqr::callback_des_vel(const ego_msgs::EgoTwist2DUnicycle  & msg)
{	
  dth_des_ = msg.ForwardVelocity;
  dphi_des_ = msg.YawRate;


  if(fabs(dth_des_) > MAX_LIN_VEL) dth_des_ = sgn(dth_des_) * MAX_LIN_VEL;
  if(fabs(dphi_des_) > MAX_ANG_VEL) dphi_des_ = sgn(dphi_des_) * MAX_ANG_VEL;

  time_cmd_vel_ = ros::Time::now();
}

void lqr::callback_gyro(const sensor_msgs::Imu & msg)
{
  gyro_ << msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;
  flag_run1_ = true;

}

void lqr::callback_imu_euler(const gazebo_msgs::LinkStates & msg)
{
  flag_run2_ = true;
  const std::vector<std::string> &names = msg.name;
  
  for(int i= 0; i<names.size(); i++)
  {
    if(names[i] == "ego_robot::base_link")
    {
      double roll;
      KDL::Rotation::Quaternion(msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z, msg.pose[i].orientation.w).GetRPY(euler_(0), euler_(1), euler_(2));
    }
  }
}

void lqr::callback_meas(const sensor_msgs::JointState & msg)
{
  flag_run3_ = true;
  
  const std::vector<std::string> &names = msg.name;
  
  for(int i= 0; i<names.size(); i++)
  {
    if(names[i] == "L_joint_baseW")
    {
      pos2 = msg.position[i];
      vel2 = msg.velocity[i];
    }
    
    if(names[i] == "R_joint_baseW")
    {
      pos1 = msg.position[i];
      vel1 = msg.velocity[i];
    }
  }
}


void lqr::callback_offset_phi(const std_msgs::Float32::ConstPtr& msg)
{
  offset_pitch_ = msg->data;

}

void lqr::run()
{
  double  a, th, dth, phi, dphi;
  Eigen::VectorXd vel_wheels = Eigen::VectorXd::Zero(2); 

  double k_int;
  Eigen::MatrixXd command(2,1), command_feed(2,1);
  Eigen::MatrixXd state(6,1);

  k_int = -10.78;

  a = dt / (0.05 + dt);
    
  // dth = forward velocity * R

  th = 0.5 * (pos1 + pos2);
  dth = 0.5 * (vel1 + vel2);
  
  // phi = yaw angle 
  // dphi = yaw rate
  
  phi = (R_ / W_) * (pos1 - pos2);
  dphi = (R_ / W_) * (vel1 - vel2);
  
  vel_wheels(0) = vel1;
  vel_wheels(1) = vel2;
  
    ////////////////////////////////inizio filtraggio velocità///////////////////////////////////////////////////

  //senza IR
  dth_des_filt_ = dth_des_;

  dth_des_filt_ = (1 - 0.002) * dth_des_filt_old_ + (0.002 * dth_des_filt_);
  dth_des_filt_old_ = dth_des_filt_;

  ////////////////////////////////fine filtraggio velocità///////////////////////////////////////////////////
  
  th_des_ += dth_des_filt_ * dt;
  phi_des_ += dphi_des_ * dt;

  //controllo LQR
  state << (th_des_ - th),  0.0 - (euler_(1) + offset_pitch_), (dth_des_filt_ - dth), 0 - gyro_(1), (phi_des_ - phi), (dphi_des_ - dphi);
//   state << (th_des_ - th),  0.0 , (dth_des_filt_ - dth), 0, (phi_des_ - phi), (dphi_des_ - dphi);
  
  command_feed =  k_feed * state ;

  command_feed(1,0)  = command_feed(1,0) + trsh_L_*sgn(command_feed(1,0));
  command_feed(0,0) = command_feed(0,0)+ trsh_R_*sgn(command_feed(0,0));
  
  for (int i = 0; i < 2; i++)
  {
    if(command_feed(i,0) > 100)
      command_feed(i,0) = 100;
    else if(command_feed(i,0) < -100)
      command_feed(i,0) = -100;
  }
    
  command = (n*0.24) * Kt/Rm* (command_feed )*10.8/68.6;
    
  for (int i = 0; i < 2; i++)
  {
    if(command(i,0) > 11)
      command(i,0) = 11;
    else if(command(i,0) < -11)
      command(i,0) = -11;
  }

  std_msgs::Float64 msg_lw;
  msg_lw.data = command(1,0);
  
  left_torque_pub_.publish(msg_lw);
  
  std_msgs::Float64 msg_rw;
  msg_rw.data = command(0,0);
  
  right_torque_pub_.publish(msg_rw);
  
//   ROS_INFO_STREAM("Left Torque: " << command(1,0) << " (Nm)" << " Right Torque: " << command(0,0)<< " (Nm) " << "Pitch " << euler_(1) );
}


int lqr::sgn(double d)
{
  return d<0? -1 : d>0; 
}


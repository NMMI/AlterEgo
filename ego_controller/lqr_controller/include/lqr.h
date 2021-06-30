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
 * \file      lqr.h
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
**/
// ----------------------------------------------------------------------------


#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>

// ROS cuustom msg

#include <ego_msgs/EgoTwist2DUnicycle.h>
// #include <qb_interface/cubeRef.h>
// #include <qb_interface/cubeCurrent.h>
// #include <qb_interface/cubePos.h>


# define PI 3.1416
# define PI2 6.2832
# define MAX_LIN_VEL 2.0
# define MAX_ANG_VEL 0.5
# define Rm 0.84278
# define Kt 0.015056
# define Kb 0.048365
# define n 160



class lqr
{
public:
  lqr();
  ~lqr();


  void run();
  double dt;



private:

  void callback_gyro(const sensor_msgs::Imu & msg);
  void callback_imu_euler(const gazebo_msgs::LinkStates & msg);
  void callback_meas(const sensor_msgs::JointState & msg);
  void callback_des_vel(const ego_msgs::EgoTwist2DUnicycle  & msg);
  void callback_offset_phi(const std_msgs::Float32::ConstPtr& msg);
  
  int sgn(double d);

  double pos1_des;
  double pos2_des;
  double vel1_des;
  double vel2_des;
  double vel1, vel2, pos1, pos2;

  double offset_pitch_;

  ros::Subscriber sub_gyro_, sub_euler_, sub_des_vel, sub_enc, sub_pitch_off_;
  ros::Publisher left_torque_pub_ , right_torque_pub_;

  Eigen::Vector3d gyro_, euler_;
  double th_des_, dth_des_, phi_des_, dphi_des_;
  double R_, W_, N_;
  double trsh_L_, trsh_R_;
  bool flag_run1_, flag_run2_, flag_run3_;
  
  Eigen::MatrixXd k_feed;

  ros::NodeHandle n_;

  ros::Time time_cmd_vel_;
  ros::Duration max_dur_cmd_vel_;

  double dth_des_filt_, dth_des_filt_old_;
};//End of class SubscribeAndPublish

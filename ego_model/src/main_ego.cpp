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
 * \file      main_ego.cpp
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
 **/
// ----------------------------------------------------------------------------

#include "EgoModel.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ego_model");
  ros::NodeHandle node("~");

  // define an object of the class EgoModel
  EgoModel EgoModel(node);
  ros::Rate loop_rate(800);

  std::string sep = "\n----------------------------------------\n";
  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

  ros::spinOnce();

  if (EgoModel.acquire_base_ == 1 || EgoModel.sim_gazebo_ == 1)
  {
    while (EgoModel.state_acquired_ == 0 && ros::ok())
    {
      ros::spinOnce();
    }
  }

  EgoModel.setInitialConditions();
  ROS_INFO("initial x: %f", EgoModel.q_(0));

  while ((EgoModel.ra_torque_acquired_ == 0 || EgoModel.la_torque_acquired_ == 0 || EgoModel.torque_acquired_ == 0) && ros::ok())
  {
    EgoModel.sentRobotState();
    ros::spinOnce();
  }

  while (ros::ok())
  {
    EgoModel.updateRobotState(); // update position and velocity datas of Frank

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

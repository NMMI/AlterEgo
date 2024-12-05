/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>

using namespace visualization_msgs;

// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
geometry_msgs::Pose pre_update_pose_;
geometry_msgs::Pose post_update_pose_;
ros::Publisher pub_right_hand_pos;

// %EndTag(vars)%

// %Tag(Box)%
Marker makeBox(InteractiveMarker &msg)
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl &makeBoxControl(InteractiveMarker &msg)
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeBox(msg));
  msg.controls.push_back(control);

  return msg.controls.back();
}
// %EndTag(Box)%

// %Tag(processFeedback)%
void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
    << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;

  // ROS_INFO_STREAM( s.str() << ": pose changed"
  //     << "\nposition = "
  //     << feedback->pose.position.x
  //     << ", " << feedback->pose.position.y
  //     << ", " << feedback->pose.position.z
  //     << "\norientation = "
  //     << feedback->pose.orientation.w
  //     << ", " << feedback->pose.orientation.x
  //     << ", " << feedback->pose.orientation.y
  //     << ", " << feedback->pose.orientation.z
  //     << "\nframe: " << feedback->header.frame_id
  //     << " time: " << feedback->header.stamp.sec << "sec, "
  //     << feedback->header.stamp.nsec << " nsec" );

  pre_update_pose_ = feedback->pose;
  post_update_pose_ = pre_update_pose_;
  pub_right_hand_pos.publish(post_update_pose_);

  server->applyChanges();
}
// %EndTag(processFeedback)%

////////////////////////////////////////////////////////////////////////////////////

// %Tag(6DOF)%
void make6DofMarker(bool fixed, unsigned int interaction_mode, const tf::Vector3 &position, bool show_6dof)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "torso";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 0.1;

  int_marker.name = "simple_6dof";
  // int_marker.description = "Simple 6-DOF Control";

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  InteractiveMarkerControl control;

  if (fixed)
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
  {
    std::string mode_text;
    if (interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D)
      mode_text = "MOVE_3D";
    if (interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D)
      mode_text = "ROTATE_3D";
    if (interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D)
      mode_text = "MOVE_ROTATE_3D";
    int_marker.name += "_" + mode_text;
  }

  if (show_6dof)
  {
    tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    menu_handler.apply(*server, int_marker.name);
}
// %EndTag(6DOF)%

// %Tag(main)%
int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic_controls");
  ros::NodeHandle n;

  // create a timer to update the published transforms
  // ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);
  server.reset(new interactive_markers::InteractiveMarkerServer("basic_controls", "", false));
  pub_right_hand_pos = n.advertise<geometry_msgs::Pose>("right/hand_pos", 10);

  ros::Duration(0.1).sleep();
  tf::Vector3 position;
  position = tf::Vector3(0, 0, 0);

  tf::TransformListener listener;
  tf::StampedTransform transform;
  try
  {
    listener.waitForTransform("torso", "right_hand_curr", ros::Time(0), ros::Duration(10.0));
    listener.lookupTransform("torso", "right_hand_curr", ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  position = tf::Vector3(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());

  make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true);

  server->applyChanges();

  ros::spin();
  server.reset();
}
// %EndTag(main)%

#include <alterego_state_publisher.h>
// ----------------------------------------------------------------------------------------------------------
// Callback Functions to acquire the element of vector Q, to update it.
// ----------------------------------------------------------------------------------------------------------
void alterego_state_publisher::yaw_neck__Callback(const std_msgs::Float64::ConstPtr &msg)
{
  upperbody_state_msgs.left_meas_neck_shaft = (double)msg->data;
  if (std::isnan(upperbody_state_msgs.left_meas_neck_shaft)) {
    ROS_WARN("Received NaN value for left_meas_neck_shaft");
  }
}
// ----------------------------------------------------------------------------------------------------------

void alterego_state_publisher::pitch_neck__Callback(const std_msgs::Float64::ConstPtr &msg)
{

  upperbody_state_msgs.right_meas_neck_shaft = (double)msg->data;
}

// ----------------------------------------------------------------------------------------------------------

void alterego_state_publisher::left_arm__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  upperbody_state_msgs.left_meas_arm_shaft.clear();

  for (int i = 0; i < msg->data.size(); i++)
  {

    upperbody_state_msgs.left_meas_arm_shaft.push_back((double)msg->data[i]);
  }
  if (!acquire_meas_arm_l)
  {

    arm_L_q_addon_init = upperbody_state_msgs.left_meas_arm_shaft[1];
    // std::cout << "addon" << arm_L_q_addon_init << std::endl;
  }

  if (use_addon)
  {
    upperbody_state_msgs.left_meas_arm_shaft[1] = upperbody_state_msgs.left_meas_arm_shaft[1] - arm_L_q_addon_init;
  }

  acquire_meas_arm_l = true;
}

// ----------------------------------------------------------------------------------------------------------

void alterego_state_publisher::right_arm__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{

  upperbody_state_msgs.right_meas_arm_shaft.clear();

  for (int i = 0; i < msg->data.size(); i++)
  {

    upperbody_state_msgs.right_meas_arm_shaft.push_back((double)msg->data[i]);
  }

  if (!acquire_meas_arm_r)
  {
    arm_R_q_addon_init = upperbody_state_msgs.right_meas_arm_shaft[1];
  }

  if (use_addon)
  {
    upperbody_state_msgs.right_meas_arm_shaft[1] = upperbody_state_msgs.right_meas_arm_shaft[1] - arm_R_q_addon_init;
  }

  acquire_meas_arm_r = true;
}

// ----------------------------------------------------------------------------------------------------------

void alterego_state_publisher::left_m1__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  upperbody_state_msgs.left_meas_arm_m1.clear();

  for (int i = 0; i < msg->data.size(); i++)
  {
    upperbody_state_msgs.left_meas_arm_m1.push_back((double)msg->data[i]);
  }
  if (!acquire_meas_m1_l)
  {
    m1_L_q_addon_init = upperbody_state_msgs.left_meas_arm_m1[1];
  }

  upperbody_state_msgs.left_meas_arm_m1[1] = upperbody_state_msgs.left_meas_arm_m1[1] - m1_L_q_addon_init;
  acquire_meas_m1_l = true;
}

// ----------------------------------------------------------------------------------------------------------

void alterego_state_publisher::right_m1__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  upperbody_state_msgs.right_meas_arm_m1.clear();

  for (int i = 0; i < msg->data.size(); i++)
  {
    upperbody_state_msgs.right_meas_arm_m1.push_back((double)msg->data[i]);
  }

  if (!acquire_meas_m1_r)
  {
    m1_R_q_addon_init = upperbody_state_msgs.right_meas_arm_m1[1];
  }

  upperbody_state_msgs.right_meas_arm_m1[1] = upperbody_state_msgs.right_meas_arm_m1[1] - m1_R_q_addon_init;
  acquire_meas_m1_r = true;
}

// ----------------------------------------------------------------------------------------------------------

void alterego_state_publisher::left_m2__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  upperbody_state_msgs.left_meas_arm_m2.clear();

  for (int i = 0; i < msg->data.size(); i++)
  {
    upperbody_state_msgs.left_meas_arm_m2.push_back((double)msg->data[i]);
  }
  if (!acquire_meas_m2_l)
  {
    m2_L_q_addon_init = upperbody_state_msgs.left_meas_arm_m2[1];
  }

  upperbody_state_msgs.left_meas_arm_m2[1] = upperbody_state_msgs.left_meas_arm_m2[1] - m2_L_q_addon_init;
  acquire_meas_m2_l = true;
}

// ----------------------------------------------------------------------------------------------------------

void alterego_state_publisher::right_m2__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  upperbody_state_msgs.right_meas_arm_m2.clear();

  for (int i = 0; i < msg->data.size(); i++)
  {
    upperbody_state_msgs.right_meas_arm_m2.push_back((double)msg->data[i]);
  }

  if (!acquire_meas_m2_r)
  {
    m2_R_q_addon_init = upperbody_state_msgs.right_meas_arm_m2[1];
  }

  upperbody_state_msgs.right_meas_arm_m2[1] = upperbody_state_msgs.right_meas_arm_m2[1] - m2_R_q_addon_init;
  acquire_meas_m2_r = true;
}

// ----------------------------------------------------------------------------------------------------------

void alterego_state_publisher::right_weight__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  upperbody_state_msgs.right_weight = msg->data[0];
}
// ----------------------------------------------------------------------------------------------------------

void alterego_state_publisher::left_weight__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  upperbody_state_msgs.left_weight = msg->data[0];
}


// ----------------------------------------------------------------------------------------------------------*
//                                                                                                           *
//                                      LowerBody Callbacks                                                  *
//                                                                                                           *
// ----------------------------------------------------------------------------------------------------------*

void alterego_state_publisher::pitch_angle__Callback(const geometry_msgs::Vector3::ConstPtr &msg)
{
  pitch_angle = msg->y;
}



// ----------------------------------------------------------------------------------------------------------
void alterego_state_publisher::pitch_rate__Callback(const geometry_msgs::Vector3::ConstPtr &msg)
{
  pitch_rate = msg->y;
}
// ----------------------------------------------------------------------------------------------------------

void alterego_state_publisher::meas_encoders__Callback(const qb_interface::cubePos::ConstPtr &msg)
{
  switch(enc_state)
  {
    case SETOFFSET:
      if (!std::isnan(msg->p_1[0]) && !std::isnan(msg->p_2[0]))
      {
        left_wheel_enc = msg->p_1[0] * 4.0 * PI / 65536; // converto da tick a radianti
        right_wheel_enc = msg->p_2[0] * 4.0 * PI / 65536;
        encL_of_ = left_wheel_enc ;
        encR_of_ = right_wheel_enc ;
        left_wheel_enc = 0;
        right_wheel_enc = 0;

        enc_state = GETMEASUREMENT;
      }
    break;
    case GETMEASUREMENT:
      if (!std::isnan(msg->p_1[0]) && !std::isnan(msg->p_2[0]))
      {
        left_wheel_enc = msg->p_1[0] * 4.0 * PI / 65536 - encL_of_;
        right_wheel_enc = msg->p_2[0] * 4.0 * PI / 65536 - encR_of_;
        base_battery_voltage = msg->p_L[0];
      }
    break;
  }
  last_time_encoders = ros::Time::now();

}
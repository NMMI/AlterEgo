#include <alterego_state_publisher.h>
alterego_state_publisher::alterego_state_publisher(double rate) : flag_first_enc_(false)
{

  ros::NodeHandle n;
  std::string robot_name = std::getenv("ROBOT_NAME");
  std::string ns;
  ns = ros::this_node::getNamespace();
  AlterEgoVersion = 3;
  n.getParam("/" + robot_name + "/use_addon", use_addon);
  n.getParam("/" + robot_name + "/right/a_mot", right_a_mot_);
  n.getParam("/" + robot_name + "/left/a_mot", left_a_mot_);
  n.getParam("/" + robot_name + "/right/k_mot", right_k_mot_);
  n.getParam("/" + robot_name + "/left/k_mot", left_k_mot_);
  n.getParam("/" + robot_name + "/AlterEgoVersion", AlterEgoVersion);
  if (AlterEgoVersion == 3)
  {
    n_act_arm = 6;
  }
  else
  {
    n_act_arm = 5;
  }
  std::string left_meas_arm_shaft;
  std::string left_meas_neck_shaft;
  std::string left_meas_arm_m1;
  std::string left_meas_arm_m2;
  std::string left_wheel_encoder;

  std::string right_meas_arm_shaft;
  std::string right_meas_neck_shaft;
  std::string right_meas_arm_m1;
  std::string right_meas_arm_m2;
  std::string right_wheel_encoder;

  std::string base_lin_vel;
  std::string base_ang_vel;

  std::string RPY;
  std::string right_weight;
  std::string left_weight;

  std::string encoder;
  std::string gyroscope;

  dt = 1 / rate;
  n.getParam("left_meas_arm_shaft", left_meas_arm_shaft);
  n.getParam("left_meas_neck_shaft", left_meas_neck_shaft);
  n.getParam("left_meas_arm_m1", left_meas_arm_m1);
  n.getParam("left_meas_arm_m2", left_meas_arm_m2);
  n.getParam("left_wheel_encoder", left_wheel_encoder);

  n.getParam("right_meas_arm_shaft", right_meas_arm_shaft);
  n.getParam("right_meas_neck_shaft", right_meas_neck_shaft);
  n.getParam("right_meas_arm_m1", right_meas_arm_m1);
  n.getParam("right_meas_arm_m2", right_meas_arm_m2);
  n.getParam("right_wheel_encoder", right_wheel_encoder);

  n.getParam("encoder", encoder);
  n.getParam("gyroscope", gyroscope);
  n.getParam("base_lin_vel", base_lin_vel);
  n.getParam("base_ang_vel", base_ang_vel);
  n.getParam("RPY", RPY);
  n.getParam("left_weight", left_weight);
  n.getParam("right_weight", right_weight);
  // ---------------------------------------------------------------------------------------------------------- Subscribers

  // ---------------------------------------------------------------------------------------------------------- SHAFT ARM AND NECK
  arm_Q_left = nh.subscribe(left_meas_arm_shaft, 1, &alterego_state_publisher::left_arm__Callback, this);
  arm_Q_right = nh.subscribe(right_meas_arm_shaft, 1, &alterego_state_publisher::right_arm__Callback, this);
  motor_m1_left = nh.subscribe(left_meas_arm_m1, 1, &alterego_state_publisher::left_m1__Callback, this);
  motor_m1_right = nh.subscribe(right_meas_arm_m1, 1, &alterego_state_publisher::right_m1__Callback, this);
  motor_m2_left = nh.subscribe(left_meas_arm_m2, 1, &alterego_state_publisher::left_m2__Callback, this);
  motor_m2_right = nh.subscribe(right_meas_arm_m2, 1, &alterego_state_publisher::right_m2__Callback, this);
  neck_Q_yaw = nh.subscribe(left_meas_neck_shaft, 1, &alterego_state_publisher::yaw_neck__Callback, this);
  neck_Q_pitch = nh.subscribe(right_meas_neck_shaft, 1, &alterego_state_publisher::pitch_neck__Callback, this);

  // LowerBody Subscribers
  sub_meas_encoders_ = nh.subscribe(encoder, 1, &alterego_state_publisher::meas_encoders__Callback, this); 
  sub_pitch_rate_ = nh.subscribe(gyroscope, 1, &alterego_state_publisher::pitch_rate__Callback, this);
  sub_pitch_angle_ = nh.subscribe(RPY, 1, &alterego_state_publisher::pitch_angle__Callback, this);

  // WEIGHT
  // left_weight_sub = nh.subscribe(left_weight, 1, &alterego_state_publisher::callback_left_weight, this);
  // right_weight_sub = nh.subscribe(right_weight, 1, &alterego_state_publisher::callback_right_weight, this);

  // ----------------------------------------------------------------------------------------------------------
  arm_L_q_addon_init = 0.0;
  arm_R_q_addon_init = 0.0;
  encL_old_ = 0.0;
  encR_old_ = 0.0;
  velL_enc = 0.0;
  velR_enc = 0.0;

  linear_vel_real = 0;
  angular_vel_real = 0;

  acquire_meas_arm_l = false;
  acquire_meas_arm_r = false;

  acquire_meas_m1_l = false;
  acquire_meas_m1_r = false;

  acquire_meas_m2_l = false;
  acquire_meas_m2_r = false;

  encL_ = 0;
  encR_ = 0;
  encL_old_ = 0;
  encR_old_ = 0;
  encL_of_ = 0;
  encR_of_ = 0;
  velL_old_ = 0;
  velR_old_ = 0;
  vel_old_ = 0;
  w_old_ = 0;
  arm_L_q_addon_init = 0;
  arm_R_q_addon_init = 0;

  //---------------------------------------------------------------------------------------------------------- Publishers
  pub_lowerbody_ = nh.advertise<alterego_msgs::LowerBodyState>("alterego_state/lowerbody", 1);
  pub_upperbody_ = nh.advertise<alterego_msgs::UpperBodyState>("alterego_state/upperbody", 1);

  //---------------------------------------------------------------------------------------------------------- Init msgs
  upperbody_state_msgs.right_meas_neck_shaft = 0;
  upperbody_state_msgs.left_meas_neck_shaft = 0;
  upperbody_state_msgs.right_meas_arm_m1.clear();
  upperbody_state_msgs.right_meas_arm_m2.clear();
  upperbody_state_msgs.right_meas_arm_shaft.clear();
  upperbody_state_msgs.left_meas_arm_m1.clear();
  upperbody_state_msgs.left_meas_arm_m2.clear();
  upperbody_state_msgs.left_meas_arm_shaft.clear();
  upperbody_state_msgs.right_q_eq.clear();
  upperbody_state_msgs.left_q_eq.clear();
  upperbody_state_msgs.right_q_preset.clear();
  upperbody_state_msgs.left_q_preset.clear();
  upperbody_state_msgs.right_q_tau_link_elastic.clear();
  upperbody_state_msgs.left_q_tau_link_elastic.clear();
  upperbody_state_msgs.right_stiff_link.clear();
  upperbody_state_msgs.left_stiff_link.clear();

  gyro_ << 0, 0, 0;
  enc_filter_coeff = dt / (0.05 + dt);

  R_ = 0.125;
  W_ = 0.55;
  N_ = 1 / 0.31;

  max_dur_encoders = ros::Duration(0.5);

}

// Deconstructor:

alterego_state_publisher::~alterego_state_publisher()
{
}
  //---------------------------------------------------------------------------------------------------------- Init function

void alterego_state_publisher::Init(){
  base_battery_voltage = 0;
  right_wheel_enc = 0;
  left_wheel_enc = 0;
  right_wheel_pos = 0;
  left_wheel_pos = 0;
  wheels_angular_pos = 0;
  right_wheel_vel = 0;
  left_wheel_vel = 0;
  wheels_angular_vel = 0;
  mobile_base_pos_x = 0;
  mobile_base_pos_y = 0;
  mobile_base_lin_displacement = 0;
  mobile_base_lin_vel = 0;
  yaw_angle = 0;
  yaw_rate = 0;
  mobile_base_ang_vel = 0;
  encL_ = 0;
  encR_ = 0;
  encL_old_ = 0;
  encR_old_ = 0;
  encL_of_ = 0;
  encR_of_ = 0;
  velL_old_ = 0;
  velR_old_ = 0;
  vel_old_ = 0;
  
  lowerbody_state_msgs.base_battery_voltage = 0;
  lowerbody_state_msgs.right_wheel_enc = 0;
  lowerbody_state_msgs.left_wheel_enc = 0;
  lowerbody_state_msgs.right_wheel_pos = 0;
  lowerbody_state_msgs.left_wheel_pos = 0;
  lowerbody_state_msgs.wheels_angular_pos = 0;
  lowerbody_state_msgs.right_wheel_vel = 0;
  lowerbody_state_msgs.left_wheel_vel = 0;
  lowerbody_state_msgs.wheels_angular_vel = 0;
  lowerbody_state_msgs.mobile_base_pos_x = 0;
  lowerbody_state_msgs.mobile_base_pos_y = 0;
  lowerbody_state_msgs.mobile_base_lin_displacement = 0;
  lowerbody_state_msgs.mobile_base_lin_vel = 0;
  lowerbody_state_msgs.yaw_angle = 0;
  lowerbody_state_msgs.yaw_rate = 0;
  lowerbody_state_msgs.mobile_base_ang_vel = 0;



}
double alterego_state_publisher::angleDiff(double a, double b)
{
  double dif = std::fmod(b - a + PI, PI2);
  if (dif < 0)
    dif += PI2;
  return dif - PI;
}
double alterego_state_publisher::unwrap(double previousAngle, double newAngle)
{
  return previousAngle - angleDiff(newAngle, previousAngle);
}


double lowPassFilter(double input, double alpha, double& prevOutput, bool& initialized) {
    if (!initialized) {
        // Initialize previous output with the first input
        prevOutput = input;
        initialized = true;
    } else {
        // Update output using the low-pass filter formula
        prevOutput = alpha * input + (1.0 - alpha) * prevOutput;
    }

    return prevOutput;
}
// ----------------------------------------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------------------------------------

/// @brief Function to update upperbody state and publish the robot state in real time:
void alterego_state_publisher::UpperBodyState()
{

  double temp_tau_el = 0;
  double temp_stiff = 0;
  double temp_gamma = 0;
  /**/

  if (acquire_meas_arm_l && acquire_meas_arm_r && acquire_meas_m1_l &&acquire_meas_m1_r && acquire_meas_m2_l &&acquire_meas_m2_r )
  {
    upperbody_state_msgs.right_q_eq.clear();
    upperbody_state_msgs.left_q_eq.clear();
    upperbody_state_msgs.right_q_preset.clear();
    upperbody_state_msgs.left_q_preset.clear();
    upperbody_state_msgs.right_q_tau_link_elastic.clear();
    upperbody_state_msgs.left_q_tau_link_elastic.clear();
    upperbody_state_msgs.right_stiff_link.clear();
    upperbody_state_msgs.left_stiff_link.clear();
    upperbody_state_msgs.right_gamma.clear();
    upperbody_state_msgs.left_gamma.clear();
    int start_from = 0;
    if (AlterEgoVersion == 3)
    {
      use_addon = true;
      start_from = 1;
      upperbody_state_msgs.right_q_preset.push_back(0.0);
      upperbody_state_msgs.left_q_preset.push_back(0.0);

      upperbody_state_msgs.right_q_eq.push_back(0.0);
      upperbody_state_msgs.left_q_eq.push_back(0.0);

      upperbody_state_msgs.right_stiff_link.push_back(0.0);
      upperbody_state_msgs.left_stiff_link.push_back(0.0);

      upperbody_state_msgs.right_q_tau_link_elastic.push_back(0.0);
      upperbody_state_msgs.left_q_tau_link_elastic.push_back(0.0);

      upperbody_state_msgs.right_gamma.push_back(0.0);
      upperbody_state_msgs.left_gamma.push_back(0.0);
    }


    for (int i = start_from; i < upperbody_state_msgs.left_meas_arm_m1.size(); i++)
    {
      upperbody_state_msgs.right_q_eq.push_back((upperbody_state_msgs.right_meas_arm_m1[i] + upperbody_state_msgs.right_meas_arm_m2[i]) / 2);
      upperbody_state_msgs.right_q_preset.push_back((upperbody_state_msgs.right_meas_arm_m1[i] - upperbody_state_msgs.right_meas_arm_m2[i]) / 2);

      upperbody_state_msgs.left_q_eq.push_back((upperbody_state_msgs.left_meas_arm_m1[i] + upperbody_state_msgs.left_meas_arm_m2[i]) / 2);
      upperbody_state_msgs.left_q_preset.push_back((upperbody_state_msgs.left_meas_arm_m1[i] - upperbody_state_msgs.left_meas_arm_m2[i]) / 2);

      // tau_el
      // tau_el = k1*sinh(a1*defl1) + k2*sinh(a2*defl2);
      temp_tau_el = (right_k_mot_[i] * sinh(right_a_mot_[i] * (upperbody_state_msgs.right_meas_arm_shaft[i] - upperbody_state_msgs.right_meas_arm_m1[i])) + right_k_mot_[i] * sinh(right_a_mot_[i] * (upperbody_state_msgs.right_meas_arm_shaft[i] - upperbody_state_msgs.right_meas_arm_m2[i])));
      upperbody_state_msgs.right_q_tau_link_elastic.push_back(temp_tau_el);

      temp_tau_el = (left_k_mot_[i] * sinh(left_a_mot_[i] * (upperbody_state_msgs.left_meas_arm_shaft[i] - upperbody_state_msgs.left_meas_arm_m1[i])) + left_k_mot_[i] * sinh(left_a_mot_[i] * (upperbody_state_msgs.left_meas_arm_shaft[i] - upperbody_state_msgs.left_meas_arm_m2[i])));
      upperbody_state_msgs.left_q_tau_link_elastic.push_back(temp_tau_el);

      // link stiffness
      // sigmaL_out = a1*k1*cosh(a1*defl1) + a2*k2*cosh(a2*defl2);
      // temp_stiff = (right_a_mot_ * right_k_mot_ * cosh(right_a_mot_ * (upperbody_state_msgs.right_meas_arm_shaft[i] - upperbody_state_msgs.right_meas_arm_m1[i])) + right_a_mot_ * right_k_mot_ * cosh(right_a_mot_ * (upperbody_state_msgs.right_meas_arm_shaft[i] - upperbody_state_msgs.right_meas_arm_m2[i])));

      temp_stiff = (right_a_mot_[i] * right_k_mot_[i] * cosh(right_a_mot_[i] * (upperbody_state_msgs.right_meas_arm_shaft[i] - upperbody_state_msgs.right_meas_arm_m1[i])) + right_a_mot_[i] * right_k_mot_[i] * cosh(right_a_mot_[i] * (upperbody_state_msgs.right_meas_arm_shaft[i] - upperbody_state_msgs.right_meas_arm_m2[i])));
      upperbody_state_msgs.right_stiff_link.push_back(temp_stiff);

      temp_stiff = (left_a_mot_[i] * left_k_mot_[i] * cosh(left_a_mot_[i] * (upperbody_state_msgs.left_meas_arm_shaft[i] - upperbody_state_msgs.left_meas_arm_m1[i])) + left_a_mot_[i] * left_k_mot_[i] * cosh(left_a_mot_[i] * (upperbody_state_msgs.left_meas_arm_shaft[i] - upperbody_state_msgs.left_meas_arm_m2[i])));
      upperbody_state_msgs.left_stiff_link.push_back(temp_stiff);

      // gamma. is the derivative of tau_el with respect to q_preset
      // using Prosthaphaeresis tau_el(q_eq,q_preset) = 2*k*sinh(a*(q-q_eq))*cosh(a*q_preset)
      // gamma = 2*a*k*sinh(a*(q-q_eq))*sinh(a*q_preset)
      temp_gamma = 2 * right_a_mot_[i] * right_k_mot_[i] * sinh(right_a_mot_[i] * (upperbody_state_msgs.right_meas_arm_shaft[i] * upperbody_state_msgs.right_q_eq[i])) * sinh(right_a_mot_[i] * upperbody_state_msgs.right_q_preset[i]);
      upperbody_state_msgs.right_gamma.push_back(temp_gamma);

      temp_gamma = 2 * left_a_mot_[i] * left_k_mot_[i] * sinh(left_a_mot_[i] * (upperbody_state_msgs.left_meas_arm_shaft[i] * upperbody_state_msgs.left_q_eq[i])) * sinh(left_a_mot_[i] * upperbody_state_msgs.left_q_preset[i]);
      upperbody_state_msgs.left_gamma.push_back(temp_gamma);
    }
  }
  else
  {
    upperbody_state_msgs.right_q_preset.assign(n_act_arm, 0.0);
    upperbody_state_msgs.left_q_preset.assign(n_act_arm, 0.0);

    upperbody_state_msgs.right_q_eq.assign(n_act_arm, 0.0);
    upperbody_state_msgs.left_q_eq.assign(n_act_arm, 0.0);

    upperbody_state_msgs.right_stiff_link.assign(n_act_arm, 0.0);
    upperbody_state_msgs.left_stiff_link.assign(n_act_arm, 0.0);

    upperbody_state_msgs.right_q_tau_link_elastic.assign(n_act_arm, 0.0);
    upperbody_state_msgs.left_q_tau_link_elastic.assign(n_act_arm, 0.0);

    upperbody_state_msgs.right_gamma.assign(n_act_arm, 0.0);
    upperbody_state_msgs.left_gamma.assign(n_act_arm, 0.0);

    upperbody_state_msgs.right_meas_arm_shaft.assign(n_act_arm, 0.0);
    upperbody_state_msgs.left_meas_arm_shaft.assign(n_act_arm, 0.0);

    upperbody_state_msgs.right_meas_arm_m1.assign(n_act_arm, 0.0);
    upperbody_state_msgs.left_meas_arm_m1.assign(n_act_arm, 0.0);

    upperbody_state_msgs.right_meas_arm_m2.assign(n_act_arm, 0.0);
    upperbody_state_msgs.left_meas_arm_m2.assign(n_act_arm, 0.0);
  }
}

/// @brief Function to calculate everytime the position (x,y) of the unicycle (mobile base)
void alterego_state_publisher::LowerBodyState()
{
  //Reset Encoder state
  if ((ros::Time::now() - last_time_encoders) > max_dur_encoders)
  {
    Init();
    enc_state = SETOFFSET;
  }

  //aggiungere sleep se non leggo piu gli encoders
  if(enc_state == GETMEASUREMENT){
    encL_g = unwrap(encL_old_, left_wheel_enc);
    // std::cout << "\n[encL] old:\t" << encL_old_ << " curr:\t" << encL_g;
    encR_g = unwrap(encR_old_, right_wheel_enc);

    velL_enc = (encL_g - encL_old_) / dt;
    // std::cout << "\n[v_encL]\t vel1_enc evaluated:\t" << velL_enc << " ";
    velR_enc = (encR_g - encR_old_) / dt;
    // std::cout << " \n encL_g:\t" << pitch_rate << " \tencR_g:\t" << encR_g;

    encL_old_ = encL_g;
    encR_old_ = encR_g;
    // posizione/velocità assoluta ruote pos1 -> left , pos2->right  tenendo conto del rapporto di riduzione N_
    posL = pitch_angle - (encL_g / N_);
    posR = pitch_angle + (encR_g / N_);

    // std::cout << "\n[ASP]pitch_rate_:\t" << pitch_rate;
    velL = pitch_rate - (velL_enc / N_);
    velR = pitch_rate + (velR_enc / N_);

    velL = (1 - enc_filter_coeff) * velL_old_ + (enc_filter_coeff * velL);
    velR = (1 - enc_filter_coeff) * velR_old_ + (enc_filter_coeff * velR);

    velL_old_ = velL;
    velR_old_ = velR;
    //Wheels angular pos
    right_wheel_pos = posR;
    left_wheel_pos = posL;
    wheels_angular_vel =  0.5 * (velR + velL); //mean value

    //Wheels angular vel
    left_wheel_vel = velL;
    right_wheel_vel = velR;
    wheels_angular_pos =  0.5 * (posL + posR);  //mean value
    
    //YAW
    yaw_angle = (R_ / W_) * (posL - posR);
    yaw_rate = (R_ / W_) * (velL - velR);

    //absolute pos and vel
    mobile_base_lin_vel = R_ * 0.5 * (velR + velL); // R * qd_wheels_tot
    mobile_base_ang_vel = (R_ / W_) * (velR - velL);

    mobile_base_pos_x = mobile_base_lin_vel * cos(yaw_angle) * 0.0025 + mobile_base_pos_x; //(dt=1/f=1/400=0.0025)
    mobile_base_pos_y = mobile_base_lin_vel * sin(yaw_angle) * 0.0025 + mobile_base_pos_y;
    mobile_base_lin_displacement = mobile_base_lin_vel * dt + mobile_base_lin_displacement;
  
  }

}


void alterego_state_publisher::Publish()
{

  lowerbody_state_msgs.base_battery_voltage = base_battery_voltage;
  lowerbody_state_msgs.right_wheel_enc = right_wheel_enc;
  lowerbody_state_msgs.left_wheel_enc = left_wheel_enc;
  lowerbody_state_msgs.right_wheel_pos = right_wheel_pos;
  lowerbody_state_msgs.left_wheel_pos = left_wheel_pos;
  lowerbody_state_msgs.wheels_angular_pos = wheels_angular_pos;
  lowerbody_state_msgs.right_wheel_vel = right_wheel_vel;
  lowerbody_state_msgs.left_wheel_vel = left_wheel_vel;
  lowerbody_state_msgs.wheels_angular_vel = wheels_angular_vel;
  lowerbody_state_msgs.mobile_base_pos_x = mobile_base_pos_x;
  lowerbody_state_msgs.mobile_base_pos_y = mobile_base_pos_y;
  lowerbody_state_msgs.mobile_base_lin_displacement = mobile_base_lin_displacement;
  lowerbody_state_msgs.mobile_base_lin_vel = mobile_base_lin_vel;
  lowerbody_state_msgs.yaw_angle = yaw_angle;
  lowerbody_state_msgs.yaw_rate = yaw_rate;
  lowerbody_state_msgs.mobile_base_ang_vel = mobile_base_ang_vel;
  lowerbody_state_msgs.pitch_angle = pitch_angle;
  lowerbody_state_msgs.pitch_rate = pitch_rate;
  
  //LowerBodyState Publishers
  // if(enc_state == GETMEASUREMENT)
  pub_lowerbody_.publish(lowerbody_state_msgs);
  pub_upperbody_.publish(upperbody_state_msgs);

}
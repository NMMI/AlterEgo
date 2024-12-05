#include <lqr.h>

void lqr::R_Botton_A__Callback(const std_msgs::Bool::ConstPtr &msg)
{

	static int last_msg(0);

	if (msg->data == true && last_msg == 0)
	{
		if (enabled_pilot == 1)
		{
			enabled_pilot = 0;
			std::cout << "control disabled" << std::endl;
		}
		else
		{
			enabled_pilot = 1;
			std::cout << "control enabled" << std::endl;
		}
	}

	if (msg->data == false)
		last_msg = 0;
	else
		last_msg = 1;

	last_cmd_time = ros::Time::now();
}

void lqr::Arms_Compliant__Callback(const std_msgs::Bool::ConstPtr &msg)
{

	if (msg->data == true)
	{
		enable_arms_compliant_control = 1;
	}

	last_cmd_time_arms_compliant_control = ros::Time::now();
}

void lqr::FallBack__Callback(const std_msgs::Bool::ConstPtr &msg)
{
	if (has_fallback_)
	{
		if (msg->data && !fallback_)
		{
			time_first_cmd_fall_ = ros::Time::now();
			state_fallback_ = 0;
		}
		else if (!msg->data && fallback_)
		{
			des_wheels_angular_pos_ = lowerbody_state_msg_.wheels_angular_pos;
			des_yaw_ = lowerbody_state_msg_.yaw_angle;
		}

		fallback_ = msg->data;
	}
}

void lqr::callback_des_vel(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
	if (!isnan(msg->twist.linear.x))
	{
		des_wheels_angular_vel_ = msg->twist.linear.x;
	}
	if (!isnan(msg->twist.angular.z))
	{
		des_yaw_rate_ = -msg->twist.angular.z;
	}

	// Saturation
	if (fabs(des_wheels_angular_vel_) > MAX_LIN_VEL)
		des_wheels_angular_vel_ = sgn(des_wheels_angular_vel_) * MAX_LIN_VEL;
	if (fabs(des_yaw_rate_) > MAX_ANG_VEL)
		des_yaw_rate_ = sgn(des_yaw_rate_) * MAX_ANG_VEL;

	time_cmd_vel_ = ros::Time::now();
	enabled_pilot = true;
}

void lqr::callback_offset_phi(const std_msgs::Float32::ConstPtr &msg)
{
	offset_pitch_ = msg->data;
}

void lqr::lowerbody_state__Callback(const ego_msgs::LowerBodyState::ConstPtr &msg)
{
	lowerbody_state_msg_ = *msg;
	flag_run1_ = true;
}
lqr::lqr()
{
	// ---------------------------------------------------------------------------------------------------Init
	robot_name = std::getenv("ROBOT_NAME");
	k_feed_from_yaml = {0};
	old_t_ = ros::Time::now();
	com_R = 0.0;
	com_L = 0.0;
	enabled_pilot = false;
	enable_arms_compliant_control = false;
	flag_run1_ = flag_run2_ = flag_run3_ = false;
	pos1_des = 0.0;
	pos2_des = 0.0;
	vel1_des = 0.0;
	vel2_des = 0.0;
	command_int = 0.0;
	offset_pitch_ = 0.0;
	sensor_IR_ = Eigen::VectorXd::Zero(8);
	arr_sensor_ = Eigen::VectorXd::Zero(10);
	des_wheels_angular_pos_ = 0.0;
	des_wheels_angular_vel_ = 0.0;
	des_yaw_ = 0.0;
	des_yaw_rate_ = 0.0;
	sensor_2_ = 0.0;
	enc1_ = 0;
	enc2_ = 0;
	enc1_old_ = 0;
	enc2_old_ = 0;
	vel1_old_ = 0;
	vel2_old_ = 0;
	enc1_g = 0.0;
	enc2_g = 0.0;
	vel1_enc = 0.0;
	vel2_enc = 0.0;
	vel1 = 0.0;
	vel2 = 0.0;
	pos1 = 0.0;
	pos2 = 0.0;
	a = 0.0;

	x_pre_ = Eigen::Vector2d::Zero();
	x_old_ = Eigen::Vector2d::Zero();
	P_pre_ = Eigen::Matrix2d::Zero();
	P_old_ << 0.0001, 0, 0, 0.0001;
	des_wheels_angular_vel_filt_ = 0.0;
	des_wheels_angular_vel_filt_old_ = 0.0;
	dphi_des_filt_ = 0.0;
	dphi_des_filt_old_ = 0.0;
	k_feed = Eigen::MatrixXd(2, 6);
	command = Eigen::MatrixXd(2, 1);
	command_feed = Eigen::MatrixXd(2, 1);
	state = Eigen::MatrixXd(6, 1);
	R_ = 0.125;
	W_ = 0.55;
	N_ = 1 / 0.31;
	max_dur_cmd_vel_ = ros::Duration(0.5);
	fallback_ = false;
	has_fallback_ = false;
	state_fallback_ = 0;
	th_curr_ = 0;
	des_yaw_rate_filt_old_ = 0.0;
	des_yaw_rate_filt_ = 0.0;
	phi_curr_ = 0;

	// ---------------------------------------------------------------------------------------------------GetParam

	n_.param("wheel_thr_L", trsh_L_, 0.0);
	n_.param("wheel_thr_R", trsh_R_, 0.0);
	n_.getParam("k_feed", k_feed_from_yaml);
	n_.param("/" + robot_name + "/AlterEgoVersion", AlterEgoVersion, 2);
	n_.param<std::string>("/" + robot_name + "/IN_topic_arms_compliant_control", IN_topic_button_dance, "IN_topic_arms_compliant_control");
	n_.param("/" + robot_name + "/SimulatedEgo", sim, false);
	n_.param("/" + robot_name + "/has_fallback", has_fallback_, false);
	// if (sim)
	// 	n_.getParam("standing_pitch_offset_simulated", offset_pitch_);
	// else
	n_.getParam("standing_pitch_offset", offset_pitch_);

	// ---------------------------------------------------------------------------------------------------Publisher

	pub_comm_ = n_.advertise<qb_interface::cubeRef>("qb_interface_node/qb_class/cube_ref", 1); // base motor ref (duty cycle [-100,100])
	pub_debug_ = n_.advertise<ego_msgs::DebugLQR>("DebugLQR", 1);

	// ---------------------------------------------------------------------------------------------------Subscriber

	sub_pitch_off_ = n_.subscribe("/" + robot_name + "/offset_phi", 1, &lqr::callback_offset_phi, this); // offset pitch due to COM movement
	sub_des_vel = n_.subscribe("segway_des_vel", 1, &lqr::callback_des_vel, this);						 // reference linear and angular velocity
	sub_lowerbody_state_ = n_.subscribe("/" + robot_name + "/alterego_state/lowerbody", 1, &lqr::lowerbody_state__Callback, this);
	if(has_fallback_) sub_fallback = n_.subscribe("fallback", 1, &lqr::FallBack__Callback, this);

	// ---------------------------------------------------------------------------------------------------Check if it is not a simulator -> start with 0 torque
	comm_pub_.p_1.push_back(com_R);
	comm_pub_.p_2.push_back(com_L);

	ros::Rate loop(10);
	ros::Duration init(3.0);
	ros::Time start = ros::Time::now();

	while ((ros::Time::now() - start < init) && (!sim))
	{
		loop.sleep();
		pub_comm_.publish(comm_pub_);
	}
	// ---------------------------------------------------------------------------------------------------Compose K_feeds

	k_feed << k_feed_from_yaml[0], k_feed_from_yaml[1], k_feed_from_yaml[2], k_feed_from_yaml[3],
		k_feed_from_yaml[4], k_feed_from_yaml[5], k_feed_from_yaml[6], k_feed_from_yaml[7],
		k_feed_from_yaml[8], k_feed_from_yaml[9], k_feed_from_yaml[10], k_feed_from_yaml[11];

	time_cmd_vel_ = ros::Time::now();
	last_cmd_time = ros::Time::now();
	std::cout << "\n ******* start_control **** \n";
	std::cout << "K_feed  \n"
			  << k_feed << std::endl;
	std::cout << "Trsh_L_: " << trsh_L_ << std::endl;
	std::cout << "Trsh_R_: " << trsh_R_ << std::endl;
	std::cout << "AlterEgoVersion: " << AlterEgoVersion << std::endl;
	std::cout << "Offset_pitch_: " << offset_pitch_ << std::endl;
}

lqr::~lqr()
{
}

void lqr::run()
{

	if (flag_run1_)
	{

		if (ros::Time::now() > time_cmd_vel_) // per evitare differenze negativi tra tempi
		{

			if ((ros::Time::now() - time_cmd_vel_) > max_dur_cmd_vel_)
			{
				des_wheels_angular_vel_ = 0.0;
				des_yaw_rate_ = 0.0;
				des_wheels_angular_vel_filt_old_ = 0.0;
				enabled_pilot = false;
			}
		}

		// senza IR
		des_wheels_angular_vel_filt_ = des_wheels_angular_vel_;
		des_wheels_angular_vel_filt_ = (1 - 0.002) * des_wheels_angular_vel_filt_old_ + (0.002 * des_wheels_angular_vel_filt_);
		des_wheels_angular_vel_filt_old_ = des_wheels_angular_vel_filt_;
		des_wheels_angular_pos_ += des_wheels_angular_vel_filt_ * dt;

		// Senza filtro
		des_yaw_ += des_yaw_rate_ * dt;

		// std::cout<< "des_yaw_"<<des_yaw_<<std::endl;
		if ((fabs(lowerbody_state_msg_.wheels_angular_pos - des_wheels_angular_pos_) > (PI)) && (!enabled_pilot))
		{
			des_wheels_angular_pos_ = lowerbody_state_msg_.wheels_angular_pos - sgn(lowerbody_state_msg_.wheels_angular_pos - des_wheels_angular_pos_) * PI;
		}
		// Des state debug
		state_debug_.des_wheels_angular_pos_ = des_wheels_angular_pos_;
		state_debug_.offset_pitch_ = offset_pitch_;
		state_debug_.des_wheels_angular_vel_ = des_wheels_angular_vel_filt_;
		state_debug_.des_pitch_rate = 0;
		state_debug_.des_yaw_ = des_yaw_;
		state_debug_.des_yaw_rate_ = des_yaw_rate_;

		// Real state debug
		state_debug_.wheels_angular_pos = lowerbody_state_msg_.wheels_angular_pos;
		state_debug_.pitch_angle = -lowerbody_state_msg_.pitch_angle;
		state_debug_.wheels_angular_vel = lowerbody_state_msg_.wheels_angular_vel;
		state_debug_.pitch_rate = lowerbody_state_msg_.pitch_rate;
		state_debug_.yaw_angle = lowerbody_state_msg_.yaw_angle;
		state_debug_.yaw_rate = lowerbody_state_msg_.yaw_rate;

		// controllo LQR
		if (!fallback_)
		{
			state << des_wheels_angular_pos_ - lowerbody_state_msg_.wheels_angular_pos,
				-offset_pitch_ - lowerbody_state_msg_.pitch_angle,
				des_wheels_angular_vel_filt_ - lowerbody_state_msg_.wheels_angular_vel,
				0.0 - lowerbody_state_msg_.pitch_rate,
				des_yaw_ - lowerbody_state_msg_.yaw_angle,
				des_yaw_rate_ - lowerbody_state_msg_.yaw_rate;

			command_feed = k_feed * state;

			//------------------------------------------------------------------------------//
			// senza loop integrale
			command(0, 0) = command_feed(0, 0);
			command(1, 0) = command_feed(1, 0);
			//------------------------------------------------------------------------------//
			com_L = (sgn(command(0, 0)) * trsh_L_ + (command(0, 0))) * PWM_EXT_RANGE_K;
			com_R = (sgn(command(1, 0)) * trsh_R_ + (command(1, 0))) * PWM_EXT_RANGE_K;
		}
		else
		{
			switch (state_fallback_)
			{
			case 0:
			{
				if ((ros::Time::now() - time_first_cmd_fall_) < ros::Duration(0.1))
				{
					com_L = 100 * PWM_EXT_RANGE_K;
					com_R = 100 * PWM_EXT_RANGE_K;
				}
				else
				{
					state_fallback_ = 1;
					time_second_cmd_fall_ = ros::Time::now();
				}

				break;
			}
			case 1:
			{
				if ((ros::Time::now() - time_second_cmd_fall_) < ros::Duration(3))
				{
					com_L = 0;
					com_R = 0;
				}
				else
					state_fallback_ = 2;

				break;
			}
			case 2:
			{
				com_L = 900 * des_wheels_angular_vel_;
				com_R = 900 * des_wheels_angular_vel_;
				break;
			}
			}
		}

		// Satura il valore di comando nel range [-2400, 2400]
		com_R = std::clamp((int)com_R, -100 * PWM_EXT_RANGE_K, 100 * PWM_EXT_RANGE_K);
		com_L = std::clamp((int)com_L, -100 * PWM_EXT_RANGE_K, 100 * PWM_EXT_RANGE_K);

		// comando ruote
		comm_pub_.p_1[0] = (com_L);
		comm_pub_.p_2[0] = (-com_R);

		pub_comm_.publish(comm_pub_);
		pub_debug_.publish(state_debug_);
	}
}

int lqr::sgn(double d)
{
	return d < 0 ? -1 : d > 0;
}

void lqr::stop_motor()
{
	com_R = 0.0;
	com_L = 0.0;

	comm_pub_.p_1.push_back(-com_R);
	comm_pub_.p_2.push_back(com_L);
	pub_comm_.publish(comm_pub_);
}

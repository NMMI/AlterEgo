#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include "boost/thread.hpp"

class twistmsg_bridge
{
public:
	twistmsg_bridge(): nh_("~")
	{


		buttonA_pub = nh_.advertise<std_msgs::Bool>("/Button_A", 1);
		input_sub_ = nh_.subscribe("/yaw_des_vel", 1, &twistmsg_bridge::inputCallback_yaw, this);
		input_sub_ = nh_.subscribe("/forward_des_vel", 1, &twistmsg_bridge::inputCallback_forward, this);
		output_des_vel=nh_.advertise<geometry_msgs::Vector3>("/segway_des_vel", 1);
		des_vel.x=0;
		des_vel.y=0;

	}
	void inputCallback_yaw(const geometry_msgs::Twist::ConstPtr &msg){
		des_vel.x=msg->linear.y;
		button_a.data=true;
		time_cmd_vel_ = ros::Time::now();
	}

	void inputCallback_forward(const geometry_msgs::Twist::ConstPtr &msg){
		des_vel.y=msg->linear.x;
		des_vel.x=msg->linear.y;
		button_a.data=true;
		time_cmd_vel_ = ros::Time::now();
	}


	void run()
	{
		ROS_INFO("run");
		boost::thread publisher_loop_t(&twistmsg_bridge::publish_loop, this);
		ros::spin();
	}

	
private:
	ros::NodeHandle nh_;
	std_msgs::Bool button_a;
	ros::Subscriber input_sub_;
	ros::Publisher buttonA_pub;
	ros::Publisher output_des_vel;
	geometry_msgs::Vector3 des_vel;
	ros::Time time_cmd_vel_;
	void publish_loop()
	{    
		std_msgs::Bool button_a;
		button_a.data=false;

		ros::Rate r(100);

		ros::Duration max_dur_cmd_vel_; 
		max_dur_cmd_vel_ = ros::Duration(1);
		time_cmd_vel_ = ros::Time::now();
		while (ros::ok())
		{
			if (ros::Time::now() > time_cmd_vel_) // per evitare differenze negativi tra tempi
			{

				if ((ros::Time::now() - time_cmd_vel_) > max_dur_cmd_vel_)
				{
					button_a.data=false;
				}
			}
			buttonA_pub.publish(button_a);
			output_des_vel.publish(des_vel);
			r.sleep();
		}
	}
};


		int main(int argc, char **argv)
		{
			ros::init(argc, argv, "twistmsg_bridge");
			twistmsg_bridge node;
			node.run();
			return 0;
		}

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/trajectory_composite.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include "boost/thread.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <message_filters/time_synchronizer.h>
#include "message_filters/subscriber.h"
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace KDL;
class arm_bridge
{
public:
	arm_bridge(): nh_("~")
	{
		buttonA_pub = nh_.advertise<std_msgs::Bool>("/Button_A", 1);
		
		transform_timeout_ = ros::Duration(5);
		tf_buffer_dur_ = ros::Duration(10);
		input_sub_ = nh_.subscribe("/desired_arm", 1, &arm_bridge::input_armCallback, this);
		left_arm=nh_.advertise<geometry_msgs::Pose>("/left/left_hand_pos", 1);
		right_arm=nh_.advertise<geometry_msgs::Pose>("/right/right_hand_pos", 1);

		transform_timeout_ = ros::Duration(tmp_val);
		tf_buffer_dur_ = ros::Duration(tmp_val_1);


	}
	void input_armCallback(const geometry_msgs::Twist::ConstPtr &msg){
		if((msg->linear.y>1.2)&&(!new_left_path)){
			new_left_path=true;
		}else if(msg->linear.y<-1.2){

			start_left_path=false;
			new_left_path=false;
		}
		if((msg->linear.x>1.2)&&(!new_right_path)){
			new_right_path=true;
		}else if(msg->linear.x<-1.2){
			start_right_path=false;
			new_right_path=false;
		}

		std::cout<<"ho finiti \n";
	}


	void run()
	{
		ROS_INFO("run");
		boost::thread publisher_L_loop_t(&arm_bridge::publish_left_loop, this);
		boost::thread publisher_R_loop_t(&arm_bridge::publish_right_loop, this);
		ros::spin();
	}

	
private:
	ros::NodeHandle nh_;
	std_msgs::Bool button_a;
	ros::Subscriber input_sub_;
	ros::Publisher right_arm;
	ros::Publisher left_arm;
	ros::Publisher buttonA_pub;
	geometry_msgs::Pose des_pose;
	ros::Time time_cmd_vel_;
	bool start_left_path=false;
	bool start_right_path=false;
	bool new_left_path=false;
	bool new_right_path=false;

    double tmp_val, tmp_val_1;

	ros::Duration transform_timeout_, tf_buffer_dur_;
	void publish_left_loop()
	{    
		
		ros::Rate r(100);
		geometry_msgs::Pose hand_shake;
		hand_shake.position.x= 0.6229150557314692;
		hand_shake.position.y= 0.16043234272620052;
		hand_shake.position.z= 0.061267118952374064;
		hand_shake.orientation.x= -0.19573566246429538;
		hand_shake.orientation.y= -0.15671990104437405;
		hand_shake.orientation.z= 0.9646160978363517;
		hand_shake.orientation.w= -0.0812358803417393;
		ros::Duration max_dur_cmd_vel_; 
		max_dur_cmd_vel_ = ros::Duration(1);
		time_cmd_vel_ = ros::Time::now();
		int i=0;
		std::unique_ptr<tf2_ros::Buffer> tf_;

		tf_ = std::make_unique<tf2_ros::Buffer>(ros::Duration(tf_buffer_dur_));
		std::string base_frame_("torso");
		std::string output_frame_("left_hand_curr");
		KDL::Frame start_pose;
		KDL::Frame end_pose(KDL::Rotation::Quaternion(hand_shake.orientation.x,hand_shake.orientation.y,hand_shake.orientation.z,hand_shake.orientation.w),KDL::Vector(hand_shake.position.x,hand_shake.position.y,hand_shake.position.z));
		
		KDL::RotationalInterpolation_SingleAxis rot_traj;
		std::vector<KDL::Frame> traj_points;

		double traj_dur=3;
		double traj_ts=0.1;
		Trajectory* traject;
		int current_point=0;


		Path_RoundedComposite* path = NULL;
		double t=0.0; 

		while (ros::ok())
		{

			// get current position
			if(new_left_path){

				if(path!=NULL) delete path;
				
				geometry_msgs::TransformStamped transformStamped;
				current_point=0;
				try
				{
					transformStamped = tf_->lookupTransform(base_frame_, output_frame_, ros::Time(0), transform_timeout_);
				}
				catch (tf2::TransformException &ex)
				{
					ROS_WARN("%s", ex.what());
					ros::Duration(1.0).sleep();
				}

			    // Convert the transform to a pose
				tf2::Stamped<tf2::Transform> stamped_transform;
				tf2::fromMsg(transformStamped, stamped_transform);
				tf2::Vector3 position = stamped_transform.getOrigin();
				tf2::Quaternion rotation = stamped_transform.getRotation();
				start_pose.p.x(position.x());
				start_pose.p.y(position.y());
				start_pose.p.z(position.z());
				start_pose.M.Quaternion(rotation.x(),rotation.y(),rotation.z(),rotation.w());

				path = new Path_RoundedComposite(0.2,0.01,new RotationalInterpolation_SingleAxis());
				path->Add(start_pose);
				path->Add(end_pose);
				path->Finish();

				 // Trajectory defines a motion of the robot along a path.
        // This defines a trapezoidal velocity profile.
				VelocityProfile* velpref = new VelocityProfile_Trap(0.5,0.1);
				velpref->SetProfile(0,path->PathLength());  
				if(traject) delete traject;
				traject = new Trajectory_Segment(path, velpref);


				new_left_path=false;
				start_left_path=true;
				t=0.0;
			}


			if(start_left_path){
				if(t <= traject->Duration()){
					Frame current_pose;
					current_pose = traject->Pos(t);
					t+= traj_ts;
					// des_pose.position.x=current_pose.p.x();
					// des_pose.position.y=current_pose.p.y();
					// des_pose.position.z=current_pose.p.z();
					current_pose.M.GetQuaternion(des_pose.orientation.x, des_pose.orientation.y, des_pose.orientation.z, des_pose.orientation.w);
					left_arm.publish(des_pose);

				}
				else{
					left_arm.publish(des_pose);

				}
			}
			r.sleep();
		}
	}


	void publish_right_loop()
	{  
		ros::Rate r(100);
		geometry_msgs::Pose hand_shake;
		hand_shake.position.x= 0.6229150557314692;
		hand_shake.position.y= 0.16043234272620052;
		hand_shake.position.z= 0.061267118952374064;
		hand_shake.orientation.x= -0.19573566246429538;
		hand_shake.orientation.y= -0.15671990104437405;
		hand_shake.orientation.z= 0.9646160978363517;
		hand_shake.orientation.w= -0.0812358803417393;
		ros::Duration max_dur_cmd_vel_; 
		max_dur_cmd_vel_ = ros::Duration(1);
		time_cmd_vel_ = ros::Time::now();
		bool new_path=false;
		int i=0;
		std::unique_ptr<tf2_ros::Buffer> tf_;

		tf_ = std::make_unique<tf2_ros::Buffer>(ros::Duration(tf_buffer_dur_));
		std::string base_frame_("torso");
		std::string output_frame_("right_hand_curr");
		KDL::Frame start_pose;
		KDL::Frame end_pose(KDL::Rotation::Quaternion(hand_shake.orientation.x,hand_shake.orientation.y,hand_shake.orientation.z,hand_shake.orientation.w),KDL::Vector(hand_shake.position.x,hand_shake.position.y,hand_shake.position.z));

		KDL::RotationalInterpolation_SingleAxis rot_traj;
		std::vector<KDL::Frame> traj_points;

		double traj_dur=3;
		double traj_ts=0.1;
		Trajectory* traject;
		int current_point=0;


		Path_RoundedComposite* path = NULL;
		double t=0.0; 

		while (ros::ok())
		{

			// get current position
			if(new_right_path){ 
				if(path!=NULL) delete path;
				geometry_msgs::TransformStamped transformStamped;
				try
				{
					transformStamped = tf_->lookupTransform(base_frame_, output_frame_, ros::Time(0), transform_timeout_);
				}
				catch (tf2::TransformException &ex)
				{
					ROS_WARN("%s", ex.what());
					ros::Duration(1.0).sleep();
				}

			    // Convert the transform to a pose
				tf2::Stamped<tf2::Transform> stamped_transform;
				tf2::fromMsg(transformStamped, stamped_transform);
				tf2::Vector3 position = stamped_transform.getOrigin();
				tf2::Quaternion rotation = stamped_transform.getRotation();

				std::cout<< "finito lookup transform \n";
				start_pose.p.x(position.x());
				start_pose.p.y(position.y());
				start_pose.p.z(position.z());
				start_pose.M.Quaternion(rotation.x(),rotation.y(),rotation.z(),rotation.w());

				path = new Path_RoundedComposite(0.2,0.01,new RotationalInterpolation_SingleAxis());
				path->Add(start_pose);
				path->Add(end_pose);
				path->Finish();
				 // Trajectory defines a motion of the robot along a path.
        // This defines a trapezoidal velocity profile.
				VelocityProfile* velpref = new VelocityProfile_Trap(0.5,0.1);
				velpref->SetProfile(0,path->PathLength());  
				if(traject) delete traject;
				traject = new Trajectory_Segment(path, velpref);


				new_right_path=false;
				t=0.0;
				start_right_path=true;
			}


			if(start_right_path){
				if(t <= traject->Duration()){
					Frame current_pose;
					current_pose = traject->Pos(t);
					t+= traj_ts;
					// des_pose.position.x=current_pose.p.x();
					// des_pose.position.y=current_pose.p.y();
					// des_pose.position.z=current_pose.p.z();
					current_pose.M.GetQuaternion(des_pose.orientation.x, des_pose.orientation.y, des_pose.orientation.z, des_pose.orientation.w);
					right_arm.publish(des_pose);

				}
				else{
					right_arm.publish(des_pose);
				}
			}
			r.sleep();
		}
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "arm_bridge");
	arm_bridge node;
	node.run();
	return 0;
}

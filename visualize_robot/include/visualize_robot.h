#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <rbdl/rbdl.h>
#include <rbdl/Constraints.h>
#include <urdf/model.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <geometry_msgs/Quaternion.h>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/JointState.h>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <tf/transform_broadcaster.h>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
// #include <ego_msgs/EgoTwist2DUnicycle.h>

#include <boost/scoped_ptr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <ego_msgs/UpperBodyState.h>
#include <ego_msgs/LowerBodyState.h>
#include <nav_msgs/Odometry.h>
#include <urdf/model.h>
#include <map>
#include <vector>
#include <string>
#include <boost/bind.hpp>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <kdl/segment.hpp>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class visualize_robot
{
public:
   visualize_robot();
   ~visualize_robot();

   void Publish();
   void odometry();
   double dt;

   // flag when the arm is ready
   int AlterEgoVersion;
   int arm_cubes_n;
   sensor_msgs::JointState joint_state_msg_;
   ego_msgs::LowerBodyState lowerbody_state_msg;
   tf::TransformBroadcaster br;
   tf::Transform transform;
   tf::Quaternion q;

private:
   // publisher:

   ros::Publisher joints_pub; // Joint_states to publish to visualize the real time robot on Rviz

   // subscriber:




   ros::Publisher odom_pub;
   void callback_lowerbody_state(const ego_msgs::LowerBodyState::ConstPtr &msg);
   void callback_upperbody_state(const ego_msgs::UpperBodyState::ConstPtr &msg);

   struct CallbackArgs {
      sensor_msgs::JointState::ConstPtr msg;
      std::string hand_part;
      std::string side;
   };

   void callback_hands_state(const CallbackArgs& args);

   // ---------------------------------------------------------------

   VectorNd Q; // vector of the 24 joint variables: x, y, yaw, pitch, front_kickstand[2], back_kickstand[2], L_wheel, R_wheel, neck[2], L_arm[6], R_arm[6]
   ros::NodeHandle nh;

   std::vector<std::string> hand_parts = {"thumb", "index", "middle", "ring", "little"};
   std::vector<std::string> sides = {"right", "left"};
   std::vector<ros::Subscriber> subscribers_;

   ros::Subscriber upperbody_state_;
   ros::Subscriber lowerbody_state_;

};

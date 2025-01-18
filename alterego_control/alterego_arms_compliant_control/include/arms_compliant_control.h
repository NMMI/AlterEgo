#include <ros/ros.h>
#include <ros/package.h>

#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <boost/scoped_ptr.hpp>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>

#include <alterego_msgs/UpperBodyState.h>
#include <alterego_msgs/LowerBodyState.h>

#include <geometry_msgs/Pose.h>

class ArmsCompliantControl 
{              
 public:         
    ArmsCompliantControl();
    ~ArmsCompliantControl();

    geometry_msgs::Pose          getArmPosition();


    geometry_msgs::Pose		init_pos;
    geometry_msgs::Pose		act_pos;
    ros::Publisher pub_posture;
    ros::Publisher pub_enable_arms_compliant_control;
    geometry_msgs::Pose		robot_pose;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    bool enable_compliant;
   
 private:
    ros::NodeHandle nh;
    void R_Botton_A__Callback(const std_msgs::Bool::ConstPtr &msg);

    ros::Subscriber sub_Botton_A_R;
    
    std::string robot_name, ns;
    int AlterEgoVersion, n_joints;

    std::string IN_topic_track, IN_topic_arms_compliant_control;
    std::string R_OUT_topic_button_A;
	  geometry_msgs::Pose		r_pilot;

    tf::TransformBroadcaster  ik_ref;
	  tf::Transform ik_tf_ref;

    std::string arm_side;


};

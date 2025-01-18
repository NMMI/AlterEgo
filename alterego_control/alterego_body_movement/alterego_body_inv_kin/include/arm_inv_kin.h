#include "ros/ros.h"
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <vector>
#include <string>
#include <pseudo_inversion.h>
#include <MovingMedianFilter.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <boost/scoped_ptr.hpp>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Float64MultiArray.h"
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Wrench.h>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <alterego_msgs/EgoArms.h>

class invKinEgo
{
public:
    invKinEgo(ros::NodeHandle *nodeH, std::string ns);
    void run(double rate);

private:
    void kinematic_loop(double run_freq);
    void update_tf(double rate);
    void publish();
    // Callback
    void posture__Callback(const geometry_msgs::Pose::ConstPtr &msg);
    void cubes_shaft__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void powerbooster__Callback(const std_msgs::Bool::ConstPtr &msg);
    void hand_closure__Callback(const std_msgs::Float64::ConstPtr &msg);
    void auto_mode__Callback(const std_msgs::Bool::ConstPtr &msg);

    ros::Time start_raise;
    ros::Time start_f_time;

    ros::Time cmd_time, cmd_time_old;

    ros::Subscriber sub_posture;
    ros::Subscriber sub_hand_cl;
    ros::Subscriber sub_cubes_shaft;
    ros::Subscriber sub_powerbooster;
    ros::Subscriber sub_auto_mode_status;

    ros::Publisher pub_cart_ref;
    ros::Publisher pub_phantom;
    ros::Publisher pub_ref_eq_des;
    ros::Publisher pub_ref_hand_eq;
    ros::Publisher ready_for_pilot;

    geometry_msgs::Pose cart_ref_msg;
    sensor_msgs::JointState phantom_msg;
    std_msgs::Float64 hand_ref_msg;
    alterego_msgs::EgoArms des_joint;

    bool VERBOSE;
    ros::NodeHandle nh_local_;
    ros::NodeHandle nh_;
    int arm_side;
    int arm_cubes_n;
    int AlterEgoVersion;

    int act_bp;
    double alpha;
    // if there is no urdf
    Eigen::MatrixXd Jac_wrench, Jac_trans_pinv_wrench;
    Eigen::VectorXd Grav_wrench;
    KDL::Vector gravity_v;
    double arm_l;
    double shoulder_l;
    double meas_shaft_addon_init;
    double cube_m;
    double cube_wrist_m;
    double cube_m_addon;
    double hand_m;
    double hand_cl;
    std::vector<double> T_t2s;
    std::vector<double> R_5to6;
    std::vector<double> T_o2t;
    std::vector<double> R_o2b;
    KDL::RigidBodyInertia inert_Q_0, inert_Q_1, inert_Q_2, inert_Q_3, inert_Q_4, inert_Q_5, inert_Q_6;

    KDL::JntArray meas_cube_shaft;
    KDL::JntArray meas_cube_shaft_grav;
    std::string side;
    bool arm_cb = false;
    bool powerbooster = false;
    bool use_addon = false;
    bool flag_pilot_out_ = true;
    bool auto_mode_enabled = false;
    // kinematic params:

    std::vector<double> a_mot;
    std::vector<double> k_mot;

    std::vector<double> q_min;
    std::vector<double> q_max;
    Eigen::MatrixXd JA;
    Eigen::MatrixXd JA_pinv;
    Eigen::VectorXd eq_dot;
    Eigen::VectorXd eq_dot_0;
    Eigen::VectorXd eq;
    Eigen::VectorXd eq_f;
    Eigen::Quaterniond ref_shaft_quat, act_shaft_quat;

    KDL::Chain chain;
    KDL::JntArray q;
    KDL::Jacobian JA_kdl;
    KDL::Jacobian J_wrench_kdl;
    KDL::Frame act_frame, frame_grav, shaft_frame, ref_frame;
    boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
    boost::scoped_ptr<KDL::ChainDynParam> dyn_solver;

    boost::scoped_ptr<KDL::ChainFkSolverPos> shaft_to_pose_solver;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> shaft_to_jac_solver;
};
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Wrench.h"

#include <alterego_msgs/EgoArms.h>
// #include "tf2_ros/transform_broadcaster.h"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/message_filter.h"
// #include "tf2/LinearMath/Matrix3x3.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "boost/thread.hpp"
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <vector>
#include <string>
#include <pseudo_inversion.h>
// #include <MovingMedianFilter.h>
#include <boost/scoped_ptr.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

using namespace KDL;
using namespace std;

#define MAX_STIFF 0.3
// #define arm_cubes_n 6
// #define use_addon true
class invDyn_GravityComp
{

public:
    invDyn_GravityComp(ros::NodeHandle *nodeH);
    void run();

private:
    bool VERBOSE;
    ros::NodeHandle nh_local_;
    ros::NodeHandle nh_;
    void joints_callback(const alterego_msgs::EgoArms::ConstPtr &msg);
    void gravity_compensation();
    void cubes_m1__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void cubes_m2__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void cubes_shaft__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void arm_stiffness__Callback(const std_msgs::Float64::ConstPtr &msg);
    void arm_stiffness__Callback_new(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void powerbooster__Callback(const std_msgs::Bool::ConstPtr &msg);
    double defl_max;
    double stiffn;
    double arm_l;
    double shoulder_l;
    int arm_cubes_n;
    bool has_control_current = false;
    int AlterEgoVersion_;
    string side;
    bool arm_cb = false;
    bool powerbooster = false;
    bool use_addon = false;

    Eigen::Quaterniond act_quat;
    Eigen::Quaterniond ref_shaft_quat, act_shaft_quat, first_cube_quat;

    // --- 	Dynamic Parameters
    std::vector<double> R_p2r;
    std::vector<double> T_h2fwk;
    std::vector<double> DH;
    std::vector<double> DH_Xtr;
    std::vector<double> DH_Xrot;
    std::vector<double> DH_Ztr;
    std::vector<double> stiffn_vec;
    std::vector<double> stiffn_vec_no_power;
    std::vector<double> stiffn_vec_power;
    std::vector<double> DH_Zrot;
    std::vector<double> T_t2s;
    std::vector<double> R_5to6;
    std::vector<double> T_o2t;
    std::vector<double> R_o2b;
    std::vector<double> q_min;
    std::vector<double> q_max;
    std::vector<int> qbmove_tf_ids;
    int softhand_tf_id;

    double meas_shaft_addon_init;
    double cube_m;
    double cube_wrist_m;
    double cube_m_addon;
    double hand_m;
    RigidBodyInertia inert_Q_0, inert_Q_1, inert_Q_2, inert_Q_3, inert_Q_4, inert_Q_5, inert_Q_6;
    KDL::Chain chain_;
    KDL::Jacobian JA_kdl;
    KDL::Jacobian J_wrench_kdl;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
    boost::scoped_ptr<KDL::ChainDynParam> dyn_solver;

    boost::scoped_ptr<KDL::ChainFkSolverPos> shaft_to_pose_solver;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> shaft_to_jac_solver;
    Vector gravity_v;
    KDL::JntArray q_des;
    KDL::JntArray qd_des;
    KDL::JntArray g_comp;
    KDL::JntArray g_wrench_comp;
    KDL::JntArray meas_cube_m1;
    KDL::JntArray meas_cube_m2;
    KDL::JntArray meas_cube_shaft;
    KDL::JntArray meas_cube_shaft_grav_;

    std::vector<double> a_mot;
    std::vector<double> k_mot;
    std::vector<double> a_mot_1;
    std::vector<double> k_mot_1;
    std::vector<double> a_mot_2;
    std::vector<double> k_mot_2;
    std::vector<double> offset_m1;
    std::vector<double> offset_m2;
    /*std::vector<double> left_a_mot_1;
    std::vector<double> left_k_mot_1;
    std::vector<double> right_a_mot_1;
    std::vector<double> right_k_mot_1;
    std::vector<double> left_a_mot_2;
    std::vector<double> left_k_mot_2;
    std::vector<double> right_a_mot_2;
    std::vector<double> right_k_mot_2;*/
    KDL::Frame des_frame, now_frame;

    ros::Duration max_cmd_time;
    ros::Duration filt_time;
    ros::Duration max_cmd_latency;
    ros::Time start_raise;
    ros::Time start_f_time;
    ros::Time initial;
    ros::Time actual;
    bool flag_pilot_out_ = true;
    double alpha;
    double theta_deflection;
    double k_rigid_shoulder = 0.0;
    double tolerance;
    double sum;
    float ISE;
    Eigen::MatrixXd Jac_wrench, Jac_trans_pinv_wrench;
    Eigen::VectorXd Grav_wrench;
    Eigen::VectorXd tau_meas;
    Eigen::Matrix<double, 6, 1> wrench;
    Eigen::VectorXd q_preset;
    Eigen::VectorXd q_send;
    Eigen::VectorXd e;
    Eigen::VectorXd q_misurata;
    Eigen::VectorXd q_des_eigen;
    Eigen::VectorXd stiffn_vec_new;
    KDL::Twist error_des_mis;
    Eigen::VectorXd error_des_mis_cart;
    Eigen::VectorXd theta_d;
    boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver;
    // Eigen::VectorXd q_des;
    geometry_msgs::Wrench wrench_msg;

    // ----- ROS PARAMETERS
    string ns;
    string chain_topic;
    string pose_ref_topic;
    string kin_des_jnt_topic;
    string stiff_ref_topic;
    string hand_cl_topic;
    string phantom_arm_topic;
    string ref_eq_arm_topic;
    string ref_pr_arm_topic;
    string ref_hand_topic;
    string cubes_m1_topic;
    string cubes_m2_topic;
    string cubes_shaft_topic;
    string power_booster_topic;
    string ISE_topic;
    string stiff_ref_topic_new;
    ros::Subscriber sub_stiffness;
    ros::Subscriber sub_des_joint;
    ros::Subscriber sub_cubes_m1;
    ros::Subscriber sub_cubes_m2;
    ros::Subscriber sub_cubes_shaft;
    ros::Subscriber sub_powerbooster;
    ros::Subscriber sub_stiffness_new;

    ros::Publisher pub_wrench;
    ros::Publisher pub_ref_eq_arm_eq;
    ros::Publisher pub_ref_pr_arm_eq;
    ros::Publisher pub_ISE;

    std_msgs::Float64MultiArray des_shaft;
    std_msgs::Float64MultiArray arm_eq_ref_msg;
    std_msgs::Float64MultiArray arm_pr_ref_msg;
    std_msgs::Float64 hand_ref_msg;
    std_msgs::Float64 weight;
    std_msgs::Float64 ISE_msg;
    unsigned long int msg_seq;
};
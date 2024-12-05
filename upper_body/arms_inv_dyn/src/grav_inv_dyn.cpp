#include <grav_inv_dyn.h>
#include "boost/thread.hpp"
#include <boost/scoped_ptr.hpp>
#include <boost/math/tools/roots.hpp>

#define M_PI 3.14159265358979323846

// We want to compute q_preset inverting the nonlinear equation tau_el = k1 * sinh(a1 * (q - stiff - offset1)) + k2 * sinh(a2 * (q + stiff - offset2))
double torque(double x, double k1, double k2, double a1, double a2, double thetad, double c, double d)
{
    return k1 * sinh(a1 * (x - thetad - c)) + k2 * sinh(a2 * (x + thetad - d));
}

pair<double, double> my_bisect(std::function<double(double)> f,
                               pair<double, double> bounds, double tol = 1e-6, uint maxiter = 1000)
{
    uint i = 0;
    double f_low = f(bounds.first);
    double f_high = f(bounds.second);
    if (f_low * f_high >= 0.0)
        return bounds;
    double amp = abs(f_high - f_low);

    do
    {
        i = i + 1;
        double midpoint = (bounds.first + bounds.second) / 2.0;
        double f_m = f(midpoint);
        // cout << bounds.first << ':' << f_low << '\t' << midpoint << ':' << f_m << '\t' << bounds.second << ':' << f_high << '\t' << i << '\t' << amp << endl;
        if (f_m == 0.0)
        {
            bounds.first = midpoint;
            bounds.second = midpoint;
            return bounds;
        }
        if (f_low * f_m >= 0)
        {
            bounds.first = midpoint;
            amp = abs(f_high - f_m);
            f_low = f_m;
        }
        else
        {
            bounds.second = midpoint;
            amp = abs(f_m - f_low);
            f_high = f_m;
        }
    } while (i < maxiter && f_high - f_low > tol);
    // cout << i << '\t' << amp << endl;
    return bounds;
}
invDyn_GravityComp::invDyn_GravityComp(ros::NodeHandle *nodeH) : nh_("~"), nh_local_(*nodeH), cube_wrist_m(0.713), cube_m_addon(0.65), hand_m(0.5), alpha(1), msg_seq(0), defl_max(0.6), meas_shaft_addon_init(0.0),
                                                                 arm_l(0.432), shoulder_l(0.2)
{
    VERBOSE = false;
    // ------------------------------------------------------------------------------------- Check/save args
    ns = ros::this_node::getNamespace();

    if (ns.find("left") != std::string::npos)
    {
        side_ = ns.substr(ns.find("left"));
    }
    else if (ns.find("right") != std::string::npos)
    {
        side_ = ns.substr(ns.find("right"));
    }

    max_cmd_time = ros::Duration(30);
    filt_time = ros::Duration(5);
    max_cmd_latency = ros::Duration(1);

    std::string robot_name = std::getenv("ROBOT_NAME");
    nh_local_.getParam("/" + robot_name + "/arm_cubes_n", arm_cubes_n);
    // std::cout << "\n number of cubes_n:\t" << arm_cubes_n;
    nh_local_.getParam("verbose", VERBOSE);
    nh_local_.getParam("arm_l", arm_l);
    // std::cout << "\n number of arm_l:\t" << arm_l;

    nh_local_.getParam("shoulder_l", shoulder_l);
    nh_local_.getParam("DH_table", DH);
    nh_local_.getParam("DH_Xtr", DH_Xtr);
    nh_local_.getParam("DH_Xrot", DH_Xrot);
    nh_local_.getParam("DH_Ztr", DH_Ztr);
    nh_local_.getParam("DH_Zrot", DH_Zrot);
    nh_local_.getParam("T_t2s", T_t2s);
    nh_local_.getParam("R_p2r", R_p2r);
    nh_local_.getParam("T_o2t", T_o2t);
    nh_local_.getParam("R_o2b", R_o2b);
    nh_local_.getParam("q_min", q_min);
    nh_local_.getParam("q_max", q_max);
    nh_local_.getParam("qbmove_tf_ids", qbmove_tf_ids);
    nh_local_.getParam("softhand_tf_id", softhand_tf_id);
    nh_local_.getParam("pose_ref_topic", pose_ref_topic);
    nh_local_.getParam("stiff_ref_topic", stiff_ref_topic);
    nh_local_.getParam("kin_des_jnt_topic", kin_des_jnt_topic);
    nh_local_.getParam("phantom_arm_topic", phantom_arm_topic);
    nh_local_.getParam("ref_eq_arm_topic", ref_eq_arm_topic);
    nh_local_.getParam("ref_pr_arm_topic", ref_pr_arm_topic);
    nh_local_.getParam("cubes_m1_topic", cubes_m1_topic);
    nh_local_.getParam("cubes_m2_topic", cubes_m2_topic);
    nh_local_.getParam("cubes_shaft_topic", cubes_shaft_topic);
    nh_local_.getParam("hand_cl_topic", hand_cl_topic);
    nh_local_.getParam("stiff_ref_topic_new", stiff_ref_topic_new);
    nh_local_.getParam("ISE_topic", ISE_topic);
    nh_local_.getParam("cube_mass", cube_m);
    nh_local_.getParam("cube_wrist_mass", cube_wrist_m);
    nh_local_.getParam("cube_addon_mass", cube_m_addon);
    nh_local_.getParam("hand_mass", hand_m);
    nh_local_.getParam("T_h2fwk", T_h2fwk);
    nh_local_.getParam("a_mot", a_mot);
    nh_local_.getParam("k_mot", k_mot);
    nh_local_.getParam("a_mot_1", a_mot_1);
    nh_local_.getParam("k_mot_1", k_mot_1);
    nh_local_.getParam("a_mot_2", a_mot_2);
    nh_local_.getParam("k_mot_2", k_mot_2);
    nh_local_.getParam("offset_m1", offset_m1);
    nh_local_.getParam("offset_m2", offset_m2);
    nh_local_.getParam("stiffness_vec", stiffn_vec);
    nh_local_.getParam("stiffness_vec", stiffn_vec_no_power);
    nh_local_.getParam("stiffness_vec_power", stiffn_vec_power);
    nh_local_.getParam("theta_deflection", theta_deflection);
    nh_local_.getParam("/" + robot_name + "/power_booster_topic", power_booster_topic);
    nh_local_.getParam("/" + robot_name + "/AlterEgoVersion", AlterEgoVersion_);

    nh_local_.getParam("/" + robot_name + "/use_addon", use_addon);
    bool has_powerboost_ = false;
    nh_local_.getParam("/" + robot_name + "/has_powerboost", has_powerboost_);

    // ------------------------------------------------------------------------------------- Kinematics

    KDL::Tree kdl_tree;
    std::string folder_path;
    nh_local_.getParam("folderPath", folder_path);
    // If folder_path exists, the chain is built from the urdf file
    if (!folder_path.empty())
    {

        const char *path;
        path = folder_path.c_str();
        std::string chain_end = side_ + "_hand_ik";
        if (!kdl_parser::treeFromFile(path, kdl_tree))
        {
            ROS_ERROR("Errore durante il parsing del file URDF");
        }
        if (!kdl_tree.getChain("torso", chain_end, chain_))
        {
            ROS_ERROR("Failed to get KDL chain");
        }
    }
    else
    {
        if (AlterEgoVersion_ == 2)
        {
            inert_Q_0 = KDL::RigidBodyInertia(0, KDL::Vector(0.0, 0.0, 0.0));
            inert_Q_1 = KDL::RigidBodyInertia(cube_m, KDL::Vector(0.0, 0.0, 0.0));
            inert_Q_2 = KDL::RigidBodyInertia(cube_m, KDL::Vector(0.0, 0.0, (0.09 - 0.007)));
            inert_Q_3 = KDL::RigidBodyInertia(cube_m, KDL::Vector(0.0, 0.0, 0.0));
            inert_Q_4 = KDL::RigidBodyInertia(cube_m, KDL::Vector(0.0, 0.0, (0.09 - 0.013)));
            inert_Q_5 = KDL::RigidBodyInertia(0.35, KDL::Vector(0.0, 0.0, 0.0));

            chain_.addSegment(Segment(Joint(Joint::None), Frame(Rotation(T_t2s[0], T_t2s[4], T_t2s[8], T_t2s[1], T_t2s[5], T_t2s[9], T_t2s[2], T_t2s[6], T_t2s[10]),
                                                                Vector(T_t2s[3], T_t2s[7], T_t2s[11]))));
            chain_.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[0], DH_Xrot[0], DH_Ztr[0], DH_Zrot[0]), inert_Q_1));
            chain_.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[1], DH_Xrot[1], DH_Ztr[1], DH_Zrot[1]), inert_Q_2));
            chain_.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[2], DH_Xrot[2], DH_Ztr[2], DH_Zrot[2]), inert_Q_3));
            chain_.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[3], DH_Xrot[3], DH_Ztr[3], DH_Zrot[3]), inert_Q_4));
            chain_.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[4], DH_Xrot[4], DH_Ztr[4], DH_Zrot[4]), inert_Q_5));
            // chain_.addSegment(Segment(Joint(Joint::None), Frame(Rotation(T_h2fwk[0], T_h2fwk[4], T_h2fwk[8], T_h2fwk[1], T_h2fwk[5], T_h2fwk[9], T_h2fwk[2], T_h2fwk[6], T_h2fwk[10]), Vector(T_h2fwk[3], T_h2fwk[7], T_h2fwk[11]))));
        }
        else if (AlterEgoVersion_ == 3)
        {
            if (VERBOSE)
                std::cout << "----------[VERBOSE]---------- Kinematics: Version " << AlterEgoVersion_ << std::endl;
            inert_Q_0 = KDL::RigidBodyInertia(0, KDL::Vector(0.0, 0.0, 0.0));
            inert_Q_1 = KDL::RigidBodyInertia(cube_m_addon, KDL::Vector(0.0, 0.0, 0.0));
            inert_Q_2 = KDL::RigidBodyInertia(cube_m, KDL::Vector(0.0, 0.0, 0.09));
            inert_Q_3 = KDL::RigidBodyInertia(cube_m + 0.1, KDL::Vector(0.0, 0.0, 0.0));
            inert_Q_4 = KDL::RigidBodyInertia(cube_m, KDL::Vector(0.0, 0.0, 0.09));
            inert_Q_5 = KDL::RigidBodyInertia(cube_wrist_m, KDL::Vector(0.0, 0.0, 0.0));
            inert_Q_6 = KDL::RigidBodyInertia(hand_m, KDL::Vector(0.0, 0.0, 0.0));

            chain_.addSegment(Segment(Joint(Joint::None), Frame(Rotation(T_t2s[0], T_t2s[4], T_t2s[8], T_t2s[1], T_t2s[5], T_t2s[9], T_t2s[2], T_t2s[6], T_t2s[10]), Vector(T_t2s[3], T_t2s[7], T_t2s[11]))));
            chain_.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[0], DH_Xrot[0], DH_Ztr[0], DH_Zrot[0]), inert_Q_1));
            chain_.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[1], DH_Xrot[1], DH_Ztr[1], DH_Zrot[1]), inert_Q_2));
            chain_.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[2], DH_Xrot[2], DH_Ztr[2], DH_Zrot[2]), inert_Q_3));
            chain_.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[3], DH_Xrot[3], DH_Ztr[3], DH_Zrot[3]), inert_Q_4));
            chain_.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[4], DH_Xrot[4], DH_Ztr[4], DH_Zrot[4]), inert_Q_5));
            chain_.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[5], DH_Xrot[5], DH_Ztr[5], DH_Zrot[5]), inert_Q_6));
            chain_.addSegment(Segment(Joint(Joint::None), Frame(Rotation(T_h2fwk[0], T_h2fwk[4], T_h2fwk[8], T_h2fwk[1], T_h2fwk[5], T_h2fwk[9], T_h2fwk[2], T_h2fwk[6], T_h2fwk[10]), Vector(T_h2fwk[3], T_h2fwk[7], T_h2fwk[11]))));
        }
    }
    g_comp.resize(chain_.getNrOfJoints());
    KDL::SetToZero(g_comp);
    g_wrench_comp.resize(chain_.getNrOfJoints());
    if (VERBOSE)
        std::cout << "\n g_wrench_size:\t" << chain_.getNrOfJoints();
    tau_meas.resize(chain_.getNrOfJoints());
    Jac_wrench.resize(6, chain_.getNrOfJoints());
    Jac_trans_pinv_wrench.resize(6, chain_.getNrOfJoints());

    gravity_v = KDL::Vector(0, 0, -9.81); // Gravity vector

    jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(chain_));
    shaft_to_pose_solver.reset(new KDL::ChainFkSolverPos_recursive(chain_));
    shaft_to_jac_solver.reset(new KDL::ChainJntToJacSolver(chain_));
    dyn_solver.reset(new KDL::ChainDynParam(chain_, gravity_v));
    jnt_to_pose_solver.reset(new KDL::ChainFkSolverPos_recursive(chain_));

    q_des.resize(chain_.getNrOfJoints()); // Joints desidered position in KDL
    KDL::SetToZero(q_des);
    JA_kdl.resize(chain_.getNrOfJoints());
    KDL::SetToZero(q_des);
    q_preset.resize(arm_cubes_n);
    q_send.resize(arm_cubes_n); // Variable generated by the control
    meas_cube_m1.resize(chain_.getNrOfJoints());
    meas_cube_m2.resize(chain_.getNrOfJoints());
    meas_cube_shaft.resize(chain_.getNrOfJoints()); // Joints measured position in KDL
    meas_cube_shaft_grav_.resize(chain_.getNrOfJoints());
    J_wrench_kdl.resize(chain_.getNrOfJoints());
    Grav_wrench.resize(chain_.getNrOfJoints());
    stiffn_vec_new.resize(chain_.getNrOfJoints());
    error_des_mis_cart.resize(3);   // End Effector error
    stiffn_vec.resize(arm_cubes_n); // Stiffness
    theta_d.resize(arm_cubes_n);    // theta_d used to create a sinusoidal stiffness and enable optimization
    theta_d << 0, 0, 0, 0, 0, 0;
    e.resize(arm_cubes_n);           // joints error, used to compute joints ISE
    q_des_eigen.resize(arm_cubes_n); // joints desidered position
    q_misurata.resize(arm_cubes_n);  // joints measured position in Eigen
    double defl_max = 0.6;
    ISE = 0.0;                          // Joints ISE
    stiffn_vec_new << 0, 0, 0, 0, 0, 0; // Variable used if you want to modify stiffness from outside (publishing on topic "stiff_ref_topic_new")

    /* -----ROS -----*/

    // ------------------------------------------------------------------------------------- Subscribe to topics
    sub_des_joint = nh_local_.subscribe(kin_des_jnt_topic, 1, &invDyn_GravityComp::joints_callback, this);
    sub_stiffness = nh_local_.subscribe(stiff_ref_topic, 1, &invDyn_GravityComp::arm_stiffness__Callback, this);
    sub_stiffness_new = nh_local_.subscribe(stiff_ref_topic_new, 1, &invDyn_GravityComp::arm_stiffness__Callback_new, this);
    // sub_cubes_m1 = nh_local_.subscribe(cubes_m1_topic, 1, &invDyn_GravityComp::cubes_m1__Callback, this);
    // sub_cubes_m2 = nh_local_.subscribe(cubes_m2_topic, 1, &invDyn_GravityComp::cubes_m2__Callback, this);
    sub_cubes_shaft = nh_local_.subscribe(cubes_shaft_topic, 1, &invDyn_GravityComp::cubes_shaft__Callback, this);
    if (has_powerboost_)
        sub_powerbooster = nh_local_.subscribe("/" + robot_name + "/" + power_booster_topic, 1, &invDyn_GravityComp::powerbooster__Callback, this);

    // ------------------------------------------------------------------------------------- Published topics
    // ros::Publisher    	pub_inv_kin		= nh_local_.advertise<qb_interface::cubeEq_Preset>(target_chain + "_eq_pre", 1000);
    // pub_cart_ref	= nh_local_.advertise<geometry_msgs::Pose>(chain_topic, 1);
    // pub_phantom			= nh_local_.advertise<sensor_msgs::JointState>(phantom_arm_topic, 1);
    pub_ref_eq_arm_eq = nh_local_.advertise<std_msgs::Float64MultiArray>(ref_eq_arm_topic, 1);
    pub_ref_pr_arm_eq = nh_local_.advertise<std_msgs::Float64MultiArray>(ref_pr_arm_topic, 1);
    pub_wrench = nh_local_.advertise<geometry_msgs::Wrench>("wrench", 1);
    pub_ISE = nh_local_.advertise<std_msgs::Float64>(ISE_topic, 1);
}

void invDyn_GravityComp::arm_stiffness__Callback(const std_msgs::Float64::ConstPtr &msg)
{
    stiffn = msg->data * MAX_STIFF;

    if (stiffn > MAX_STIFF)
        stiffn = MAX_STIFF;

    if (stiffn < 0)
        stiffn = 0;
}

void invDyn_GravityComp::arm_stiffness__Callback_new(const std_msgs::Float64MultiArray::ConstPtr &msg)
{

    for (int i = 0; i < msg->data.size(); i++)
    {
        stiffn_vec_new(i) = msg->data[i];
    }

    for (int i = 0; i < msg->data.size(); i++)
    {
        if (stiffn_vec_new(i) > 0.6)
            stiffn_vec_new(i) = 0.6;

        if (stiffn_vec_new(i) < 0)
            stiffn_vec_new(i) = 0;
    }
}
/*---------------------------------------------------------------------*
 * PowerBooster CALLBACK                                                *
 *                                                                      *
 *----------------------------------------------------------------------*/
void invDyn_GravityComp::powerbooster__Callback(const std_msgs::Bool::ConstPtr &msg)
{

    powerbooster = msg->data;
}
void invDyn_GravityComp::cubes_m1__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    for (int i = 0; i < msg->data.size(); i++)
    {
        meas_cube_m1(i) = msg->data[i];
    }

    if (AlterEgoVersion_ == 3)
    {
        meas_cube_m1(0) = -meas_cube_m1(0); // to control for the left one
        // meas_cube_m1(1) = meas_cube_m1(1) / 2;
    }
}

void invDyn_GravityComp::cubes_m2__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    for (int i = 0; i < msg->data.size(); i++)
    {
        meas_cube_m2(i) = msg->data[i];
    }

    if (AlterEgoVersion_ == 3)
    {

        meas_cube_m2(0) = -meas_cube_m2(0);
        // meas_cube_m2(1) = meas_cube_m2(1) / 2;
    }
}

void invDyn_GravityComp::cubes_shaft__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    for (int i = 0; i < msg->data.size(); i++)
    {
        meas_cube_shaft(i) = msg->data[i];
    }
    if (AlterEgoVersion_ == 3)
    {

        // meas_cube_shaft(1) = (meas_cube_shaft(1)) / 2.086;

        if (!arm_cb)
            meas_shaft_addon_init = meas_cube_shaft(1);

        meas_cube_shaft(1) = (meas_cube_shaft(1) - meas_shaft_addon_init); // 19° -> 0.33rad flange offset w.r.t. cube zero position
    }
    arm_cb = true;
}

void invDyn_GravityComp::joints_callback(const ego_msgs::EgoArms::ConstPtr &msg)
{
    for (int i = 0; i < msg->q_des.size(); i++)
    {
        q_des(i) = msg->q_des[i];
    }
}

void invDyn_GravityComp::gravity_compensation()
{
    int run_freq = 100;
    std::string robot_name = std::getenv("ROBOT_NAME");
    nh_local_.getParam("/" + robot_name + "/arm_cubes_n", arm_cubes_n);
    nh_local_.getParam("/" + robot_name + "/arms_frequency", run_freq); // Override if configured
    pair<double, double> P(-10.0, 10.0);

    //Massa e lunghezza del braccio
    double m = 4;
    double L = 0.8;
    double g = 9.81;
    // Calcolare il momento dovuto alla gravità
    double M_g = m * g * L/2;
    // Calcolare la costante elastica dopo una prova sperimentale con una deflessione di 0.34 rad
    k_rigid_shoulder = M_g/theta_deflection;

    ros::Rate loop_rate(run_freq);
    initial = ros::Time::now();
    while (ros::ok())
    {

        // Gravity
        dyn_solver->JntToGravity(q_des, g_comp);

        if (powerbooster)
        {
            stiffn_vec = stiffn_vec_power;
        }
        else
        {
            stiffn_vec = stiffn_vec_no_power;
        }

        // We create a sinusoidal stiffness to do optimization
        actual = ros::Time::now();
        double tempo_attuale = actual.toSec();
        for (int i = 0; i < 6; i++)
        {
            theta_d(i) = 0.5 * sin(0.1 * M_PI * tempo_attuale);
        }

        // Calcolo indice di prestazione
        for (int i = 0; i < meas_cube_shaft.rows(); ++i)
        {
            q_misurata(i) = meas_cube_shaft(i); // Transformation from JntArray to Eigen
        }

        for (int i = 0; i < q_des.rows(); ++i)
        {
            q_des_eigen(i) = q_des(i); // Transformation from JntArray to Eigen
        }

        // Joints ISE to evaluete controller performance
        e = q_des_eigen - q_misurata;
        sum = 0.0;
        for (int i = 0; i < 6; ++i)
        {
            sum += e(i) * e(i);
        }
        ISE += sum / run_freq;

        // End Effector error
        jnt_to_pose_solver->JntToCart(q_des, des_frame);
        jnt_to_pose_solver->JntToCart(meas_cube_shaft, now_frame);
        error_des_mis = KDL::diff(des_frame, now_frame);
        error_des_mis_cart << error_des_mis[0], error_des_mis[1], error_des_mis[2];

        // q_preset with a and k different for each motor
        for (int i = 0; i < arm_cubes_n; i++)
        {
            double k1 = k_mot_1[i];
            double k2 = k_mot_2[i];
            double a1 = a_mot_1[i];
            double a2 = a_mot_2[i];
            double tau = g_comp(i);
            double stiff = stiffn_vec[i];
            double c = offset_m1[i];
            double d = offset_m2[i];
            auto result = my_bisect([k1, k2, a1, a2, stiff, tau, c, d](double
                                                                           x)
                                    { return torque(x, k1, k2, a1, a2, stiff, c, d) - tau; },
                                    P, 1e-12);
            q_preset(i) = (result.first + result.second) / 2.0;
            q_send(i) = q_des(i) + q_preset(i);
        }

        // q_preset with the same a and k for each motor
        /*for (int i = 0; i < arm_cubes_n; i++)
        {

            if (AlterEgoVersion_ == 2)
            {
                if (VERBOSE)
                    std::cout << ns << "Version" << AlterEgoVersion_ << " Q_send deflection:\n " << q_send << std::endl;

                q_preset(i) = 1 / a_mot[i] * (asinh(g_comp(i) / (2 * k_mot[i] * cosh(a_mot[i] * stiffn_vec[i]))));

                if (isnan(q_preset(i)))
                {
                    q_preset(i) = 0;
                }

                q_send(i) = q_des(i) + q_preset(i);
            }
            else if (AlterEgoVersion_ == 3)
            {

                if (i == 0)
                {

                    // Calcolare il preset come funzione del momento di gravità
                    q_preset(i) = g_comp(i) / k_rigid_shoulder;
                }
                else
                    q_preset(i) = 1 / a_mot[i] * (asinh(g_comp(i) / (2 * k_mot[i] * cosh(a_mot[i] * stiffn_vec[i]))));

                if (isnan(q_preset(i)))
                {
                    q_preset(i) = 0;
                }

                q_send(i) = q_des(i) + q_preset(i); // q_send(i) = 2.086*(q_des(i) +q_preset(i)-(0.95));     //54.5° -> 0.95rad flange offset w.r.t. cube zero position
            }
        }*/

        if (VERBOSE)
            std::cout << ns << "Version" << AlterEgoVersion_ << " Q_preset deflection:\n " << q_preset << std::endl;
        if (VERBOSE)
            std::cout << ns << "Version" << AlterEgoVersion_ << " Q_send: \n"
                      << q_send << std::endl;

        // Saturation
        for (int i = 0; i < arm_cubes_n; i++)
        {
            if (q_send(i) > (q_max[i] + q_preset(i)))
                q_send(i) = q_max[i] + q_preset(i);
            if (q_send(i) < (q_min[i] + q_preset(i)))
                q_send(i) = (q_min[i] + q_preset(i));
        }

        // --- set all messages ---
        arm_eq_ref_msg.data.clear();
        arm_pr_ref_msg.data.clear();
        if (ros::Time::now() - initial < ros::Duration(30))
        {
            ISE_msg.data = ISE;
        }
        for (int i = 0; i < chain_.getNrOfJoints(); i++)
        {
            arm_eq_ref_msg.data.push_back(q_send(i));
            arm_pr_ref_msg.data.push_back(stiffn_vec[i]);
        }
        pub_ref_eq_arm_eq.publish(arm_eq_ref_msg);
        pub_ref_pr_arm_eq.publish(arm_pr_ref_msg);
        pub_ISE.publish(ISE_msg);
        loop_rate.sleep();
    }
}

void invDyn_GravityComp::run()
{
    ROS_INFO("Started Gravity Comp ");
    boost::thread publisher_loop_t(&invDyn_GravityComp::gravity_compensation, this);
    ros::spin();
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "arm_inv_dyn");
    ros::NodeHandle handler;
    invDyn_GravityComp node(&handler);
    node.run();
    return 0;
}
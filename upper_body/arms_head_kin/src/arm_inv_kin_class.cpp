#include <arm_inv_kin.h>

/*---------------------------------------------------------------------*
 * POSTURE CALLBACK                                                     *
 *                                                                      *
 *----------------------------------------------------------------------*/
void invKinEgo::posture__Callback(const geometry_msgs::Pose::ConstPtr &msg)
{
    Eigen::Quaterniond ref_quat;
    static Eigen::Quaterniond old_quat;
    double sign_check;
    Eigen::Vector3d r_p;
    Eigen::Matrix3d r_M;
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    r_p << msg->position.x, msg->position.y, msg->position.z;

    ref_quat.x() = msg->orientation.x;
    ref_quat.y() = msg->orientation.y;
    ref_quat.z() = msg->orientation.z;
    ref_quat.w() = msg->orientation.w;

    sign_check = ref_quat.w() * old_quat.w() + ref_quat.x() * old_quat.x() + ref_quat.y() * old_quat.y() + ref_quat.z() * old_quat.z();
    if (sign_check < 0.0)
    {
        ref_quat.w() = -ref_quat.w();
        ref_quat.vec() = -ref_quat.vec();
    }
    old_quat = ref_quat;
    r_M = ref_quat;

    ref_frame = KDL::Frame(KDL::Rotation(r_M(0, 0), r_M(0, 1), r_M(0, 2), r_M(1, 0), r_M(1, 1), r_M(1, 2), r_M(2, 0), r_M(2, 1), r_M(2, 2)), KDL::Vector(r_p(0), r_p(1), r_p(2)));
    cmd_time_old = cmd_time;
    cmd_time = ros::Time::now();
}

void invKinEgo::cubes_shaft__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    for (int i = 0; i < msg->data.size(); i++)
    {
        meas_cube_shaft(i) = msg->data[i];
    }
    if (AlterEgoVersion == 3)
    {


        if (!arm_cb)
            meas_shaft_addon_init = meas_cube_shaft(1);

        meas_cube_shaft(1) = (meas_cube_shaft(1) - meas_shaft_addon_init); // 19° -> 0.33rad flange offset w.r.t. cube zero position
    }
    arm_cb = true;
}

/*---------------------------------------------------------------------*
 * PowerBooster CALLBACK                                                *
 *                                                                      *
 *----------------------------------------------------------------------*/
void invKinEgo::powerbooster__Callback(const std_msgs::Bool::ConstPtr &msg)
{

    powerbooster = msg->data;
}

/*---------------------------------------------------------------------*
 * HAND CLOSURE CALLBACK                                                *
 *                                                                      *
 *----------------------------------------------------------------------*/
void invKinEgo::hand_closure__Callback(const std_msgs::Float64::ConstPtr &msg)
{

    hand_ref_msg.data = msg->data;
}

int sgn(double d)
{
    return d < 0 ? -1 : d > 0;
}


/*---------------------------------------------------------------------*
 * AUTO MODE CALLBACK                                                *
 *                                                                      *
 *----------------------------------------------------------------------*/
void invKinEgo::auto_mode__Callback(const std_msgs::Bool::ConstPtr &msg)
{
    auto_mode_enabled = msg->data;
    if (auto_mode_enabled)
    {
        ROS_INFO("Auto mode enabled");
    }
    else
    {
        ROS_INFO("Auto mode disabled");
    }
}


invKinEgo::invKinEgo(ros::NodeHandle *nodeH, std::string ns) : nh_("~"), nh_local_(*nodeH)
{
    if (VERBOSE)
        std::cout << "starting \n \n";  
    if (ns.find("left") != std::string::npos)
    {
        side = ns.substr(ns.find("left"));
    }
    else if (ns.find("right") != std::string::npos)
    {
        side = ns.substr(ns.find("right"));
    }

    std::string robot_name = std::getenv("ROBOT_NAME");

    nh_local_.getParam("/" + robot_name + "/arm_cubes_n", arm_cubes_n);
    std::string chain_topic;
    std::string pose_ref_topic;
    std::string hand_cl_topic;
    std::string phantom_arm_topic;
    std::string ref_eq_arm_topic;
    std::string ref_pr_arm_topic;
    std::string ref_hand_topic;
    std::string cubes_shaft_topic;
    std::string power_booster_topic;
    std::string kin_des_jnt_topic;
    
    act_bp = 1;
    nh_local_.getParam("active_back_pos", act_bp);

    alpha = 1;
    nh_local_.getParam("active_back_pos", alpha);
    nh_local_.getParam("pose_ref_topic", pose_ref_topic);
    nh_local_.getParam("phantom_arm_topic", phantom_arm_topic);
    nh_local_.getParam("ref_eq_arm_topic", ref_eq_arm_topic);
    nh_local_.getParam("ref_pr_arm_topic", ref_pr_arm_topic);
    nh_local_.getParam("ref_hand_topic", ref_hand_topic);
    nh_local_.getParam("cubes_shaft_topic", cubes_shaft_topic);
    nh_local_.getParam("hand_cl_topic", hand_cl_topic);
    nh_local_.getParam("cube_mass", cube_m);
    nh_local_.getParam("cube_wrist_mass", cube_wrist_m);
    nh_local_.getParam("cube_addon_mass", cube_m_addon);
    nh_local_.getParam("kin_des_jnt_topic", kin_des_jnt_topic);

    nh_local_.getParam("q_min", q_min);
    nh_local_.getParam("q_max", q_max);
    AlterEgoVersion = 3;
    nh_local_.getParam("/" + robot_name + "/AlterEgoVersion", AlterEgoVersion);
    KDL::Tree kdl_tree;
    std::string folder_path;
    const char *path;

    sub_posture = nh_local_.subscribe(pose_ref_topic, 1, &invKinEgo::posture__Callback, this);
    sub_hand_cl = nh_local_.subscribe(hand_cl_topic, 1, &invKinEgo::hand_closure__Callback, this);
    sub_cubes_shaft = nh_local_.subscribe(cubes_shaft_topic, 1, &invKinEgo::cubes_shaft__Callback, this);
    sub_auto_mode_status = nh_local_.subscribe("/" + robot_name + "/auto_mode_status", 1, &invKinEgo::auto_mode__Callback, this);
    pub_ref_eq_des = nh_local_.advertise<ego_msgs::EgoArms>(kin_des_jnt_topic, 1);
    pub_ref_hand_eq = nh_local_.advertise<std_msgs::Float64>(ref_hand_topic, 1);

    nh_local_.getParam("folderPath", folder_path);
    if (!folder_path.empty())
    {
        ROS_WARN("LOADING URDF on side %s",side.c_str());

        path = folder_path.c_str();
        std::string chain_end = side + "_hand_ik";
        if (!kdl_parser::treeFromFile(path, kdl_tree))
        {
            ROS_ERROR("Errore durante il parsing del file URDF");
        }
        if (!kdl_tree.getChain("torso", chain_end, chain))
        {
            ROS_ERROR("Failed to get KDL chain");
        }
    }
    else
    {
        ROS_WARN("LOADING DH on side %s",side.c_str());
        using namespace KDL;
        std::vector<double> R_p2r;
        std::vector<double> T_h2fwk;
        std::vector<double> DH;
        std::vector<double> DH_Xtr;
        std::vector<double> DH_Xrot;
        std::vector<double> DH_Ztr;
        std::vector<double> DH_Zrot;
        nh_local_.getParam("arm_l", arm_l);
        nh_local_.getParam("shoulder_l", shoulder_l);
        nh_local_.getParam("DH_table", DH);
        nh_local_.getParam("DH_Xtr", DH_Xtr);
        nh_local_.getParam("DH_Xrot", DH_Xrot);
        nh_local_.getParam("DH_Ztr", DH_Ztr);
        nh_local_.getParam("DH_Zrot", DH_Zrot);
        nh_local_.getParam("T_t2s", this->T_t2s);
        nh_local_.getParam("R_p2r", R_p2r);       // forse non serve
        nh_local_.getParam("T_o2t", this->T_o2t); // forse non serve
        nh_local_.getParam("R_o2b", this->R_o2b); // forse non serve
        nh_local_.getParam("T_h2fwk", T_h2fwk);
        // ------------------------------------------------------------------------------------- Kinematics
        if (AlterEgoVersion == 2)
        {
            inert_Q_0 = KDL::RigidBodyInertia(0, KDL::Vector(0.0, 0.0, 0.0));
            inert_Q_1 = KDL::RigidBodyInertia(cube_m, KDL::Vector(0.0, 0.0, 0.0));
            inert_Q_2 = KDL::RigidBodyInertia(cube_m, KDL::Vector(0.0, 0.0, (0.09 - 0.007)));
            inert_Q_3 = KDL::RigidBodyInertia(cube_m, KDL::Vector(0.0, 0.0, 0.0));
            inert_Q_4 = KDL::RigidBodyInertia(cube_m, KDL::Vector(0.0, 0.0, (0.09 - 0.013)));
            inert_Q_5 = KDL::RigidBodyInertia(0.35, KDL::Vector(0.0, 0.0, 0.0));

            chain.addSegment(Segment(Joint(Joint::None), Frame(Rotation(T_t2s[0], T_t2s[4], T_t2s[8], T_t2s[1], T_t2s[5], T_t2s[9], T_t2s[2], T_t2s[6], T_t2s[10]),
                                                               Vector(T_t2s[3], T_t2s[7], T_t2s[11]))));
            chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[0], DH_Xrot[0], DH_Ztr[0], DH_Zrot[0]), inert_Q_1));
            chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[1], DH_Xrot[1], DH_Ztr[1], DH_Zrot[1]), inert_Q_2));
            chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[2], DH_Xrot[2], DH_Ztr[2], DH_Zrot[2]), inert_Q_3));
            chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[3], DH_Xrot[3], DH_Ztr[3], DH_Zrot[3]), inert_Q_4));
            chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[4], DH_Xrot[4], DH_Ztr[4], DH_Zrot[4]), inert_Q_5));
            // chain.addSegment(Segment(Joint(Joint::None),Frame(Rotation(T_h2fwk[0],T_h2fwk[4],T_h2fwk[8],T_h2fwk[1],T_h2fwk[5],T_h2fwk[9],T_h2fwk[2],T_h2fwk[6],T_h2fwk[10]),
            // Vector(T_h2fwk[3],T_h2fwk[7],T_h2fwk[11]))));
        }
        else if (AlterEgoVersion == 3)
        {
            if (VERBOSE)
                std::cout << "----------[VERBOSE]---------- Kinematics: Version " << AlterEgoVersion << std::endl;
            inert_Q_0 = KDL::RigidBodyInertia(0, KDL::Vector(0.0, 0.0, 0.0));
            inert_Q_1 = KDL::RigidBodyInertia(cube_m_addon, KDL::Vector(0.0, 0.0, 0.0));
            inert_Q_2 = KDL::RigidBodyInertia(cube_m, KDL::Vector(0.0, 0.0, 0.09));
            inert_Q_3 = KDL::RigidBodyInertia(cube_m + 0.1, KDL::Vector(0.0, 0.0, 0.0));
            inert_Q_4 = KDL::RigidBodyInertia(cube_m, KDL::Vector(0.0, 0.0, 0.09));
            inert_Q_5 = KDL::RigidBodyInertia(cube_wrist_m, KDL::Vector(0.0, 0.0, 0.0));
            inert_Q_6 = KDL::RigidBodyInertia(hand_m, KDL::Vector(0.0, 0.0, 0.0));

            chain.addSegment(Segment(Joint(Joint::None), Frame(Rotation(T_t2s[0], T_t2s[4], T_t2s[8], T_t2s[1], T_t2s[5], T_t2s[9], T_t2s[2], T_t2s[6], T_t2s[10]),
                                                               Vector(T_t2s[3], T_t2s[7], T_t2s[11]))));
            chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[0], DH_Xrot[0], DH_Ztr[0], DH_Zrot[0]), inert_Q_1));
            chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[1], DH_Xrot[1], DH_Ztr[1], DH_Zrot[1]), inert_Q_2));
            chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[2], DH_Xrot[2], DH_Ztr[2], DH_Zrot[2]), inert_Q_3));
            chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[3], DH_Xrot[3], DH_Ztr[3], DH_Zrot[3]), inert_Q_4));
            chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[4], DH_Xrot[4], DH_Ztr[4], DH_Zrot[4]), inert_Q_5));
            chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[5], DH_Xrot[5], DH_Ztr[5], DH_Zrot[5]), inert_Q_6));
            // chain.addSegment(Segment(Joint(Joint::None), Frame(Rotation(T_h2fwk[0], T_h2fwk[4], T_h2fwk[8], T_h2fwk[1], T_h2fwk[5], T_h2fwk[9], T_h2fwk[2], T_h2fwk[6], T_h2fwk[10]),
            //                                                    Vector(T_h2fwk[3], T_h2fwk[7], T_h2fwk[11]))));
        }
    }

    cart_ref_msg.position.x = 0;
    cart_ref_msg.position.y = 0;
    cart_ref_msg.position.z = 0;
    hand_ref_msg.data = 0;

    des_joint.q_des.assign(chain.getNrOfJoints(), 0);
    des_joint.qd_des.assign(chain.getNrOfJoints(), 0);
    des_joint.qdd_des.assign(chain.getNrOfJoints(), 0);

    gravity_v = KDL::Vector(0, 0, -9.81);
    jnt_to_pose_solver.reset(new KDL::ChainFkSolverPos_recursive(chain));
    jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(chain));
    shaft_to_pose_solver.reset(new KDL::ChainFkSolverPos_recursive(chain));
    shaft_to_jac_solver.reset(new KDL::ChainJntToJacSolver(chain));
    dyn_solver.reset(new KDL::ChainDynParam(chain, gravity_v));

    meas_cube_shaft.resize(chain.getNrOfJoints());
    KDL::SetToZero(meas_cube_shaft);
    meas_cube_shaft_grav.resize(chain.getNrOfJoints());
    KDL::SetToZero(meas_cube_shaft_grav);
    J_wrench_kdl.resize(chain.getNrOfJoints());
    Grav_wrench.resize(chain.getNrOfJoints());
    eq.resize(chain.getNrOfJoints());
    eq = Eigen::VectorXd::Zero(chain.getNrOfJoints());
    eq_f.resize(chain.getNrOfJoints());
    eq_f = Eigen::VectorXd::Zero(chain.getNrOfJoints());
    eq_dot.resize(chain.getNrOfJoints());
    eq_dot = Eigen::VectorXd::Zero(chain.getNrOfJoints());
    eq_dot_0.resize(chain.getNrOfJoints());
    eq_dot_0 = Eigen::VectorXd::Zero(chain.getNrOfJoints());
    JA.resize(6, arm_cubes_n);
    JA_pinv.resize(arm_cubes_n, 6);

    hand_cl = 0;

    q.resize(chain.getNrOfJoints());
    JA_kdl.resize(chain.getNrOfJoints());
    KDL::SetToZero(q);

    shaft_to_pose_solver->JntToCart(meas_cube_shaft, shaft_frame);
    jnt_to_pose_solver->JntToCart(q, ref_frame);
    jnt_to_jac_solver->JntToJac(q, JA_kdl);
}

void invKinEgo::kinematic_loop(double run_freq)
{
    ros::Rate loop(run_freq);
    ros::Duration max_cmd_time = ros::Duration(600);
    ros::Duration filt_time = ros::Duration(5);
    ros::Duration max_cmd_latency = ros::Duration(1);
    bool flag_pilot_out_ = true;
    Eigen::Matrix<double, 6, 6> K_v;
    Eigen::Matrix<double, 6, 6> K_d;
    Eigen::MatrixXd K_0(arm_cubes_n, arm_cubes_n);

    K_d = 0.1 * Eigen::MatrixXd::Identity(6, 6);
    K_0 = Eigen::MatrixXd::Zero(arm_cubes_n, arm_cubes_n);
    Eigen::MatrixXd W(6, 6);
    W = Eigen::MatrixXd::Identity(6, 6);
    KDL::Twist err_twist;
    Eigen::Matrix<double, 6, 1> err_post;
    Eigen::VectorXd x_post(6);
    if (AlterEgoVersion == 2)
    {

        K_v = 1500 / run_freq * Eigen::MatrixXd::Identity(6, 6);
        K_0(2, 2) = 0.1;
        K_0(4, 4) = 0.1;
    }
    else if (AlterEgoVersion == 3)
    {
        K_v = 12 * Eigen::MatrixXd::Identity(6, 6);
        K_0(2, 2) = 0.3;
        K_0(4, 4) = 0.3;
        if (VERBOSE)
            std::cout << "\nK_0:\t" << K_0;

    }
    // --- Inverse Kinematics ---
    
    if (VERBOSE)
    {   
        std::cout << "\nstart_the loop";
        std::cout << "\neq_dot_0:\t" << eq_dot_0;
        std::cout << "\neq_dot:\t" << eq_dot;
        std::cout << "\neq:\t" << eq;
        std::cout << "\nq_max:\t";
    }
    for (double el : q_max)
    {
        
        if (VERBOSE) std::cout << el << " ";

    }
    if (VERBOSE) std::cout << "\nq_min:\t";
    for (double el : q_min)
    {
        if (VERBOSE) std::cout << el << " ";
    }

    while (ros::ok())
    {

        jnt_to_pose_solver->JntToCart(q, act_frame);
        jnt_to_jac_solver->JntToJac(q, JA_kdl);
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < arm_cubes_n; j++)
                if (fabs(JA_kdl(i, j)) > 0.000001)
                    JA(i, j) = JA_kdl(i, j);
                else
                    JA(i, j) = 0;
        }

        err_twist = KDL::diff(act_frame, ref_frame);

        err_post << err_twist[0], err_twist[1], err_twist[2], err_twist[3], err_twist[4], err_twist[5];
        err_post = K_v * err_post;

        if (AlterEgoVersion == 2)
        {
            JA_pinv = JA.transpose() * ((JA * JA.transpose() + K_d).inverse());
        }
        else if (AlterEgoVersion == 3)
        {
            JA_pinv = W.inverse() * JA.transpose() * ((JA * W.inverse() * JA.transpose() + K_d).inverse());
        }

        // --- Redundancy contribution ---
        for (int i = 0; i < chain.getNrOfJoints(); i++)
        {
            eq_dot_0(i) = -K_0(i, i) * (q(i) - (q_max[i] - q_min[i]));
        }

        eq_dot = JA_pinv * err_post + (Eigen::MatrixXd::Identity(chain.getNrOfJoints(), chain.getNrOfJoints()) - JA_pinv * JA) * eq_dot_0;
        eq += eq_dot / run_freq;
        // Back position
        if (act_bp == 1 && (ros::Time::now() - cmd_time > max_cmd_time))
        {
            eq = Eigen::VectorXd::Zero(arm_cubes_n);
            start_f_time = ros::Time::now();
            alpha = 1;
            flag_pilot_out_ = true;
        }
        else
            flag_pilot_out_ = false;

        // Filtering position
        if (ros::Time::now() - start_f_time < filt_time)
        {
            alpha -= 1 / (filt_time.toSec() * run_freq);
            if (alpha < 0)
                alpha = 0;
            eq_f = alpha * eq_f + (1 - alpha) * eq;
        }
        else
        {
            eq_f = eq;
        }

        // controllo che ognuno di questi gdl sia in un range di 0.3 dallo 0. per (1) considero che ha un offset di +- 0.33
        // if (flag_pilot_out_ && (std::fabs(meas_cube_shaft(0)) < 0.1) && ((std::fabs(meas_cube_shaft(1)) > 0.25) && (std::fabs(meas_cube_shaft(1)) < 0.35)) && (std::fabs(meas_cube_shaft(3)) < 0.1))
        // {
        //     std_msgs::Bool msg;
        //     msg.data = true;
        //     ready_for_pilot.publish(msg);
        // }

        des_joint.q_des.clear();
        for (int i = 0; i < arm_cubes_n; i++)
        {
            if (eq_f(i) > q_max[i])
            {
                eq_f(i) = q_max[i];
            }
            if (eq_f(i) < q_min[i])
            {
                eq_f(i) = q_min[i];
            }

            eq(i) = eq_f(i);
            q(i) = eq_f(i);
            des_joint.q_des.push_back(q(i));
        }
        // Print auto_mode status

        
        if(!auto_mode_enabled) publish();
        loop.sleep();
    }
}
void invKinEgo::update_tf(double rate)
{ // --- Rviz ---
    ros::Rate r(rate);
    tf::TransformBroadcaster ik_br, ik_ref, ik_shaft;
    tf::Transform ik_tf, ik_tf_ref, ik_tf_shaft;
    while (ros::ok())
    { // Rviz TF:
        // act_frame.M.GetQuaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w());
        // ik_tf.setRotation(tf::Quaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w()));
        // ik_tf.setOrigin(tf::Vector3(act_frame.p[0], act_frame.p[1], act_frame.p[2]));
        // ik_br.sendTransform(tf::StampedTransform(ik_tf, ros::Time::now(), "torso", side + "_ego_ik"));


		ref_frame.M.GetQuaternion(ref_shaft_quat.x(), ref_shaft_quat.y(), ref_shaft_quat.z(), ref_shaft_quat.w());
		ik_tf_ref.setRotation(tf::Quaternion(ref_shaft_quat.x(), ref_shaft_quat.y(), ref_shaft_quat.z(), ref_shaft_quat.w()));
		ik_tf_ref.setOrigin(tf::Vector3(ref_frame.p[0], ref_frame.p[1], ref_frame.p[2]));
		ik_ref.sendTransform(tf::StampedTransform(ik_tf_ref, ros::Time::now(), "torso", side + "_hand_ref"));

		shaft_to_pose_solver->JntToCart(meas_cube_shaft, shaft_frame);
		shaft_frame.M.GetQuaternion(act_shaft_quat.x(), act_shaft_quat.y(), act_shaft_quat.z(), act_shaft_quat.w());
		ik_tf_shaft.setOrigin(tf::Vector3(shaft_frame.p[0], shaft_frame.p[1], shaft_frame.p[2]));
		ik_tf_shaft.setRotation(tf::Quaternion(act_shaft_quat.x(), act_shaft_quat.y(), act_shaft_quat.z(), act_shaft_quat.w()));
		ik_shaft.sendTransform(tf::StampedTransform(ik_tf_shaft, ros::Time::now(), "torso", side + "_hand_curr"));
        
        r.sleep();
    }
}
/// @brief here one can add all the publish commands
void invKinEgo::publish()
{
    pub_ref_eq_des.publish(des_joint);
    pub_ref_hand_eq.publish(hand_ref_msg);
}

void invKinEgo::run(double rate)
{
    ROS_INFO("Started Inv kin %s", side.c_str());
    boost::thread kinematic_t(&invKinEgo::kinematic_loop, this, rate);
    boost::thread tf_t(&invKinEgo::update_tf, this, rate);
    ros::spin();
}
#include <visualize_robot.h>
// ----------------------------------------------------------------------------------------------------------
// Callback Functions to acquire the element of vector Q, to update it.
// ----------------------------------------------------------------------------------------------------------

void visualize_robot::callback_upperbody_state(const ego_msgs::UpperBodyState::ConstPtr &msg)
{

  for (int i = 0; i < joint_state_msg_.name.size(); i++)
  {

    if (joint_state_msg_.name[i] == "base_link_to_neck")
      joint_state_msg_.position[i] = (double)msg->left_meas_neck_shaft;

    if (joint_state_msg_.name[i] == "neck_to_neck_cube")
      joint_state_msg_.position[i] = (double)msg->right_meas_neck_shaft;

    if (joint_state_msg_.name[i] == "base_link_to_left_shoulder_flange")
    {
      for (int j = 0; j < msg->left_meas_arm_shaft.size(); j++)
      {
        joint_state_msg_.position[i + j] = (double)msg->left_meas_arm_shaft[j];
      }
    }

    if (joint_state_msg_.name[i] == "base_link_to_right_shoulder_flange")
    {

      for (int j = 0; j < msg->right_meas_arm_shaft.size(); j++)
      {
        joint_state_msg_.position[i + j] = (double)msg->right_meas_arm_shaft[j];
      }
    }

    
  }

}


void visualize_robot::callback_lowerbody_state(const ego_msgs::LowerBodyState::ConstPtr &msg)
{

  for (int i = 0; i < joint_state_msg_.name.size(); i++)
  {


    if (joint_state_msg_.name[i] == "base_link_to_wheel_L")
      joint_state_msg_.position[i] = (double)msg->left_wheel_enc; // shouldn it be posL ??

    if (joint_state_msg_.name[i] == "base_link_to_wheel_R")
      joint_state_msg_.position[i] = (double)msg->right_wheel_enc;
  }

  lowerbody_state_msg.mobile_base_pos_x = (double)msg->mobile_base_pos_x;
  lowerbody_state_msg.mobile_base_pos_y = (double)msg->mobile_base_pos_y;
  lowerbody_state_msg.yaw_angle = (double)msg->yaw_angle;
  lowerbody_state_msg.pitch_angle = (double)msg->pitch_angle;
}

void visualize_robot::callback_hands_state(const CallbackArgs& args)
{
  std::map<std::string, std::vector<std::string>> joint_names = {
    {"thumb", {args.side + "_hand_v3_thumb_knuckle_joint", args.side + "_hand_v3_thumb_proximal_joint", args.side + "_hand_v3_thumb_distal_joint", args.side + "_hand_v3_thumb_proximal_virtual_joint", args.side + "_hand_v3_thumb_distal_virtual_joint"}},
    {"index", {args.side + "_hand_v3_index_knuckle_joint", args.side + "_hand_v3_index_proximal_joint", args.side + "_hand_v3_index_middle_joint", args.side + "_hand_v3_index_distal_joint", args.side + "_hand_v3_index_proximal_virtual_joint", args.side + "_hand_v3_index_middle_virtual_joint", args.side + "_hand_v3_index_distal_virtual_joint"}},
    {"middle", {args.side + "_hand_v3_middle_knuckle_joint", args.side + "_hand_v3_middle_proximal_joint", args.side + "_hand_v3_middle_middle_joint", args.side + "_hand_v3_middle_distal_joint", args.side + "_hand_v3_middle_proximal_virtual_joint", args.side + "_hand_v3_middle_middle_virtual_joint", args.side + "_hand_v3_middle_distal_virtual_joint"}},
    {"ring", {args.side + "_hand_v3_ring_knuckle_joint", args.side + "_hand_v3_ring_proximal_joint", args.side + "_hand_v3_ring_middle_joint", args.side + "_hand_v3_ring_distal_joint", args.side + "_hand_v3_ring_proximal_virtual_joint", args.side + "_hand_v3_ring_middle_virtual_joint", args.side + "_hand_v3_ring_distal_virtual_joint"}},
    {"little", {args.side + "_hand_v3_little_knuckle_joint", args.side + "_hand_v3_little_proximal_joint", args.side + "_hand_v3_little_middle_joint", args.side + "_hand_v3_little_distal_joint", args.side + "_hand_v3_little_proximal_virtual_joint", args.side + "_hand_v3_little_middle_virtual_joint", args.side + "_hand_v3_little_distal_virtual_joint"}}
    
    // Add other hand parts here...
  };

  for (int i = 0; i < joint_state_msg_.name.size(); i++)
  {
    for (int j = 0; j < joint_names[args.hand_part].size(); j++)
    {
      if (joint_state_msg_.name[i] == joint_names[args.hand_part][j])
        joint_state_msg_.position[i] = (double)args.msg->position[j];
    }
  }
}


void getChains(const KDL::SegmentMap::const_iterator& segment, const std::string& base_link, KDL::Tree& kdl_tree, std::vector<std::string>& joint_names)
{
    if (segment->second.segment.getJoint().getType() != KDL::Joint::None)
    {
        joint_names.push_back(segment->second.segment.getJoint().getName());
    }

    for (const auto& child : segment->second.children)
    {
        getChains(child, base_link, kdl_tree, joint_names);
    }
    
}

visualize_robot::visualize_robot()
{


  std::string robot_name = std::getenv("ROBOT_NAME");
  nh.getParam("/" + robot_name + "/arm_cubes_n", arm_cubes_n);
  nh.getParam("/" + robot_name + "/AlterEgoVersion", AlterEgoVersion);

  // Topic you want to subscribe
  upperbody_state_ = nh.subscribe("/" + robot_name + "/alterego_state/upperbody", 1, &visualize_robot::callback_upperbody_state, this);
  lowerbody_state_ = nh.subscribe("/" + robot_name + "/alterego_state/lowerbody", 1, &visualize_robot::callback_lowerbody_state, this);



  // In questo esempio, sto creando un ros::Subscriber per ogni combinazione di parte della mano e lato. 
  // Sto anche costruendo il nome del topic da sottoscrivere utilizzando questi due valori.
  // Nota che sto memorizzando i sottoscrittori in un contenitore chiamato subscribers_.
  // Questo è necessario perché ros::Subscriber va fuori scope e si disiscrive automaticamente dal topic quando viene distrutto.
  // Devi quindi assicurarti di mantenere i sottoscrittori in scope per tutta la durata del tuo programma.
  // AIl membro std::vector<ros::Subscriber> subscribers_ contiene i sottoscrittori.
  for (const auto& side : sides) {
      for (const auto& hand_part : hand_parts) {
          std::string topic = "/" + side + "_hand_v3/" + hand_part + "_state";
          ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>(topic, 1, 
              [this, hand_part, side](const sensor_msgs::JointState::ConstPtr& msg){
                  CallbackArgs args;
                  args.msg = msg;
                  args.hand_part = hand_part;
                  args.side = side;
                  this->callback_hands_state(args);
              });
          // Store the subscriber in a container to keep it in scope
          subscribers_.push_back(sub);
      }
  }




  odom_pub = nh.advertise<nav_msgs::Odometry>("/" + robot_name +"/odom", 50);
  joints_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

  //---------------------------------------------------------------------------------------------------------

  // For debug on rviz:

  std::string xml_string;
  if (!nh.getParam("/robot_description", xml_string))
  {
      ROS_ERROR("Failed to get robot_description");
  }

  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromString(xml_string, kdl_tree))
  {
      ROS_ERROR("Failed to construct kdl tree");
  }

  std::vector<std::string> joint_names;
  getChains(kdl_tree.getRootSegment(), "base_link", kdl_tree, joint_names);
    //  printf("Joint names:\n");
    // for (const auto& joint_name : joint_names) {
    //     printf("  %s\n", joint_name.c_str());
    // }
  
  // Assuming joint_names_ is populated from the URDF file
  joint_state_msg_.name.resize(joint_names.size());
  joint_state_msg_.position.resize(joint_names.size());

  std::fill(joint_state_msg_.position.begin(), joint_state_msg_.position.end(), 0);

  for (size_t i = 0; i < joint_names.size(); ++i) {
      joint_state_msg_.name[i] = joint_names[i];
  }

}

// Deconstructor:

visualize_robot::~visualize_robot()
{
}

// ----------------------------------------------------------------------------------------------------------
// Function to visualize the robot model and tf in real time on Rviz:
// ----------------------------------------------------------------------------------------------------------

void visualize_robot::Publish()
{
  // --- Rviz ---

  joint_state_msg_.header.stamp = ros::Time::now();
  joints_pub.publish(joint_state_msg_);
}

// ----------------------------------------------------------------------------------------------------------
// Function to calculate everytime the position (x,y) of the unicycle (mobile base)
// ----------------------------------------------------------------------------------------------------------

void visualize_robot::odometry()
{

  transform.setOrigin(tf::Vector3(lowerbody_state_msg.mobile_base_pos_x, -lowerbody_state_msg.mobile_base_pos_y, 0.125));
  q.setRPY(0, lowerbody_state_msg.pitch_angle, -lowerbody_state_msg.yaw_angle);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));


  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(-lowerbody_state_msg.yaw_angle);
  //set the position
  odom.pose.pose.position.x = lowerbody_state_msg.mobile_base_pos_x;
  odom.pose.pose.position.y = -lowerbody_state_msg.mobile_base_pos_y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = 0;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = 0;

  //publish the message
  odom_pub.publish(odom);

}

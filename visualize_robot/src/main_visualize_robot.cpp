#include <ros/ros.h>
#include "visualize_robot.h"

#include <iostream>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>


// // Funzione ricorsiva per stampare l'albero
// void printKDLTree(const KDL::SegmentMap::const_iterator& segment, const std::string& prefix) {
//     std::cout << prefix << segment->second.segment.getName() << std::endl;
//     for (const auto& child : segment->second.children) {
//         printKDLTree(child, prefix + "  ");
//     }
// }

// // Funzione per ottenere le catene
// void getChains(const KDL::SegmentMap::const_iterator& segment, const std::string& base_link, const KDL::Tree& tree, std::vector<std::string>& joint_names) {
//     if (segment->second.segment.getJoint().getType() != KDL::Joint::None) {
//         joint_names.push_back(segment->second.segment.getJoint().getName());
//     }
//     for (const auto& child : segment->second.children) {
//         getChains(child, base_link, tree, joint_names);
//     }
// }

int main(int argc, char **argv)
{
  double rateHZ = 10;
  
  ros::init(argc, argv, "visualize_robot");
  
  ros::NodeHandle nh;

  visualize_robot Obj;  
  Obj.dt = 1 / rateHZ;
  ros::Rate r(rateHZ);
  
  // Start Debug--------------------------------------------------------------

    // // For debug on rviz:
    // std::string xml_string;
    // if (!nh.getParam("/robot_description", xml_string)) {
    //     ROS_ERROR("Failed to get robot_description");
    //     return -1;
    // }

    // KDL::Tree kdl_tree;
    // if (!kdl_parser::treeFromString(xml_string, kdl_tree)) {
    //     ROS_ERROR("Failed to construct kdl tree");
    //     return -1;
    // }
    // // Stampare l'albero KDL per il debug
    // std::cout << "KDL Tree:" << std::endl;
    // printKDLTree(kdl_tree.getRootSegment(), "");    

    // std::vector<std::string> joint_names;
    // getChains(kdl_tree.getRootSegment(), "base_link", kdl_tree, joint_names);
    // printf("Joint names:\n");
    // for (const auto& joint_name : joint_names) {
    //     printf("  %s\n", joint_name.c_str());
    // }
// End Debug--------------------------------------------------------------

  while(ros::ok())
  {
  
    
    Obj.odometry();
    Obj.Publish();

    ros::spinOnce();
    r.sleep();
    
  }
  return 0;
}

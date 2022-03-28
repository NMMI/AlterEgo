#include <urdf/model.h>
#include "ros/ros.h"

// Libraries for KDL
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>


void print_position(KDL::Frame frame){
	KDL::Vector pos = frame.p;
	std::cout << pos.data[0] << "\t" << pos.data[1] << "\t" << pos.data[2] << std::endl << std::endl;
}

void print_rotation(KDL::Frame frame){
	KDL::Rotation rot = frame.M;
	std::cout << rot.data[0] << "\t" << rot.data[1] << "\t" << rot.data[2] << std::endl;
	std::cout << rot.data[3] << "\t" << rot.data[4] << "\t" << rot.data[5] << std::endl;
	std::cout << rot.data[6] << "\t" << rot.data[6] << "\t" << rot.data[8] << std::endl << std::endl;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "my_parser");
	if (argc != 2) {
		ROS_ERROR("Need a urdf file as argument");
		return -1;
	}
	
	std::string urdf_file = argv[1]
	std::string const urdf_file = "simple_humanoid.urdf";
	
	urdf::Model model;
	if (!model.initFile(urdf_file)){
		ROS_ERROR("Failed to parse urdf file");
		return -1;
	}
	ROS_INFO("Parsing URDF file ok");

	KDL::Tree my_tree;
	if (!kdl_parser::treeFromUrdfModel(model, my_tree)){
		ROS_ERROR("Failed to construct kdl tree");
		return -1;
	}
	
	// Everything is ok, so let's try something out
	std::cout << "Number of joints in the model: ";
	std::cout << my_tree.getNrOfJoints() << std::endl;
	
	std::cout << "Number or links in the model: ";
	std::cout << my_tree.getNrOfSegments() << std::endl;
	
	// Getting the names of the various links
	std::cout << my_tree.getSegments().begin()->first << std::endl;
	//std::cout << my_tree.getSegments().end()->first << std::endl; // is wrong... why?!
	
	// Get a chain from the tree (what's the difference)
	KDL::Chain my_chain;
	if(!my_tree.getChain("base_link", "hand_R", my_chain)){
		ROS_ERROR("Failed to get chain!");
		return -1;
	}
	
	ROS_INFO("Number of segments in the chain: %d", my_chain.getNrOfSegments());
	ROS_INFO("Number of joints in the chain: %d",  my_chain.getNrOfJoints());
	
	// Let's try to compute the forward kinematics with our friends DH
	KDL::ChainFkSolverPos_recursive FKSolver(my_chain);
	
	KDL::JntArray angles = KDL::JntArray(my_chain.getNrOfJoints());
	
	KDL::Frame endEffector;
	if(FKSolver.JntToCart(angles, endEffector) == -1){
		ROS_ERROR("Problem with the JntToCart function");
		return -1;
	}
	
	// Get end effector rotation matrix
	KDL::Rotation rot(endEffector.M);
	std::cout << "**** All joints are zero ****" << std::endl;
	print_rotation(endEffector);
	
	// Get end effector position
	print_position(endEffector);
	
	angles(1) = 1.57;
	FKSolver.JntToCart(angles, endEffector);
	rot = endEffector.M;
	std::cout << "**** Rotate a little ****" << std::endl;
	print_rotation(endEffector);
	
	// Get end effector position
	print_position(endEffector);
	
	// Let's try something for the dynamics
	KDL::Vector gravity(0.0, 0.0, -9.81);
	KDL::ChainIdSolver_RNE prova = KDL::ChainIdSolver_RNE(my_chain, gravity);
	
	
	return 0;
}


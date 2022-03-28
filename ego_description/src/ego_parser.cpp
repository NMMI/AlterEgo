#include <urdf/model.h>
#include "ros/ros.h"

// Libraries for KDL
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>

// Libraries for RBDL
#include <rbdl/rbdl.h>
#include <rbdl/Constraints.h>

#include <rbdl/addons/urdfreader/urdfreader.h>

#include <Eigen/Geometry> 

#define PI 3.14

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main(int argc, char** argv){
	ros::init(argc, argv, "my_parser");
	
// 	std::string const urdf_file = "simple_humanoid.urdf";
	
	Model* model = new Model();
	
	
	/*if (!Addons::URDFReadFromFile ("/home/federico/catkin_ws/src/onearm_ego/onearm_description/urdf/onearm_ego.xacro", model, false)) {
		std::cerr << "Error loading model ./onearm_ego.xacro" << std::endl;
		//abort();
	}
	ROS_INFO("Parsing URDF file ok");*/
	
	if (!Addons::URDFReadFromFile ("/home/grazia/AlterEgo_ws/src/AlterEgo/ego_description/urdf/egoarm_6DoFs_wrist.urdf", model, false, false)) {
		std::cerr << "Error loading model ./onearm_ego.xacro" << std::endl;
		//abort();
	}	
	ROS_INFO("Parsing URDF file ok");	
	
	model->gravity = Vector3d (0., 0., -9.81);
	
// 	std::cout << "Number of joints: " << std::endl;
// 	std::cout << model->q_size << 	std::endl;
// 	
// 	std::cout << "Degree of freedom overview:" << std::endl;
// 	std::cout << Utils::GetModelDOFOverview(*model);
// 
// 	std::cout << "Model Hierarchy:" << std::endl;
// 	std::cout << Utils::GetModelHierarchy(*model);
// 	
	VectorNd Q = VectorNd::Zero (model->q_size);
	VectorNd QDot = VectorNd::Zero (model->qdot_size);
	VectorNd C = VectorNd::Zero (model->qdot_size);
	VectorNd QDDot = VectorNd::Zero (model->qdot_size);
	
// 	Q(3) = 0.7;
// 	Q(6) = 0.0;
// 	Q(9) = 0.0;

// 	std::cout << "Forward Dynamics:" << std::endl;
// 	ForwardDynamics (*model, Q, QDot, C, QDDot);
// 
// 	std::cout << QDDot.transpose() << std::endl;
// 	
// 	MatrixNd M = MatrixNd::Zero(model->q_size, model->q_size);
// 	CompositeRigidBodyAlgorithm(*model, Q, M);
// 	
// 	std::cout << "Inertia matrix:" << std::endl;
// 	std::cout << M << std::endl;
	
	Q(0) = 3.14*2/3;
	NonlinearEffects(*model, Q, QDot, C);
	
	std::cout << "Gravity" << std::endl;
	std::cout << C << std::endl;
// 	
// 	/* Now, let's try to implement a feedback linearized controller
// 	 * 
// 	 */
// 	
// // 	std::cout << "Body name: " << std::endl;
// // 	std::cout << model->GetBodyName(0) << 	std::endl;
// 	
// 	std::cout << "base_link Id: " << std::endl;
// 	std::cout << model->GetBodyId("torso") << std::endl;
// 	
// 	/* Compute some forward kinematics, just to see some numbers that make sense */
// 	Vector3d EE = CalcBodyToBaseCoordinates(*model, Q, 2147483647, Vector3d(0., 0., 0.));
// 	
// 	std::cout << "torso" << std::endl;
// 	std::cout << EE << std::endl;
// 	
// 	std::cout << "wheel_L Id: " << std::endl;
// 	std::cout << model->GetBodyId("wheel_L") <<std::endl;
// 	
// 	/* Compute some forward kinematics, just to see some numbers that make sense */
// 	EE = CalcBodyToBaseCoordinates(*model, Q, 5, Vector3d(0., 0., 0.));
// 	
// 	std::cout << "wheel_L_link" << std::endl;
// 	std::cout << EE << std::endl;
// 	
// 	std::cout << "Wheel_R Id: " << std::endl;
// 	std::cout << model->GetBodyId("wheel_R") <<std::endl;
// 	
// 	/* Compute some forward kinematics, just to see some numbers that make sense */
// 	EE = CalcBodyToBaseCoordinates(*model, Q, 6, Vector3d(0., 0., 0.));
// 	
// 	std::cout << "wheel_R link" << std::endl;
// 	std::cout << EE << std::endl;
// 	
// 	std::cout << "neck_Id: " << std::endl;
// 	std::cout << model->GetBodyId("neck") <<std::endl;
// 	
// 	/* Compute some forward kinematics, just to see some numbers that make sense */
// 	EE = CalcBodyToBaseCoordinates(*model, Q, model->GetBodyId("neck"), Vector3d(0., 0., 0.));
// 	
// 	std::cout << "neck_link" << std::endl;
// 	std::cout << EE << std::endl;
// 	
// 	std::cout << "right_EE: " << std::endl;
// 	std::cout << model->GetBodyId("right_EE") <<std::endl;
// 	
// 	Eigen::VectorXd rhand_0;
// 	/* Compute some forward kinematics, just to see some numbers that make sense */
// 	rhand_0 = CalcBodyToBaseCoordinates(*model, Q, model->GetBodyId("left_EE"), Vector3d(0., -0.08, 0.));
// 	
// 	std::cout << "right_hand link" << std::endl;
// 	std::cout << rhand_0 << std::endl;
// 	
// // //	the first three rows are the angulare part and the last three are che the linear part
// // 	MatrixNd Jrhand = MatrixNd::Zero(6, model->q_size);
// // 	CalcPointJacobian6D(*model, Q, model->GetBodyId("right_EE"), Vector3d(0., -0.08, 0.), Jrhand);
// // 	
// // //	the first three rows are the the linear part and the other zero	
// // // 	CalcPointJacobian(*model, Q, model->GetBodyId("right_EE"), Vector3d(0., -0.08, 0.), Jrhand);
// // 
// // 	std::cout << "Jrhand:" << std::endl;
// // 	std::cout << Jrhand << std::endl;
// 	
// 	Vector3d com_pos;
// 	com_pos  = Vector3d(0., 0., 0.);
// 	Vector3d com_vel = Vector3d(0., 0., 0.);
// 	
// 	double M_tot;
// 	
// 	
//         Utils::CalcCenterOfMass( *model, Q, QDot, &QDDot, M_tot, com_pos, &com_vel); 		
// 	std::cout << "M_tot: " << M_tot <<std::endl;
// 	std::cout<< "Com_pos: "<<std::endl;
// 	std::cout<< com_pos <<std::endl;
// 	
// 	std::cout<< "Com_vel: "<<std::endl;
// 	std::cout<< com_vel <<std::endl;
// 	
// 	Eigen::VectorXd masses = Eigen::VectorXd::Zero(13);
// 	masses << 13.3260, 1.22, 1.22,  0.6010, 0.6430, 0.5910, 0.3530, 0.3810, 0.6010, 0.6430, 0.5910, 0.3530, 0.3810;
// 	
// 	VectorNd index_link= VectorNd::Zero(13);
// 	
// 	index_link(0) = model->GetBodyId("torso");
// 	index_link(1) = model->GetBodyId("wheel_L");
// 	index_link(2) = model->GetBodyId("wheel_R");
// 	index_link(3) = model->GetBodyId("left_shoulder_cube");
// 	index_link(4) = model->GetBodyId("left_arm_cube");
// 	index_link(5) = model->GetBodyId("left_elbow_cube");
// 	index_link(6) = model->GetBodyId("left_forearm_cube");
// 	index_link(7) = model->GetBodyId("left_EE");
// 	index_link(8) = model->GetBodyId("right_shoulder_cube");
// 	index_link(9) = model->GetBodyId("right_arm_cube");
// 	index_link(10) = model->GetBodyId("right_elbow_cube");
// 	index_link(11) = model->GetBodyId("right_forearm_cube");
// 	index_link(12) = model->GetBodyId("right_EE");
// 	
// // 	std::cout << std::setprecision(30);
// 	std::cout<<"Index vector of the link"<<std::endl;
// 	std::cout<< index_link<<std::endl;
// 	
// 	Eigen::MatrixXf rel_pos_com_i(3,13);
// 	
// 	rel_pos_com_i.col(0) << -4.9e-3, 0, 0.2871;
// 	rel_pos_com_i.col(1) << 0,0,0;
// 	rel_pos_com_i.col(2) << 0,0,0;
// 	rel_pos_com_i.col(3) << 0.001336, 0, -3.15e-3;
// 	rel_pos_com_i.col(4) << 9.25e-4,  0.08266, 0;
// 	rel_pos_com_i.col(5) << 0,  -0.0025, 1.36e-3;
// 	rel_pos_com_i.col(6) << 0,  0.07663, -1.78e-3;
// 	rel_pos_com_i.col(7) << 4.62e-3, 0.062, -4.88e-3;
// 	rel_pos_com_i.col(8) << -0.001336, 0, -3.15e-3;
// 	rel_pos_com_i.col(9) << 9.25e-4,  -0.08266, 0;
// 	rel_pos_com_i.col(10) << 0,  0.0025, 1.36e-3;
// 	rel_pos_com_i.col(11) << 0, -0.07663, -1.78e-3;
// 	rel_pos_com_i.col(12) << 4.62e-3, -0.062, -4.88e-3;
// 	
// 	Eigen::MatrixXd CoM_jacobian = Eigen::MatrixXd::Zero(3,model->q_size);
// 	MatrixNd J = MatrixNd::Zero(3, model->q_size);
// 	Vector3d com_pos_link;
// 	
// 	for (int i = 0; i<13; i++)
// 	{
// 	  com_pos_link = Vector3d(rel_pos_com_i(0,i), rel_pos_com_i(1,i), rel_pos_com_i(2,i) );
// 	  CalcPointJacobian(*model, Q, index_link(i), com_pos_link, J);
// 	  CoM_jacobian = CoM_jacobian + J*masses(i)/M_tot;
// 	}
// 	
// 	std::cout<< "Jcom: "<<std::endl;
// 	std::cout<< CoM_jacobian <<std::endl;
//  	
// 	Utils::CalcCenterOfMass( *model, Q, QDot, &QDDot, M_tot, com_pos, &com_vel); 	
// 	std::cout<< "Com_pos: "<<std::endl;
// 	std::cout<< com_pos <<std::endl;
// 	
// 	Matrix3d Rhand_rot = CalcBodyWorldOrientation (*model, Q,model->GetBodyId("wheel_L"));
// 	Quaternion Quat;
// 	std::cout<<"Rhand rbdl"<<std::endl;
// 	std::cout<<Rhand_rot<<std::endl;
// 	
// 	Quat=Quaternion::fromMatrix(Rhand_rot);
// 	 std::cout<<"Quaternion"<<std::endl;
// 	 std::cout<<Quat<<std::endl;
// 	double x;
// 	double y; 
// 	double z;
// 	double w;
// 	double roll;
// 	double pitch;
// 	double yaw;
// 
// 	double x_des;
// 	double y_des;
// 	double z_des;
// 	double w_des;	
// 	
// 	
// 	KDL::Rotation RwheelDrot;
// 	RwheelDrot = KDL::Rotation::Quaternion(0,0,0,1);
// 	RwheelDrot.DoRotZ(1.57);
// 	
// 	
// // 	MatrixNd W_inv = MatrixNd::Zero(model->q_size, model->q_size);
// 	Quaternion Quat_des;
// 	
// 	RwheelDrot.GetQuaternion(Quat_des(0), Quat_des(1), Quat_des(2), Quat_des(3));
// 	std::cout<<std::endl;
// 	std::cout<< Quat_des(0) <<std::endl;
// 	std::cout<<Quat_des(1)<<std::endl;
// 	std::cout<<Quat_des(2)<<std::endl;
// 	std::cout<<Quat_des(3)<<std::endl;
// 	
// 	int N = 100;
// 	
// 	Quaternion Quat_curr;
// 	double alpha;
// 	
// 	for(int i = 0; i<=N; i++)
// 	{
// 	  alpha = (double)i/N;
// 	  std::cout <<"Alpha : "<<alpha<<std::endl;
// 	  Quat_curr = Quat.slerp(alpha, Quat_des);
// 	  std::cout<<"Quaternion computed"<<std::endl;
// 	  std::cout<<Quat_curr<<std::endl;
// 	  
// 	}

	return 0;
}


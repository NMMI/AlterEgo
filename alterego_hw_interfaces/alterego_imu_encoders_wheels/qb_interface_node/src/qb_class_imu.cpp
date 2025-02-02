
#include "qb_class_imu.h"

//-----------------------------------------------------
//                                         qb_class_imu
//-----------------------------------------------------

/*
/ *****************************************************
/ Costructor of qb_class_imu class
/ *****************************************************
/   parameters:
/				port - # of IMUs connected
/					   
/   return:
/
*/

qb_class_imu::qb_class_imu(){

	// Variables to get param 
	vector<int> ID_imuboard;
	bool isOldBoard = false;

	string aux;
	std::string ns_name, systemcheck_path;
	// Initialize ROS Node
	node_ = new ros::NodeHandle("qb_interface_node_imu_");
	ns_name = ros::this_node::getNamespace();

	//---------------Set SystemCheck variable from rosparam to make it available for qbImuBoard 
	std::string robot_name = std::getenv("ROBOT_NAME");
	node_->getParam("/"+robot_name+"/SystemCheckPath", systemcheck_path);
	string varName = "SYSTEMCHECK_PATH=";
    string varValue = varName+systemcheck_path;
	char* scpath = new char[varValue.size() + 1];
	std::strcpy(scpath, varValue.c_str());
	putenv( scpath );
	//---------------


	// Get param from roslaunch or yaml file
	node_->searchParam(ns_name+"/IDimuboards", aux);
	node_->getParam(aux, ID_imuboard);
	node_->getParam(ns_name+"/imu_frequency", imu_frequency);
	//node_->param<double>("/step_time_imu", step_time_imu_, 0.002);
	
	/********************* Not needed for AlterEgo 	
	node_->param<int>("/hand_step_div", hand_step_div_, 10);
	*******************************************/
	
	node_->param<bool>(ns_name+"/compute_angles", compute_angles_, true);
	node_->getParam(ns_name+"/is_old_board", isOldBoard);


    qbImuBoard* tmp_imuboard;

    for (int i = ID_imuboard.size(); i--;) {

        tmp_imuboard = new qbImuBoard(qb_comm_, ID_imuboard[i], isOldBoard);
        
       	// IF an error is find
        if (tmp_imuboard == NULL){
        	cout << "[ERROR] Unable to allocate space for imu board structure." << endl;
            return;
        }

        imuboard_chain_.push_back(tmp_imuboard);
    } 


	// Initialize publisher and subscriber

	if (!imuboard_chain_.empty()){

		// Publisher initialize
		imuboard_pub_acc_  = node_->advertise<qb_interface::inertialSensorArray>(ns_name+"/qb_class_imu/acc", 1);
		imuboard_pub_gyro_ = node_->advertise<qb_interface::inertialSensorArray>(ns_name+"/qb_class_imu/gyro", 1);
		imuboard_pub_mag_  = node_->advertise<qb_interface::inertialSensorArray>(ns_name+"/qb_class_imu/mag", 1);
		imuboard_pub_quat_ = node_->advertise<qb_interface::quaternionArray>(ns_name+"/qb_class_imu/quat", 1);
		imuboard_pub_temp_ = node_->advertise<qb_interface::temperatureArray>(ns_name+"/qb_class_imu/temp", 1);

		if (compute_angles_) {
			imuboard_pub_angles_ = node_->advertise<qb_interface::anglesArray>(ns_name+"/qb_class_imu/angles", 1);
		}

		Acc_.resize(imuboard_chain_[0]->n_imu_,3);
		Acc_old_.resize(imuboard_chain_[0]->n_imu_,3);
		Gyro_.resize(imuboard_chain_[0]->n_imu_,3);
		Ext_Quat_.resize(imuboard_chain_[0]->n_imu_,4);
		Angles_.resize(imuboard_chain_[0]->n_imu_, 3);

		Acc_.setZero();
		Acc_old_.setZero();
		Gyro_.setZero();
		Ext_Quat_.setZero();
		Angles_.setZero();


		for (int k = imuboard_chain_.size(); k--;){
			for (int i = 0; i < imuboard_chain_[k]->n_imu_; i++) 
			{
				Ext_Quat_(i,0) = 1;
			}
		}

	}


}

//-----------------------------------------------------
//                                            ~qb_class_imu
//-----------------------------------------------------

/*
/ *****************************************************
/ Descructor of qb_class_imu class
/ *****************************************************
/   parameters:
/   return:
/
*/


qb_class_imu::~qb_class_imu(){

}




//-----------------------------------------------------
//                                              readIMU
//-----------------------------------------------------

/*
/ *****************************************************
/ Read measurements of all IMUs
/ *****************************************************
/   parameters:
/   return:
/               [state]
/
*/
bool qb_class_imu::readIMU(){
	qb_interface::inertialSensor tmp_acc, tmp_gyro, tmp_mag;
	qb_interface::inertialSensorArray acc, gyro, mag;
	qb_interface::quaternion tmp_quat;
	qb_interface::quaternionArray quat;
	qb_interface::temperature tmp_temp;
	qb_interface::temperatureArray temp;
	qb_interface::angles tmp_angles;
	qb_interface::anglesArray angles;

	// std::cout << "# board " << imuboard_chain_.size() << std::endl;	

	for (int k = imuboard_chain_.size(); k--;){
	    if (imuboard_chain_[k]->getImuReadings() < 0)
	    {
	    	//std::cout << "sono qui"<<std:: endl;
	    	return false;

	    }
		
		for (int i = 0; i < imuboard_chain_[k]->n_imu_; i++) 
		{
			
			// printf("IMU: %d\n", imuboard_chain_[k]->ids_[i]);
		
			if (imuboard_chain_[k]->imu_table_[5*i + 0])
			{
				tmp_acc.id = imuboard_chain_[k]->ids_[i];
				tmp_acc.x  = imuboard_chain_[k]->imu_values_[(3*3+4+1)*i];
				tmp_acc.y  = imuboard_chain_[k]->imu_values_[(3*3+4+1)*i+1];
				tmp_acc.z  = imuboard_chain_[k]->imu_values_[(3*3+4+1)*i+2];
				acc.m.push_back(tmp_acc);

				Acc_(i,0) = tmp_acc.x; 
				Acc_(i,1) = tmp_acc.y; 
				Acc_(i,2) = tmp_acc.z; 
			}
			if (imuboard_chain_[k]->imu_table_[5*i + 1])
			{
				tmp_gyro.id = imuboard_chain_[k]->ids_[i];
				tmp_gyro.x  = imuboard_chain_[k]->imu_values_[(3*3+4+1)*i+3];
				tmp_gyro.y  = imuboard_chain_[k]->imu_values_[(3*3+4+1)*i+4];
				tmp_gyro.z  = imuboard_chain_[k]->imu_values_[(3*3+4+1)*i+5];
				gyro.m.push_back(tmp_gyro);

				Gyro_(i,0) = tmp_gyro.x; 
				Gyro_(i,1) = tmp_gyro.y; 
				Gyro_(i,2) = tmp_gyro.z; 
			}

			if (imuboard_chain_[k]->imu_table_[5*i + 2] )
			{
				tmp_mag.id = imuboard_chain_[k]->ids_[i];
				tmp_mag.x  = imuboard_chain_[k]->imu_values_[(3*3+4+1)*i+6];
				tmp_mag.y  = imuboard_chain_[k]->imu_values_[(3*3+4+1)*i+7];
				tmp_mag.z  = imuboard_chain_[k]->imu_values_[(3*3+4+1)*i+8];
				mag.m.push_back(tmp_mag);
			}

			
			if (imuboard_chain_[k]->imu_table_[5*i+3])
			{
				tmp_quat.id = imuboard_chain_[k]->ids_[i];
				tmp_quat.w  = imuboard_chain_[k]->imu_values_[(3*3+4+1)*i+9];
				tmp_quat.x  = imuboard_chain_[k]->imu_values_[(3*3+4+1)*i+10];
				tmp_quat.y  = imuboard_chain_[k]->imu_values_[(3*3+4+1)*i+11];
				tmp_quat.z  = imuboard_chain_[k]->imu_values_[(3*3+4+1)*i+12];
				quat.m.push_back(tmp_quat);
			} 

			/********************* Not needed for AlterEgo 

			else { // [GS] Parte che calcola il quaternione
				if (imuboard_chain_[k]->imu_table_[5*i + 0] && imuboard_chain_[k]->imu_table_[5*i + 1]) { // Controllo se ho il necessario per calcolare il quaternione
					Ext_Quat_Computer(i);
					tmp_quat.id = imuboard_chain_[k]->ids_[i];
					tmp_quat.w  = Ext_Quat_(i,0);
					tmp_quat.x  = Ext_Quat_(i,1);
					tmp_quat.y  = Ext_Quat_(i,2);
					tmp_quat.z  = Ext_Quat_(i,3);
					quat.m.push_back(tmp_quat);

					if (compute_angles_) {
						Quat_to_Angles(i);
						tmp_angles.id = imuboard_chain_[k]->ids_[i];
						tmp_angles.r = Angles_(i,0);
						tmp_angles.p = Angles_(i,1);
						tmp_angles.y = Angles_(i,2);
						angles.m.push_back(tmp_angles);
					}
				}
			}
			*******************************************/
			
			if (imuboard_chain_[k]->imu_table_[5*i+4])
			{
				tmp_temp.id = imuboard_chain_[k]->ids_[i];
				tmp_temp.value  = imuboard_chain_[k]->imu_values_[(3*3+4+1)*i+13];
				temp.m.push_back(tmp_temp);
			}
			
			// verify if this usleep is needed
			//usleep(0.5);
		}
		usleep(1);
	}

	// if (1)
	if ((Acc_old_ - Acc_).sum() != 0)
 	{
		imuboard_pub_acc_.publish(acc);
		imuboard_pub_gyro_.publish(gyro);	
		
		/********************* Not needed for AlterEgo 
		imuboard_pub_mag_.publish(mag);	
		imuboard_pub_quat_.publish(quat);
		imuboard_pub_temp_.publish(temp);
		*******************************************/

		if (compute_angles_){
			imuboard_pub_angles_.publish(angles);
		}
 	}

 	Acc_old_ = Acc_;

 	return true;

}


//-----------------------------------------------------
//                                    Ext_Quat_Computer
//-----------------------------------------------------

/*
/ *****************************************************
/ Compute the Orientation Quaternion of all IMUs
/ *****************************************************
/   parameters:
/   return:
/               [state]
/
*/
void qb_class_imu::Ext_Quat_Computer(int n) {

	float q1, q2 ,q3 ,q4;
	float sx, sy, sz;
	Eigen::Vector3d aP,fa;
    Eigen::Vector4d gP,qL,qdot,Napla;
    Eigen::MatrixXd Ja(3,4); 

   	float Min_Acc_Norm = 0.8;
	float Max_Acc_Norm = 1.2;
	float beta = 2.0;
	float sampleFreq = 50;
	float Gyro_Th = 18;

    // Check and Prepare Accelerations
	aP(0)  = Acc_(n,0);  
	aP(1)  = Acc_(n,1);  
	aP(2)  = Acc_(n,2);

	if (aP.norm() < Min_Acc_Norm) {
		aP(0) = Acc_old_(n,0); 
		aP(1) = Acc_old_(n,1); 
		aP(2) = Acc_old_(n,2); 
	}
	if (aP.norm() > Max_Acc_Norm) {
		//std::cout<<"\n\033[1;32mSono Grosso \033[0m"<< "P:" << P << std::endl;
		aP(0) = Acc_old_(n,0); 
		aP(1) = Acc_old_(n,1); 
		aP(2) = Acc_old_(n,2); 
	}	

 	aP = aP / aP.norm();
	sx = aP(0); 
	sy = aP(1); 
	sz = aP(2); 
	
 	// Check And Prepare Gyros
 	if (fabs(Gyro_(n,0) < Gyro_Th)) {Gyro_(n,0) = 0;}
   	if (fabs(Gyro_(n,1) < Gyro_Th)) {Gyro_(n,1) = 0;}
    if (fabs(Gyro_(n,2) < Gyro_Th)) {Gyro_(n,2) = 0;}
	gP(0)  = 0; 
	gP(1)  = Gyro_(n,0);  
	gP(2)  = Gyro_(n,1);  
	gP(3)  = Gyro_(n,2);
	gP = gP*(M_PI/180);
	
	// Prepare Quaternion
	q1 = Ext_Quat_(n,0);  
	q2 = Ext_Quat_(n,1); 
	q3 = Ext_Quat_(n,2); 
	q4 = Ext_Quat_(n,3);

	qL(0) = q1;
	qL(1) = q2;
	qL(2) = q3;
	qL(3) = q4;

	// Cost Function
	fa(0) =  2*(q2*q4-q1*q3) - sx; 
	fa(1) =  2*(q1*q2 + q3*q4) - sy;
	fa(2) =  2*(0.5 - q2*q2 -q3*q3) - sz; 	
	
	// Compute the Jacobian
	Ja << -2*q3,	2*q4,	-2*q1,	2*q2,
		   2*q2,    2*q1,	 2*q4,  2*q3,
		   	  0,   -4*q2,   -4*q3,     0;
	
	// Compute the Napla
  	Napla = Ja.transpose() * fa;
	
	//qdot = 0.5*QxQ( qL,g ) - ( beta*Napla );				
	qdot(0) = qL(0)*gP(0) - (qL(1)*gP(1) + qL(2)*gP(2) + qL(3)*gP(3));
	qdot(1) = qL(0)*gP(1) + qL(1)*gP(0) + (qL(2)*gP(3) - qL(3)*gP(2));
	qdot(2) = qL(0)*gP(2) + qL(2)*gP(0) + (qL(3)*gP(1) - qL(1)*gP(3));
	qdot(3) = qL(0)*gP(3) + qL(3)*gP(0) + (qL(1)*gP(2) - qL(2)*gP(1));

	qdot = 0.5*qdot - (beta*Napla);

	qL = qL + qdot / sampleFreq;  	
	qL = qL /qL.norm();

	Ext_Quat_(n,0) = qL(0);
	Ext_Quat_(n,1) = qL(1);
	Ext_Quat_(n,2) = qL(2);
	Ext_Quat_(n,3) = qL(3);
	
}


//-----------------------------------------------------
//                                       Quat_to_Angles
//-----------------------------------------------------

/*
/ *****************************************************
/ Compute the Angles from a given Quaternion of all IMUs
/ *****************************************************
/   parameters:
/   return:
/               [state]
/
*/
void qb_class_imu::Quat_to_Angles(int n) {

    float w = Ext_Quat_(n, 0);
    float x = Ext_Quat_(n, 1);
    float y = Ext_Quat_(n, 2);
	float z = Ext_Quat_(n, 3);

	Angles_(n,0) = - atan2(2*x*w - 2*y*z, 1-2*x*x - 2*z*z)*(180/M_PI); //ROLL
	Angles_(n,1) = - atan2(2*y*w - 2*x*z, 1-2*y*y - 2*z*z)*(180/M_PI); // PITCH	
	Angles_(n,2) =  asin(2*x*y + 2*z*w)*(180/M_PI); //YAW
}

//-----------------------------------------------------
//                                             spinOnce
//-----------------------------------------------------

/*
/ *****************************************************
/ Read all devices and set position if new ref. is
/ arrived.
/ *****************************************************
/   parameters:
/   return:
/
*/

void qb_class_imu::spinOnce(){

	/********************* Not needed for AlterEgo 
	static int counter = 0;

	if (counter > hand_step_div_){
		qb_class::spinOnce();
		counter = 0;
	}
	counter++;
	*******************************************/

	// Read measurementes of all IMUs and send them on topics
	readIMU();

}

//-----------------------------------------------------
//                                                 spin
//-----------------------------------------------------

/*
/ *****************************************************
/ Read all devices and set position if new ref. is 
/ arrived.  
/ *****************************************************
/   parameters:
/   return:
/
*/

void qb_class_imu::spin(){

	// 1/step_time is the rate in Hz
	//ros::Rate loop_rate(1.0 / step_time_imu_);
	ros::Rate loop_rate(imu_frequency);

	while(ros::ok()) {
		readIMU();

		ros::spinOnce();

		loop_rate.sleep();
	}

}
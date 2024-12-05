#include <stdio.h>
#include <stdlib.h>

#include <time.h>

#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

using namespace std;

#define N_IMU 			1
#define N_AXES			3
#define USEFUL_SAMPLES 	10
#define BP_FILT_ORDER	4//8

float acc_values[N_IMU][N_AXES];

void acc_read__Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	for(int i = 0; i < msg->data.size(); i++) {
		acc_values[i/N_AXES][i%N_AXES] = msg->data[i];
	}
}

// 250-300 Hz bandpass filter with stopband freq 200,350 Hz @ -100db attenuation
double bandpass_filter(double new_value, uint8_t idx, uint8_t dir){

	static double vect_acc[N_IMU][N_AXES][BP_FILT_ORDER + 1];			// Last samples
	static double vect_acc_filt[N_IMU][N_AXES][BP_FILT_ORDER + 1];	// Filter output last samples

/* OLD FILTER (KATIA)
	static double A[BP_FILT_ORDER + 1] = { 1.0, 0.19227, 1.09197, 0.15811, 0.72636 };
	static double B[BP_FILT_ORDER + 1] = { 0.06423, 0, -0.12845, 0, 0.06423};
*/
	// 4-th order bandpass filter with 3db attenuation @ 175 Hz - 225 Hz range frequencies
	// static double A[BP_FILT_ORDER + 1] = { 1.0, 0, 1.6185, 0, 0.7106 };
	// static double B[BP_FILT_ORDER + 1] = { 0.0205, 0, -0.041, 0, 0.0205 };

//	static double A[BP_FILT_ORDER + 1] = {1.0, 3.368, 4.359, 3.198, 2.130, 1.230, 0.302, 0.014, 0.019};
//	static double B[BP_FILT_ORDER + 1] = {0.124, 0, -0.496, 0, 0.744, 0, -0.496, 0, 0.124};

//Last bandpass (good)
	// static double A[BP_FILT_ORDER + 1] = {1.0, 0.9619, 1.0342, 0.5415, 0.3939};
	// static double B[BP_FILT_ORDER + 1] = {0.1382, 0, -0.2765, 0, 0.1382};

//Highpass
	static double A[BP_FILT_ORDER + 1] = {1.0, 0.3051, 0.8802, 0.3576, 0.2068};
	static double B[BP_FILT_ORDER + 1] = {0.0793, -0.3174, 0.476, -0.3174, 0.0793};

//	static double A[BP_FILT_ORDER + 1] = { 1.0, 1.7306, 0.7841, 0.3765, 0.3229 };
//	static double B[BP_FILT_ORDER + 1] = { 0.3548, 0, -0.7096, 0, 0.3548 };
    // Shift previous values by one step
    for (int i = BP_FILT_ORDER; i >= 1; i--){
    	vect_acc[idx][dir][i] = vect_acc[idx][dir][i-1];
    	vect_acc_filt[idx][dir][i] = vect_acc_filt[idx][dir][i-1];
    }
    vect_acc[idx][dir][0] = new_value;

    // Compute filter output
    vect_acc_filt[idx][dir][0] = B[0]*vect_acc[idx][dir][0];
    for (int i = 1; i <= BP_FILT_ORDER; i++){
    	vect_acc_filt[idx][dir][0] += B[i]*vect_acc[idx][dir][i] - A[i]*vect_acc_filt[idx][dir][i]; 
    }

    return ( vect_acc_filt[idx][dir][0] ); 
}

double outliersRemoval(double u, double x[USEFUL_SAMPLES], double score_thr){

	double mu = 0.0;
	double sigma = 0.0;
    double sum = 0.0;
    int i = 0;
    double y;

    for(i = 0; i < USEFUL_SAMPLES; i++)
    {
        sum += x[i];
    }

    mu = sum / USEFUL_SAMPLES;

    for(i = 0; i < USEFUL_SAMPLES; i++)
        sigma += pow(x[i] - mu, 2);

    sigma = sqrt(sigma / (USEFUL_SAMPLES-1));
	if (sigma < 1e-3){
		sigma = 1e-3;
	}

	if (fabs(u - mu)/sigma < score_thr){
		y = u;
	}
	else {
		y = 0.9 * x[USEFUL_SAMPLES-1] + (1.0 - 0.9) * u;
	}

	// Shift previous values by one step
    for (int i = 0; i < USEFUL_SAMPLES-1; i++){
    	x[i] = x[i+1];
    }
    x[USEFUL_SAMPLES - 1] = y;

    return y;
}

void updateCovarianceMatrix(int imu_idx, double u[N_AXES], double covM[N_IMU][N_AXES][N_AXES]){
	
	static double x[N_IMU][N_AXES][USEFUL_SAMPLES];
	double mu[N_AXES];

	// Shift previous values by one step
    for (int i = 0; i < N_AXES; i++){

    	for (int j = 0; j < USEFUL_SAMPLES-1; j++){
	    	x[imu_idx][i][j] = x[imu_idx][i][j+1];
	    }
	    x[imu_idx][i][USEFUL_SAMPLES-1] = u[i];
    }
    
    for (int i = 0; i < N_AXES; i++){

    	double sum = 0.0;
	    for(int j = 0; j < USEFUL_SAMPLES; j++){
	        sum += x[imu_idx][i][j];
	    }

	    mu[i] = sum / USEFUL_SAMPLES;
    }


    // Update covariance matrix components
    for (int i = 0; i < N_AXES; i++){
    	for (int j = 0; j < N_AXES; j++){

    		covM[imu_idx][i][j] = 0.0;

    		// C = x * x' - mu * mu'
    		for(int k = 0; k < N_AXES; k++){
             	covM[imu_idx][i][j] += x[imu_idx][i][k] * x[imu_idx][j][k];
            }
            covM[imu_idx][i][j] -= mu[i]*mu[j]; 

    	}

    }
}


void updateRelevanceComponentsVector(int imu_idx, double covM[N_IMU][N_AXES][N_AXES], double v[N_IMU][N_AXES], double relevance_comp_v_weight){

	double cV[N_IMU][N_AXES];
	double normV = 0.0;

	for(int i = 0; i < N_AXES; i++){

		cV[imu_idx][i] = 0.0;

		// v = alpha * v  + (1-alpha) * C*v;
		for(int k = 0; k < N_AXES; k++){
         	cV[imu_idx][i] += covM[imu_idx][i][k] * v[imu_idx][k];
        }

		v[imu_idx][i] = relevance_comp_v_weight * v[imu_idx][i] + (1.0 - relevance_comp_v_weight) * cV[imu_idx][i];
	}

	// v = v / norm(v)
	for(int i = 0; i < N_AXES; i++){
		normV += v[imu_idx][i]*v[imu_idx][i];
	}
	normV = sqrt(normV);

	for(int i = 0; i < N_AXES; i++){
		v[imu_idx][i] /= normV;
	}

}


int main(int argc, char **argv)
{
    double 				run_freq(800);			// Desired main rate in Hz [800 Hz -> 1250 us]
	
	string				ns;
	string 				acc_hand_topic;
	string 				act_fb_topic;
	bool				pub_topic_enabled(false);
	
	std_msgs::Float64 	msg_act_fb;

	double score_thr = 20;
	double relevance_comp_v_weight = 0.1;

	bool acc_init[N_IMU];
	double acc_history[N_AXES][USEFUL_SAMPLES];
	double vect_filt[N_IMU][N_AXES];
	double covariance_matrix[N_IMU][N_AXES][N_AXES];
	double relevance_comp_v[N_IMU][N_AXES];
    double input_act[N_IMU];
    for (int i = 0; i < N_IMU; i++){
    	input_act[i] = 0;
    } 
    double actValue = 0.0;
    bool first_disconnect = false;

    ros::init(argc,argv,"vibrotactile_fb");  // Initiate new ROS node

	// NodeHandle is the main access point to communications with the ROS system. The first NodeHandle constructed will fully initialize this node, and the last NodeHandle destructed will close down the node.
	ros::NodeHandle n;  

	n.getParam("act_fb_topic", act_fb_topic);	
	n.getParam("score_thr", score_thr);
	n.getParam("relevance_comp_v_weight", relevance_comp_v_weight);
	n.getParam("pub_act_fb", pub_topic_enabled);
	n.getParam("acc_hand_topic", acc_hand_topic);
	
	ros::Subscriber sub_acc		= n.subscribe(acc_hand_topic, 1, acc_read__Callback);
	ros::Publisher 	pub_act_fb  = n.advertise<std_msgs::Float64>(act_fb_topic, 1);


	n.getParam("/vibrotactile_fb_frequency", run_freq);
	ros::Rate loop_rate(run_freq);
	
  	ns = ros::this_node::getNamespace();

/************************************* VIBROTACTILE FEEDBACK INIT ********************************/
	
	// Init vector
	for (int i = 0; i < N_IMU; i++){
		acc_init[i] = false;

		relevance_comp_v[i][0] = 0;
		relevance_comp_v[i][1] = 0;
		relevance_comp_v[i][2] = 1;

		for (int j = 0; j < N_AXES; j++){

			for (int k = 0; k < N_AXES; k++){
				covariance_matrix[i][j][k] = 0.0;
			}	
		}
	}

	for (int i = 0; i < N_AXES; i++){
		// Smart parametric way to init each axis vector as {0, 1, -1, 0, 1, -1, 0, 1, -1, 0}
		for (int j = 0; j < USEFUL_SAMPLES; j++){
			acc_history[i][j] = (j%3 == 2)?(-1):(j%3);
		}
	}

	for (int i = 0; i < N_IMU; i++){
		for (int j = 0; j < N_AXES; j++){
			acc_values[i][j] = 0.0;
		}
	}

    cout << endl << "Vibrotactile feedback configured and active on " << ns.substr(1,ns.length()-1) << " hand" << endl;
    if (pub_act_fb){
    	cout << "Publishing actuators input commands on topic " << ns << "/" << act_fb_topic << endl;
    }


	while ( ros::ok() )
	{
			
		/************************* VIBRO **********************************/
		for (int i = 0; i < N_IMU; i++){

			// Wait for first message reading (values initialization)
			if (acc_values[i][0] || acc_values[i][1] || acc_values[i][2]){
				acc_init[i] = true;
			}

			if (acc_init[i]){


				for (int j = 0; j < N_AXES; j++){

					// Outliers removal
					double acc_value_to_filt = outliersRemoval(acc_values[i][j], &acc_history[j][0], score_thr);

					// Signal filtering with the VT bandpass filter
					vect_filt[i][j] = bandpass_filter(acc_value_to_filt, i, j);
		
				}

				// Covariance Matrix update
				updateCovarianceMatrix(i, &vect_filt[i][0], &covariance_matrix[i]);


				// Components Relevance vector update
				updateRelevanceComponentsVector(i, &covariance_matrix[i], &relevance_comp_v[i], relevance_comp_v_weight);
		
			}

		}

		// 3D->1D Matrix Reduction
		for (int i = 0; i < N_IMU; i++){

			double pwm_acc = 0.0;

			// Actuators input computation
			for (int j = 0; j < N_AXES; j++){
				pwm_acc += vect_filt[i][j] * relevance_comp_v[i][j];
			}		

			input_act[i] = pwm_acc;
		}
/*
		for (int i = 0; i < N_IMU; i++){
			input_act[i] = sqrt(acc_values[i][0]*acc_values[i][0] + acc_values[i][1]*acc_values[i][1] + acc_values[i][2]*acc_values[i][2]);
		}
*/

		actValue = input_act[0];

		// Consider palm IMU as backup in case of index IMU disconnection
//		actValue = input_act[1];

		if (actValue == 0.0){
			// Exact 0.0 value means an IMU disconnection occurred
//			actValue = input_act[0];
			if (!first_disconnect){
				std::cout << "IMU on finger disconnected on " << ns.substr(1,ns.length()-1) << " hand" << endl;
				first_disconnect = true;
			}
		}
		

		// Publish message on topic (if enabled)
		if (pub_topic_enabled){
			//msg_act_fb.data.clear();
			
			msg_act_fb.data = actValue;
			
			/*for (int i = 0; i < N_IMU; i++){
				msg_act_fb.data.push_back(input_act[i]);
			}*/

			// Publish actuators inputs
			pub_act_fb.publish(msg_act_fb);
		}
	/****************** END VIBRO **************************************/
		

		ros::spinOnce();     // Need to call this function often to allow ROS to process incoming messages 
		loop_rate.sleep();   // Sleep for the rest of the cycle, to enforce the loop rate
	}

	return 0;

}


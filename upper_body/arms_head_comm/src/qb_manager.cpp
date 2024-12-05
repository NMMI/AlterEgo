#include "ros/ros.h"
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <vector>
#include <string>
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
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>
#include <qbmove_communications.h>
#include <cp_communications.h>
#include <definitions.h>
#include <fstream>
using namespace KDL;
using namespace std;
using namespace Eigen;

bool VERBOSE = false;
#define MAX_PRESET 0.6
#define MIN_PRESET 0
#define MAX_HAND_CL 16000

VectorXd ref_c_eq(20);
VectorXd ref_c_eq_prec(20); // Cube ref LP filter
VectorXd ref_c_pr(20);
VectorXd ref_eq(20);
VectorXd ref_pr(20);
double ref_hand(0);
bool ref_hand_changed = false;
double ref_neck(0);
int arm_cubes_n(7);
double offset_hand = 0.0;
std::vector<bool> activatedQB{false, false, false, false, false, false};
bool activatedNeck = false;
bool activatedHand = false;
bool rigid_shoulder = false;
// IMU Reading Global Variables
int n_imu_;
uint8_t *imu_table_;
uint8_t *mag_cal_;
std::vector<float> imu_values_;

// Analog sensors Global Variables
int n_adc_sensors_;
std::vector<short int> adc_raw_;
int AlterEgoVersion;

string ns;

/*---------------------------------------------------------------------*
 *                                                *
 *                                                                      *
 *----------------------------------------------------------------------*/
void ref_arm_eq__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
	for (int i = 0; i < arm_cubes_n; i++)
	{
		// ref_c_eq(i) = msg->data[i];
		ref_c_eq(i) = 0.05 * msg->data[i] + 0.95 * ref_c_eq_prec(i); // [IGT] Cube ref LP filter
		// ref_c_eq(i) = msg->data[i];									 //+0.07 * ref_c_eq_prec(i);									 // Cube ref LP filter
		ref_c_eq_prec(i) = ref_c_eq(i); // Cube ref LP filter
	}
	if (AlterEgoVersion == 3)
	{
		if (ns.find("left") != std::string::npos) // if v3
		{
			// std::cout << ns << std::endl;
			//  ref_c_eq(0) = 1.9; //54.5° -> 0.95rad flange offset w.r.t. cube zero position
			ref_c_eq(1) = (ref_c_eq(1) - 0.33) * 2.086; // 19° -> 0.33rad flange offset w.r.t. cube zero position
		}
		else
		{
			// std::cout << ns << std::endl;
			//  ref_c_eq(0) = -1.9;//54.5° -> 0.95rad flange offset w.r.t. cube zero position
			ref_c_eq(1) = (ref_c_eq(1) + 0.33) * 2.086; // 19° -> 0.33rad flange offset w.r.t. cube zero position
		}
	}
}

/*---------------------------------------------------------------------*
 *                                                *
 *                                                                      *
 *----------------------------------------------------------------------*/
void ref_arm_preset__Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
	if (AlterEgoVersion == 2)
	{
		for (int i = 0; i < arm_cubes_n; i++)
		{
			ref_c_pr(i) = msg->data[i];

			if (ref_c_pr(i) > MAX_PRESET)
				ref_c_pr(i) = MAX_PRESET;
			if (ref_c_pr(i) < 0)
				ref_c_pr(i) = 0;
		}
	}
	else if (AlterEgoVersion == 3)
	{

		ref_pr(0) = 0.0;

		for (int i = 1; i < arm_cubes_n; i++)
		{
			ref_c_pr(i) = msg->data[i];

			if (ref_c_pr(i) > 1)
				ref_c_pr(i) = 1;
			if (ref_c_pr(i) < 0)
				ref_c_pr(i) = 0;

			ref_c_pr(i) = ref_c_pr(i);
		}
	}
}

/*---------------------------------------------------------------------*
 *                                                *
 *                                                                      *
 *----------------------------------------------------------------------*/
void ref_hand__Callback(const std_msgs::Float64::ConstPtr &msg)
{
	double ref_hand_old = ref_hand;
	double a = log(1.7);
	double b = -0.7;

	// ref_hand = exp(a*msg->data) + b

	ref_hand = msg->data;
	if (ref_hand > 1.0)
		ref_hand = 1.0;
	ref_hand_changed = true;
}

/*---------------------------------------------------------------------*
 *                                                *
 *                                                                      *
 *----------------------------------------------------------------------*/
void ref_neck__Callback(const std_msgs::Float64::ConstPtr &msg)
{
	ref_neck = msg->data;
}

/*---------------------------------------------------------------------*
 *                                                *
 *                                                                      *
 *----------------------------------------------------------------------*/
double tick2rad(short int meas)
{
	return (double(meas)) * (2.0 * M_PI) / 32768.0;
}

/*---------------------------------------------------------------------*
 *                                                *
 *                                                                      *
 *----------------------------------------------------------------------*/
void compute_cube_motor_position(double eq, double preset, short int &th1, short int &th2)
{
	if (preset > MAX_PRESET)
		preset = MAX_PRESET;
	if (preset < MIN_PRESET)
		preset = MIN_PRESET;

	th1 = (short int)((eq + preset) * 32768.0 / (2 * M_PI));
	th2 = (short int)((eq - preset) * 32768.0 / (2 * M_PI));
}

int getADCValues(comm_settings *file_descriptor, const int &id, int n_adc_sensors, std::vector<short int> &adc_raw)
{
	return commGetADCRawValues(file_descriptor, id, n_adc_sensors, (short int *)&adc_raw[0]);
}

int getIMUValues(comm_settings *file_descriptor, const int &id, uint8_t *imu_table, uint8_t *imus_magcal, int n_imu, const bool &custom_read_timeout, std::vector<float> &imu_values)
{
	if (custom_read_timeout)
	{
		long r_timeout = 1250 * n_imu; // [usec], e.g. a 4-imus board with all sensors ON takes 4700 us avg. to be read
		return commGetImuReadings(file_descriptor, id, (uint8_t *)&imu_table[0], (uint8_t *)&imus_magcal[0], n_imu, (float *)&imu_values[0], r_timeout);
	}
	// Default read timeout (READ_TIMEOUT macro)
	return commGetImuReadings(file_descriptor, id, (uint8_t *)&imu_table[0], (uint8_t *)&imus_magcal[0], n_imu, (float *)&imu_values[0]);
}

int initializeIMU(comm_settings *comm_settings_b, int ID, bool isOldBoard)
{

	uint8_t aux_string[2000];
	uint8_t PARAM_SLOT_BYTES = 50;
	uint8_t num_imus_id_params = 6;
	uint8_t num_mag_cal_params = 0;
	uint8_t first_imu_parameter = 2;

	/**************** IMU parameters initialization section ***********************/
	if (isOldBoard)
	{
		commGetParamList(comm_settings_b, ID, 0, NULL, 0, 0, aux_string);
	}
	else
	{
		commGetIMUParamList(comm_settings_b, ID, 0, NULL, 0, 0, aux_string);
	}
	ros::Duration(0.05).sleep();

	n_imu_ = aux_string[8];

	if (isOldBoard == true)
	{
		// If the response variable 'old_board' is set to true, the connected board is a PSoC3 board instead of a STM32 or PSoC5 board
		// so update the number of id_params to the right value
		num_imus_id_params = 6;
	}
	else
	{
		num_imus_id_params = 7;
	}

	// aux_string[6] <-> packet_data[2] on the firmware
	cout << "Number of connected IMUs: " << n_imu_ << endl;

	if (n_imu_ <= 0)
	{
		ROS_INFO_STREAM_NAMED("device_hw", "[IMU] device [" << ID << "] has no IMU connected");
		return -1;
	}

	// Compute number of read parameters depending on global_args.n_imu and
	// update packet_length
	num_mag_cal_params = (n_imu_ / 2);
	if ((n_imu_ - num_mag_cal_params * 2) > 0)
		num_mag_cal_params++;

	ROS_INFO_STREAM_NAMED("device_hw", "[IMU] device [" << ID << "] parameters table:");

	/********************** MAG CAL SECTION (not needed) *********************************/
	/*
		// Retrieve magnetometer calibration parameters
		mag_cal_ = (uint8_t *) calloc(n_imu_, 3*sizeof(uint8_t));
		int v = 0;
		for (int k=1; k <= num_mag_cal_params; k++) {
		  mag_cal_[3*v + 0] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 8];
		  mag_cal_[3*v + 1] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 9];
		  mag_cal_[3*v + 2] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 10];
		  ROS_INFO_STREAM_NAMED("device_hw", "[IMU] MAG PARAM: " << (int)mag_cal_[3*v + 0] << " " << (int)mag_cal_[3*v + 1] << " " << (int)mag_cal_[3*v + 2]);
		  v++;

		  if (aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 7] == 6) {
			mag_cal_[3*v + 0] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 11];
			mag_cal_[3*v + 1] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 12];
			mag_cal_[3*v + 2] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 13];
			ROS_INFO_STREAM_NAMED("device_hw", "[IMU] MAG PARAM: " << (int)mag_cal_[3*v + 0] << " " << (int)mag_cal_[3*v + 1] << " " << (int)mag_cal_[3*v + 2]);
			v++;
		  }
		}
	*/

	first_imu_parameter = 1 + num_imus_id_params + num_mag_cal_params + 1;
	imu_table_ = (uint8_t *)calloc(n_imu_, 5 * sizeof(uint8_t));
	for (int i = 0; i < n_imu_; i++)
	{
		imu_table_[5 * i + 0] = aux_string[first_imu_parameter * PARAM_SLOT_BYTES + 8 + 50 * i];
		imu_table_[5 * i + 1] = aux_string[first_imu_parameter * PARAM_SLOT_BYTES + 9 + 50 * i];
		imu_table_[5 * i + 2] = aux_string[first_imu_parameter * PARAM_SLOT_BYTES + 10 + 50 * i];
		imu_table_[5 * i + 3] = aux_string[first_imu_parameter * PARAM_SLOT_BYTES + 11 + 50 * i];
		imu_table_[5 * i + 4] = aux_string[first_imu_parameter * PARAM_SLOT_BYTES + 12 + 50 * i];
		ROS_INFO_STREAM_NAMED("device_hw", "[IMU] ID: " << ID << " - sensors table: " << (int)imu_table_[5 * i + 0] << ", " << (int)imu_table_[5 * i + 1] << ", " << (int)imu_table_[5 * i + 2] << ", " << (int)imu_table_[5 * i + 3] << ", " << (int)imu_table_[5 * i + 4]);
	}
	/**************** End of IMU parameters initialization section ***********************/

	return 0;
}

int initializeADC(comm_settings *comm_settings_b, int ID)
{

	uint8_t adc_map[100];
	uint8_t tot_adc_channels = 0;

	n_adc_sensors_ = 0;

	commGetADCConf(comm_settings_b, ID, &tot_adc_channels, adc_map);
	usleep(100000);

	for (int i = 0; i < tot_adc_channels; i++)
	{
		if (adc_map[i] == 1)
		{
			n_adc_sensors_++;
		}
	}

	printf("Number of ADC channels: %d\n", n_adc_sensors_);

	if (n_adc_sensors_ <= 0)
	{
		ROS_INFO_STREAM_NAMED("device_hw", "[ADC] device [" << ID << "] has no ADC sensors connected");
		return -1;
	}

	return 0;
}

void readIMU(comm_settings *comm_settings_b, int ID, bool isOldBoard)
{

	bool custom_read_timeout_ = !isOldBoard; // If the connected board is a STM32 or PSoC5 board, set custom read timeout as default

	/**************** IMU reading section ***********************/

	getIMUValues(comm_settings_b, ID, imu_table_, mag_cal_, n_imu_, custom_read_timeout_, imu_values_);
}

/*---------------------------------------------------------------------*
 * MAIN                                                                 *
 *                                                                      *
 *----------------------------------------------------------------------*/
int main(int argc, char **argv)
{
	string port;
	string ref_arm_eq_topic;
	string ref_arm_preset_topic;
	string ref_hand_topic;
	string ref_neck_topic;
	string meas_arm_m1_topic;
	string meas_arm_m2_topic;
	string meas_arm_shaft_topic;
	string meas_neck_shaft_topic;
	string meas_hand_topic;
	string acc_hand_topic;
	string curr_m1_topic;
	string curr_m2_topic;
	string pressure_topic;
	bool hand_has_old_board = false;
	bool vibrotactile_fb_enabled = false;
	bool pressure_fb_enabled = false;

	double main_freq(1000);
	double body_freq;
	double curr_freq;
	double hand_move_freq;
	double vibro_fb_freq;
	double pressure_fb_freq;
	int loop_cnt = 0;

	int baudrate(2000000);
	int IDhand;
	int IDneck;
	int waiting_nsec(50);

	short int inputs[2];
	short int measurements[3];
	short int curr_meas[2];

	std::vector<double> IDsArm;

	comm_settings comm_settings_t;

	ros::Subscriber sub_ref_arm_eq;
	ros::Subscriber sub_ref_arm_preset;
	ros::Subscriber sub_ref_hand;
	ros::Subscriber sub_ref_neck;

	ros::Publisher pub_meas_cubes_m1;
	ros::Publisher pub_meas_cubes_m2;
	ros::Publisher pub_meas_cubes_shaft;
	ros::Publisher pub_meas_neck_shaft;
	ros::Publisher pub_meas_hand;
	ros::Publisher pub_acc_hand;
	ros::Publisher pub_pressure;
	ros::Publisher pub_stiffness_m1;
	ros::Publisher pub_stiffness_m2;
	// current
	ros::Publisher pub_curr_cubes_m1;
	ros::Publisher pub_curr_cubes_m2;

	std_msgs::Float64MultiArray msg_meas_cubes_m1;
	std_msgs::Float64MultiArray msg_meas_cubes_m2;
	std_msgs::Float64MultiArray msg_meas_cubes_shaft;
	std_msgs::Float64MultiArray msg_meas_cubes_m1_old;
	std_msgs::Float64MultiArray msg_meas_cubes_m2_old;
	std_msgs::Float64MultiArray msg_meas_cubes_shaft_old;
	std_msgs::Float64MultiArray msg_meas_stiffness_m1;
	std_msgs::Float64MultiArray msg_meas_stiffness_m2;
	std_msgs::Float64 msg_meas_neck_shaft;
	std_msgs::Float64 msg_meas_hand_cl;
	std_msgs::Float64MultiArray msg_imu_hand;
	std_msgs::Float64MultiArray msg_pressure;

	// current

	std_msgs::Float64MultiArray msg_curr_cubes_m1;
	std_msgs::Float64MultiArray msg_curr_cubes_m2;
	bool curr_update = true;

	ref_hand = offset_hand;

	// ------------------------------------------------------------------------------------- Init Node
	ros::init(argc, argv, "qb_manager");
	ros::NodeHandle n;
	ns = ros::this_node::getNamespace();
	std::string robot_name = std::getenv("ROBOT_NAME");

	n.getParam("port", port);
	n.getParam("baudrate", baudrate);
	n.getParam("IDsArm", IDsArm);
	n.getParam("IDhand", IDhand);
	n.getParam("IDneck", IDneck);
	n.getParam("/" + robot_name + "/arm_cubes_n", arm_cubes_n);
	n.getParam("ref_arm_eq_topic", ref_arm_eq_topic);
	n.getParam("ref_arm_preset_topic", ref_arm_preset_topic);
	n.getParam("ref_hand_topic", ref_hand_topic);
	n.getParam("ref_neck_topic", ref_neck_topic);
	n.getParam("meas_arm_m1_topic", meas_arm_m1_topic);
	n.getParam("meas_arm_m2_topic", meas_arm_m2_topic);
	n.getParam("cubes_curr_m1_topic", curr_m1_topic);
	n.getParam("cubes_curr_m2_topic", curr_m2_topic);
	n.getParam("meas_arm_shaft_topic", meas_arm_shaft_topic);
	n.getParam("meas_neck_shaft_topic", meas_neck_shaft_topic);
	n.getParam("meas_hand_topic", meas_hand_topic);
	n.getParam("acc_hand_topic", acc_hand_topic);
	n.getParam("pressure_topic", pressure_topic);
	n.getParam("hand_has_old_board", hand_has_old_board);
	n.getParam("/" + robot_name + "/vibrotactile_fb_enabled", vibrotactile_fb_enabled);
	n.getParam("/" + robot_name + "/pressure_fb_enabled", pressure_fb_enabled);
	n.getParam("/" + robot_name + "/rigid_shoulder", rigid_shoulder);
	n.getParam("/" + robot_name + "/AlterEgoVersion", AlterEgoVersion);

	if (VERBOSE)
		std::cout << "[" << ns << "] "<<"--------rigid shoulder " << rigid_shoulder << std::endl;
	if (VERBOSE)
		std::cout << "[" << ns << "] "<< "--------AlterEgoVersion " << AlterEgoVersion << std::endl;
	// Frequencies settings
	ros::Rate loop_rate(main_freq); // Run node at max. frequency [1000 Hz default]

	// All frequencies must be sub-multiples of main_freq
	curr_freq = 50;													// Default
	n.getParam("/" + robot_name + "/current_frequency", curr_freq); // Override if configured
	curr_freq = (curr_freq > main_freq) ? main_freq : curr_freq;	// Limit body frequency to main frequency

	body_freq = 50;												 // Default
	n.getParam("/" + robot_name + "/body_frequency", body_freq); // Override if configured
	body_freq = (body_freq > main_freq) ? main_freq : body_freq; // Limit body frequency to main frequency
	std::cout << "[" << ns << "] "<<"arm body frequency:\t" << body_freq << std::endl;
	hand_move_freq = body_freq;													// Default
	n.getParam("/" + robot_name + "/hand_move_frequency", hand_move_freq);		// Override if configured
	hand_move_freq = (hand_move_freq > body_freq) ? body_freq : hand_move_freq; // Limit hand frequency to body frequency

	vibro_fb_freq = main_freq;													// Default
	n.getParam("/" + robot_name + "/vibrotactile_fb_frequency", vibro_fb_freq); // Override if configured
	vibro_fb_freq = (vibro_fb_freq > main_freq) ? main_freq : vibro_fb_freq;	// Limit vibro_fb frequency to main frequency

	pressure_fb_freq = main_freq;													  // Default
	n.getParam("/" + robot_name + "/pressure_fb_frequency", pressure_fb_freq);		  // Override if configured
	pressure_fb_freq = (pressure_fb_freq > main_freq) ? main_freq : pressure_fb_freq; // Limit pressure_fb frequency to main frequency

	// --- node param ---

	// ------------------------------------------------------------------------------------- Subscribe to topics
	sub_ref_arm_eq = n.subscribe(ref_arm_eq_topic, 1, ref_arm_eq__Callback);
	sub_ref_arm_preset = n.subscribe(ref_arm_preset_topic, 1, ref_arm_preset__Callback);
	sub_ref_hand = n.subscribe(ref_hand_topic, 1, ref_hand__Callback);
	sub_ref_neck = n.subscribe("/" + robot_name + ref_neck_topic, 1, ref_neck__Callback);

	// ------------------------------------------------------------------------------------- Published topics
	pub_meas_cubes_m1 = n.advertise<std_msgs::Float64MultiArray>(meas_arm_m1_topic, 1);
	pub_meas_cubes_m2 = n.advertise<std_msgs::Float64MultiArray>(meas_arm_m2_topic, 1);
	pub_meas_cubes_shaft = n.advertise<std_msgs::Float64MultiArray>(meas_arm_shaft_topic, 1);
	pub_meas_neck_shaft = n.advertise<std_msgs::Float64>(meas_neck_shaft_topic, 1);
	pub_meas_hand = n.advertise<std_msgs::Float64>(meas_hand_topic, 1);
	pub_stiffness_m1 = n.advertise<std_msgs::Float64MultiArray>("meas_stiffness_m1", 1);
	pub_stiffness_m2 = n.advertise<std_msgs::Float64MultiArray>("meas_stiffness_m2", 1);

	// current
	pub_curr_cubes_m1 = n.advertise<std_msgs::Float64MultiArray>(curr_m1_topic, 1);
	pub_curr_cubes_m2 = n.advertise<std_msgs::Float64MultiArray>(curr_m2_topic, 1);

	// ------------------------------------------------------------------------------------- Init Comm
	cout << "[" << ns << "]" << " Opening communication on port: " << port << endl;
	openRS485(&comm_settings_t, port.c_str());
	usleep(1000 * waiting_nsec);
	int active = -1000;
	int qbcube_count = 0;
	char pack_in;
	std::ofstream system_check_file;
    std::string path;
	n.getParam("/"+robot_name+"/SystemCheckPath", path);
	std::string system_check = path;
	system_check_file.open(system_check, std::ios_base::app);

	// system_check_file << "----[QBs Activation]-- \n";
	system_check_file << "[" << ns << "]"  << " ------- QB ACTIVATION  " << endl;

	for (int i = 0; i < arm_cubes_n; i++)
	{
		ref_c_eq(i) = 0;
		ref_c_pr(i) = 0;
		cout << "[" << ns << "]" << " Activating cube: " << IDsArm[i] << endl;


		while (qbcube_count <= 4)
		{
			commActivate(&comm_settings_t, IDsArm[i], 1);

			active = commGetActivate(&comm_settings_t, IDsArm[i], &pack_in);
			if (active == 0)
			{
				qbcube_count = 5;
			}
			else
			{
				commActivate(&comm_settings_t, IDsArm[i], 0);

				qbcube_count += 1;
			}

			usleep(100 * waiting_nsec);

		}

		if (active == 0)
			system_check_file << "[" << ns << "]"
								<< " cube:" << IDsArm[i] << " activated. " << endl;
		else
			system_check_file << "[" << ns << "]"
							<< " cube:" << IDsArm[i] << " is NOT active. " << endl;

		qbcube_count = 0;
		usleep(10000 * waiting_nsec);
	}
	cout << "[" << ns << "]" << " Activating neck: " << IDneck << endl;
	commActivate(&comm_settings_t, IDneck, 1);
	usleep(10000 * waiting_nsec);
	cout << "[" << ns << "]" << " Activating hand: " << IDhand << endl;
	commActivate(&comm_settings_t, IDhand, 1);
	usleep(10000 * waiting_nsec);

	if (AlterEgoVersion == 3)
	{
		if (ns.find("left") != std::string::npos)
		{
			// ref_c_eq(0) = 1.9; //54.5° -> 0.95rad flange offset w.r.t. cube zero position
			ref_c_eq(1) = -0.33 * 2.086; // 19° -> 0.33rad flange offset w.r.t. cube zero position
		}
		else
		{
			// ref_c_eq(0) = -1.9;//54.5° -> 0.95rad flange offset w.r.t. cube zero position
			ref_c_eq(1) = 0.33 * 2.086; // 19° -> 0.33rad flange offset w.r.t. cube zero position
		}
		if (VERBOSE)
			std::cout << "[" << ns << "] "<< "--------AlterEgoVersion " << AlterEgoVersion << std::endl;
	}
	ref_c_eq_prec = ref_c_eq;



	int hand_count = 0;
	// Tentativi di attivazione della mano
	while (hand_count <= 4)
	{
		active = commGetActivate(&comm_settings_t, IDhand, &pack_in);
		if (active == 0)
		{
			activatedHand = true;
			hand_count = 5;
		}
		else
		{
			activatedHand = false;
			hand_count += 1;
		}

		usleep(1000 * waiting_nsec);
	}
	if (activatedHand)
		system_check_file << "[" << ns << "]"
						  << " Hand " << IDhand << " is activated. " << endl;
	else
		system_check_file << "[" << ns << "]"
						  << " Hand " << IDhand << " is NOT activated. " << endl;

	// Se la mano si attiva provo ad attivare le imu per usare l'aptica
	if (vibrotactile_fb_enabled && activatedHand)
	{

		pub_acc_hand = n.advertise<std_msgs::Float64MultiArray>(acc_hand_topic, 1);

		if (!initializeIMU(&comm_settings_t, IDhand, hand_has_old_board))
		{
			// Read imu table (cast from std::vector to uint8 array)
			imu_values_.resize(n_imu_ * (3 * 3 + 4 + 1));

			usleep(10000 * waiting_nsec);
			cout << "[" << ns << "]"
				 << " Vibrotactile feedback initialized on hand: " << IDhand << endl;
		}
		else
		{
			cout << "[" << ns << "]"
				 << " Error in vibrotactile feedback initialization on hand: " << IDhand << endl;
		}
	}

	// Se la mano si attiva provo ad attivare i sensori di pressione
	if (pressure_fb_enabled && activatedHand)
	{

		pub_pressure = n.advertise<std_msgs::Float64MultiArray>(pressure_topic, 1);

		if (!initializeADC(&comm_settings_t, IDhand))
		{

			adc_raw_.resize(n_adc_sensors_);

			usleep(10000 * waiting_nsec);
			cout << "[" << ns << "]"
				 << " Pressure feedback initialized on hand: " << IDhand << endl;
		}
		else
		{
			cout << "[" << ns << "]"
				 << " Error in pressure feedback initialization on hand: " << IDhand << endl;
		}
	}

	for (int i = 0; i < arm_cubes_n; i++)
	{
		if (commGetMeasurements(&comm_settings_t, IDsArm[i], measurements) > 0)
		{
			while (ros::ok() && (isnan(measurements[0]) || isnan(measurements[1]) || isnan(measurements[2])))
			{
				if (commGetMeasurements(&comm_settings_t, IDsArm[i], measurements) > 0)
					std::cout << "Ho beccato Nan" << std::endl;
			}
			msg_meas_cubes_m1.data.push_back(tick2rad(measurements[0]));
			msg_meas_cubes_m2.data.push_back(tick2rad(measurements[1]));
			msg_meas_cubes_shaft.data.push_back(tick2rad(measurements[2]));
			msg_meas_stiffness_m1.data.push_back(tick2rad(measurements[0] - measurements[2]));
			msg_meas_stiffness_m2.data.push_back(tick2rad(measurements[2] - measurements[1]));
		}

		// std::cout << "Index " << i << std::endl;
	}

	msg_meas_cubes_m1_old = msg_meas_cubes_m1;
	msg_meas_cubes_m2_old = msg_meas_cubes_m2;
	msg_meas_cubes_shaft_old = msg_meas_cubes_shaft;

	// ------ Verify if QBs have been correctly activated


	active = commGetActivate(&comm_settings_t, IDneck, &pack_in);
	if (active == 0)
	{
		system_check_file << "[" << ns << "]"
						  << " Neck:" << IDneck << " activated." << endl;
	}
	else
	{
		system_check_file << "[" << ns << "]"
						  << " Neck:" << IDneck << " is NOT active." << endl;
	}

	// system_check_file << "----[END QBs Activation]-- \n\n";
	system_check_file.close();
	msg_meas_neck_shaft.data = 0;					//Init zero value for neck cubes
	// ------------------------------------------------------------------------------------- MAIN LOOP
	while (ros::ok())
	{
		// BODY

		// curr_update=(loop_cnt % (int)(main_freq/curr_freq) == 0 );
		if (loop_cnt % (int)(main_freq / body_freq) == 0)
		{ // This part is executed at body_freq frequency

			// --- Get Measurements ---
			msg_meas_cubes_m1.data.clear();
			msg_meas_cubes_m2.data.clear();
			msg_meas_cubes_shaft.data.clear();
			// current
			msg_curr_cubes_m1.data.clear();
			msg_curr_cubes_m2.data.clear();
			msg_meas_stiffness_m1.data.clear();
			msg_meas_stiffness_m2.data.clear();

			// Compongo i messaggi m1 m2 e shaft con le misure dei cubi se non sono vuote
			for (int i = 0; i < arm_cubes_n; i++)
			{
				if (commGetMeasurements(&comm_settings_t, IDsArm[i], measurements) > 0)
				{
					if (!isnan(measurements[0]))
						msg_meas_cubes_m1.data.push_back(tick2rad(measurements[0]));
					else
					{
						msg_meas_cubes_m1.data.push_back(msg_meas_cubes_m1_old.data[i]);
						std::cout << "[" << ns << "] "<<"Ho beccato Nan M1" << std::endl;
					}

					if (!isnan(measurements[1]))
						msg_meas_cubes_m2.data.push_back(tick2rad(measurements[1]));
					else
					{
						msg_meas_cubes_m2.data.push_back(msg_meas_cubes_m2_old.data[i]);
						std::cout << "Ho beccato Nan M2" << std::endl;
					}

					if (!isnan(measurements[2]))
						msg_meas_cubes_shaft.data.push_back(tick2rad(measurements[2]));
					else
					{
						msg_meas_cubes_shaft.data.push_back(msg_meas_cubes_shaft_old.data[i]);
						std::cout << "Ho beccato Nan Shaft" << std::endl;
					}
				}
				else
				{
					msg_meas_cubes_m1.data.push_back(msg_meas_cubes_m1_old.data[i]);
					msg_meas_cubes_m2.data.push_back(msg_meas_cubes_m2_old.data[i]);
					msg_meas_cubes_shaft.data.push_back(msg_meas_cubes_shaft_old.data[i]);
				}

				msg_meas_stiffness_m1.data.push_back(tick2rad(msg_meas_cubes_m1.data[i] - msg_meas_cubes_shaft.data[i]));
				msg_meas_stiffness_m2.data.push_back(tick2rad(msg_meas_cubes_shaft.data[i] - msg_meas_cubes_m2.data[i]));
			}

			// --- Clear old Measurements ---
			msg_meas_cubes_m1_old.data.clear();
			msg_meas_cubes_m2_old.data.clear();
			msg_meas_cubes_shaft_old.data.clear();

			msg_meas_cubes_m1_old = msg_meas_cubes_m1;
			msg_meas_cubes_m2_old = msg_meas_cubes_m2;
			msg_meas_cubes_shaft_old = msg_meas_cubes_shaft;

			if (AlterEgoVersion == 3)
			{
				msg_meas_cubes_m1.data[1] = msg_meas_cubes_m1.data[1] / 2.086;
				msg_meas_cubes_m2.data[1] = msg_meas_cubes_m2.data[1] / 2.086;
				msg_meas_cubes_shaft.data[1] = msg_meas_cubes_shaft.data[1] / 2.086;

				// msg_meas_stiffness_m1.data[1] = (tick2rad(msg_meas_cubes_m1.data[1] - msg_meas_cubes_shaft.data[1]));
				// msg_meas_stiffness_m2.data[1] = (tick2rad(msg_meas_cubes_shaft.data[1] - msg_meas_cubes_m2.data[1]));
			}
			// Inserisco le misure nei topic
			pub_meas_cubes_m1.publish(msg_meas_cubes_m1);
			pub_meas_cubes_m2.publish(msg_meas_cubes_m2);
			pub_meas_cubes_shaft.publish(msg_meas_cubes_shaft);
			pub_stiffness_m1.publish(msg_meas_stiffness_m1);
			pub_stiffness_m2.publish(msg_meas_stiffness_m2);

			if (commGetMeasurements(&comm_settings_t, IDneck, measurements) > 0)
			{
				msg_meas_neck_shaft.data = tick2rad(measurements[2]);
			}
			// else
			// {
			// 	msg_meas_neck_shaft.data = 0;
			// }
			pub_meas_neck_shaft.publish(msg_meas_neck_shaft);

			// -- Set References ---
			ref_eq = ref_c_eq; // Copy ref values from last callback (so that arm SetInputs is atomic and coherent)
			ref_pr = ref_c_pr;

			if (rigid_shoulder)
			{
				// -- Set input for rigid shoulder ---
				inputs[0] = (short int)((ref_eq(0)) * 32768.0 / (2 * M_PI));
				inputs[1] = 0;
				commSetInputs(&comm_settings_t, IDsArm[0], inputs);
			}
			else
			{
				// -- Set input for normal shoulder ---
				compute_cube_motor_position(ref_eq(0), ref_pr(0), inputs[0], inputs[1]);
				commSetInputs(&comm_settings_t, IDsArm[0], inputs);
			}
			usleep(waiting_nsec);

			// -- Set input for the the arm ---
			for (int i = 1; i < arm_cubes_n; i++)
			{
				compute_cube_motor_position(ref_eq(i), ref_pr(i), inputs[0], inputs[1]);
				commSetInputs(&comm_settings_t, IDsArm[i], inputs);
				usleep(waiting_nsec);
			}
			compute_cube_motor_position(ref_neck, 0.4, inputs[0], inputs[1]);
			commSetInputs(&comm_settings_t, IDneck, inputs);
			usleep(waiting_nsec);
		}

		// HAND
		if (activatedHand)
		{
			if (loop_cnt % (int)(main_freq / hand_move_freq) == 0)
			{ // This part is executed at hand_move_freq frequency
				// Allow handling of new SoftHand Pro hands with active IMU reading

				if (commGetMeasurements(&comm_settings_t, IDhand, measurements) > 0)
				{
					msg_meas_hand_cl.data = tick2rad(measurements[0]);
				}
				else
				{
					msg_meas_hand_cl.data = 0;
				}
				pub_meas_hand.publish(msg_meas_hand_cl);

				if (ref_hand_changed)
				{
					inputs[0] = ref_hand * MAX_HAND_CL;
					inputs[1] = 0;

					// Small break to allow multiple consecutive COM readings
					ros::Duration(0.00025).sleep(); // 250 us sleep

					commSetInputs(&comm_settings_t, IDhand, inputs);
					// cout << "[" << ns << "] " << "hand: " << IDhand << "  r1= " << inputs[0] << "  r2= " << inputs[1] << endl;

					ref_hand_changed = false;
				}
			}

			// VIBROTACTILE FEEDBACK
			if (vibrotactile_fb_enabled && n_imu_ > 0 && loop_cnt % (int)(main_freq / vibro_fb_freq) == 0)
			{ // This part is executed at vibro_fb_freq frequency
				// Allow handling of new SoftHand Pro hands with active IMU reading

				// --- Get Measurements ---
				msg_imu_hand.data.clear();

				readIMU(&comm_settings_t, IDhand, hand_has_old_board);

				for (int i = 0; i < n_imu_; i++)
				{

					// std::cout << "ACC: " << imu_values_.at((3*3+4+1)*i + 0) << " " << imu_values_.at((3*3+4+1)*i + 1) << " " << imu_values_.at((3*3+4+1)*i + 2) << std::endl;

					// Acceleration readings from the imu_read vector
					if (imu_table_[5 * i + 0])
					{

						if (n_imu_ > 1 && i != n_imu_ - 1)
						{
							continue;
						}
						else
						{
							// n_imu_ == 1 || i == n_imu_ - 1

							if (fabs(imu_values_.at((3 * 3 + 4 + 1) * i + 0)) < 1e-4 && fabs(imu_values_.at((3 * 3 + 4 + 1) * i + 1)) < 1e-4 && fabs(imu_values_.at((3 * 3 + 4 + 1) * i + 2)) < 1e-4)
							{
								// No signal on IMU, so take first one on SH logic board
								msg_imu_hand.data.push_back(imu_values_.at(0));
								msg_imu_hand.data.push_back(imu_values_.at(1));
								msg_imu_hand.data.push_back(imu_values_.at(2));
							}
							else
							{
								msg_imu_hand.data.push_back(imu_values_.at((3 * 3 + 4 + 1) * i + 0));
								msg_imu_hand.data.push_back(imu_values_.at((3 * 3 + 4 + 1) * i + 1));
								msg_imu_hand.data.push_back(imu_values_.at((3 * 3 + 4 + 1) * i + 2));
							}
						}
					}
				}

				pub_acc_hand.publish(msg_imu_hand);
			}

			// PRESSURE FEEDBACK
			if (pressure_fb_enabled && n_adc_sensors_ > 0 && loop_cnt % (int)(main_freq / pressure_fb_freq) == 0)
			{ // This part is executed at pressure_fb_freq frequency

				// --- Get Measurements ---
				msg_pressure.data.clear();

				if (getADCValues(&comm_settings_t, IDhand, n_adc_sensors_, adc_raw_) >= 0)
				{

					for (int i = 0; i < n_adc_sensors_; i++)
					{
						msg_pressure.data.push_back(adc_raw_.at(i) / 4096.0 * 100); //[4096 ticks -> 100 %]
					}
				}
				else
				{

					for (int i = 0; i < n_adc_sensors_; i++)
					{
						msg_pressure.data.push_back(0);
					}
				}
				pub_pressure.publish(msg_pressure);
			}
		}

		// --- cycle ---
		ros::spinOnce();
		loop_rate.sleep();

		// Update loop counter
		loop_cnt++;
		if (loop_cnt > main_freq)
		{
			loop_cnt -= main_freq;
		}
	}
	// desired motion : before deactivation hands, wrists and elbows should reach 0 position
	inputs[0] = 0 * MAX_HAND_CL;
	inputs[1] = 0;
	// Small break to allow multiple consecutive COM readings
	ros::Duration(0.00025).sleep(); // 250 us sleep

	if (activatedHand)
		commSetInputs(&comm_settings_t, IDhand, inputs);

	compute_cube_motor_position(0, 0, inputs[0], inputs[1]);
	// commSetInputs(&comm_settings_t, IDsArm[3], inputs);
	// usleep(10000*waiting_nsec);
	// commSetInputs(&comm_settings_t, IDsArm[5], inputs);
	// usleep(10000*waiting_nsec);
	// commSetInputs(&comm_settings_t, IDsArm[6], inputs);
	// cout << "cube: " << IDsArm[i] << "  r1= " << inputs[0] << "  r2= " << inputs[1] << endl;
	usleep(waiting_nsec);

	// cout << "[" << ns << "] " << "hand: " << IDhand << "  r1= " << inputs[0] << "  r2= " << inputs[1] << endl;

	for (int i = 0; i < arm_cubes_n; i++)
	{
		cout << "[" << ns << "]"
			 << " Deactivating cube: " << IDsArm[i] << endl;
		commActivate(&comm_settings_t, IDsArm[i], 0);
		usleep(10000 * waiting_nsec);
	}
	cout << "[" << ns << "]"
		 << " Deactivating neck: " << IDneck << endl;
	commActivate(&comm_settings_t, IDneck, 0);
	usleep(10000 * waiting_nsec);
	cout << "[" << ns << "]"
		 << " Deactivating hand: " << IDhand << endl;
	commActivate(&comm_settings_t, IDhand, 0);
	usleep(10000 * waiting_nsec);

	cout << "[" << ns << "]"
		 << " Closing communication on port: " << port << endl;
	closeRS485(&comm_settings_t);
}
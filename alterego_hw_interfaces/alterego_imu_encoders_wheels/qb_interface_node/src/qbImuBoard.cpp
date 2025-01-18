#include "qbImuBoard.h"
#include <fstream>
using std::cout;
using std::cerr;
using std::cin;
using std::endl;

//-----------------------------------------------------
//                                           qbImuBoard
//-----------------------------------------------------

/*
/ *****************************************************
/ Costructor of qbImuBoard class
/ *****************************************************
/   argument:
/       - id, ID of the cube
/   return:
/
*/

qbImuBoard::qbImuBoard(const int id) : qbInterface(id) {}

//-----------------------------------------------------
//                                           qbImuBoard
//-----------------------------------------------------

/*
/ *****************************************************
/ External costructor of qbImuBoard class
/ *****************************************************
/   argument:
/       - cs, serial communication pointer to the cube
/       - id, ID of the cube
/   return:
/
*/

qbImuBoard::qbImuBoard(comm_settings* cs, const int id, bool isOldBoard) : qbInterface(cs, id) {

	is_old_board_ = isOldBoard;
	
	initImuBoard();
	
}


//-----------------------------------------------------
//                                          ~qbImuBoard
//-----------------------------------------------------

/*
/ *****************************************************
/ Distructor of qbImuBoard class
/ *****************************************************
/
/   arguments:
/   return:
/
*/

qbImuBoard::~qbImuBoard() {}


//========================== OTHER FUNCTIONS ===================================


//-----------------------------------------------------
//                                         initImuBoard
//-----------------------------------------------------
void qbImuBoard::initImuBoard() {

	
	//Apro il file per il LOG scrivo l'errore e lo richiudo
	std::ofstream system_check_file;

	std::string system_check = getenv("SYSTEMCHECK_PATH");
	system_check_file.open (system_check, std::ios_base::app);
	uint8_t aux_string[2000];
	uint8_t PARAM_SLOT_BYTES = 50;
	uint8_t num_imus_id_params = 6;
	uint8_t num_mag_cal_params = 0;
	uint8_t first_imu_parameter = 2;
	int v = 0;
	
/**************** IMU parameters initialization section ***********************/
	if (is_old_board_){
		commGetParamList(cube_comm_, id_, 0, NULL, 0, 0, aux_string);
	}
	else {
		commGetIMUParamList(cube_comm_, id_, 0, NULL, 0, 0, aux_string);	
	}
	usleep(500);

	n_imu_ = aux_string[8];

    if (is_old_board_ == true){
      // If the response variable 'old_board' is set to true, the connected board is a PSoC3 board instead of a STM32 or PSoC5 board
      // so update the number of id_params to the right value
      num_imus_id_params = 6;
    } 
    else {
      num_imus_id_params = 7;
    }

    custom_read_timeout_ = !is_old_board_;  // If the connected board is a STM32 or PSoC5 board, set custom read timeout as default

    //aux_string[6] <-> packet_data[2] on the firmware
    cout << "Number of connected IMUs: " << n_imu_ << endl;
 		system_check_file << "[IMU] Number of connected IMUs: " << n_imu_ << endl;

    if (n_imu_ <= 0) {
      printf("[IMU] device %d has no IMU connected\n", id_);
      system_check_file<<"[IMU] device "<<(int)id_<<" has no IMU connected\n"<<std::endl;
      return;
    }
  
    // Compute number of read parameters depending on global_args.n_imu and
    // update packet_length
    num_mag_cal_params = (n_imu_ / 2);
    if ( (n_imu_ - num_mag_cal_params*2) > 0 ) num_mag_cal_params++;

	printf("[IMU] device %d parameters table:\n", (int)id_);
	system_check_file<<"[IMU] device "<<(int)id_<< "parameters table:\n"<<std::endl;


	
	ids_ = (uint8_t *) calloc(n_imu_, sizeof(uint8_t));
	v = 0;
	for (int k = 1; k <= num_imus_id_params; k++){
		if (aux_string[k*PARAM_SLOT_BYTES + 8] != 255) {
			ids_[v] = aux_string[k*PARAM_SLOT_BYTES + 8];
			v++;
		}
		if (aux_string[k*PARAM_SLOT_BYTES + 9] != 255) {
			ids_[v] = aux_string[k*PARAM_SLOT_BYTES + 9];
			v++;
		}
		if (aux_string[k*PARAM_SLOT_BYTES + 10] != 255) {
			ids_[v] = aux_string[k*PARAM_SLOT_BYTES + 10];
			v++;
		}
	}

    /********************** MAG CAL SECTION (not needed) *********************************/

    // Retrieve magnetometer calibration parameters
    mag_cal_ = (uint8_t *) calloc(n_imu_, 3*sizeof(uint8_t));
    v = 0;
    for (int k=1; k <= num_mag_cal_params; k++) {
      mag_cal_[3*v + 0] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 8];
      mag_cal_[3*v + 1] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 9];
      mag_cal_[3*v + 2] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 10];
      printf("[IMU] MAG PARAM: %d %d %d\n", (int)mag_cal_[3*v + 0], (int)mag_cal_[3*v + 1], (int)mag_cal_[3*v + 2]);
      system_check_file <<"[IMU] MAG PARAM: "<<(int)mag_cal_[3*v + 0]<<" "<<(int)mag_cal_[3*v + 1]<<" "<<(int)mag_cal_[3*v + 2]<<endl;
      v++;
      
      if (aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 7] == 6) {
        mag_cal_[3*v + 0] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 11];
        mag_cal_[3*v + 1] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 12];
        mag_cal_[3*v + 2] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 13];
        printf("[IMU] MAG PARAM: %d %d %d\n", (int)mag_cal_[3*v + 0], (int)mag_cal_[3*v + 1], (int)mag_cal_[3*v + 2]);
        system_check_file<<"[IMU] MAG PARAM: "<< (int)mag_cal_[3*v + 0]<<" "<<(int)mag_cal_[3*v + 1]<<" "<<(int)mag_cal_[3*v + 2]<<endl;
        v++;
      }
    }

  
    first_imu_parameter = 1 + num_imus_id_params + num_mag_cal_params + 1;
    imu_table_ = (uint8_t *) calloc(n_imu_, 5*sizeof(uint8_t));
    for (int i=0; i< n_imu_; i++){
      imu_table_[5*i + 0] = aux_string[first_imu_parameter*PARAM_SLOT_BYTES + 8 + 50*i];
      imu_table_[5*i + 1] = aux_string[first_imu_parameter*PARAM_SLOT_BYTES+ 9 + 50*i];
      imu_table_[5*i + 2] = aux_string[first_imu_parameter*PARAM_SLOT_BYTES + 10 + 50*i];
      imu_table_[5*i + 3] = aux_string[first_imu_parameter*PARAM_SLOT_BYTES + 11 + 50*i];
      imu_table_[5*i + 4] = aux_string[first_imu_parameter*PARAM_SLOT_BYTES + 12 + 50*i];
      printf("ID: %d  - %d, %d, %d, %d, %d\n", ids_[i], imu_table_[5*i + 0], imu_table_[5*i + 1], imu_table_[5*i + 2], imu_table_[5*i + 3], imu_table_[5*i + 4]);		
  	  system_check_file<<"[IMU] ID:"<<" "<<(int)ids_[i]<< " "<<(int)imu_table_[5*i + 0]<< " "<<(int)imu_table_[5*i + 1]<< " "<<(int)imu_table_[5*i + 2]<< " "<<(int)imu_table_[5*i + 3]<< " "<<(int)imu_table_[5*i + 4]<<endl;
  
    }
    
    system_check_file.close();

	
	// Imu values is a 3 sensors x 3 axes x n_imu_ values
	imu_values_ = (float *) calloc(n_imu_, 3*3*sizeof(float)+4*sizeof(float)+sizeof(float));
		
}


int getIMUValues(comm_settings *file_descriptor, const int &id, uint8_t* imu_table, uint8_t* imus_magcal, int n_imu, const bool &custom_read_timeout, float* imu_values) {
	if (custom_read_timeout){
		long r_timeout = 2000*n_imu; //[AlterEgo] it previously was 1250*n_imu;    // [usec], e.g. a 4-imus board with all sensors ON takes 4700 us avg. to be read
  		return commGetImuReadings(file_descriptor, id, (uint8_t*)&imu_table[0], (uint8_t*)&imus_magcal[0], n_imu, (float*)&imu_values[0], r_timeout);
	}
	// Default read timeout (READ_TIMEOUT macro)
	return commGetImuReadings(file_descriptor, id, (uint8_t*)&imu_table[0], (uint8_t*)&imus_magcal[0], n_imu, (float*)&imu_values[0]);
}

//-----------------------------------------------------
//                                  	 getImuReadings
//-----------------------------------------------------
int qbImuBoard::getImuReadings() {
	
	return getIMUValues(cube_comm_, id_, imu_table_, mag_cal_, n_imu_, custom_read_timeout_, imu_values_);
		

	 // for (int i = 0; i < n_imu_; i++) {
		
	 // 	printf("IMU: %d\n", ids_[i]);
	
	 // 	if (imu_table_[5*i + 0]){
	 // 		printf("Accelerometer\n");
	 // 		printf("%f, %f, %f\n", imu_values_[3*3*i], imu_values_[3*3*i+1], imu_values_[3*3*i+2]);
	 // 	}
	 // 	if (imu_table_[5*i + 1]){
	 // 		printf("Gyroscope\n");
		// 	printf("%f, %f, %f\n", imu_values_[3*3*i+3], imu_values_[3*3*i+4], imu_values_[3*3*i+5]);
		// }
		// if (imu_table_[5*i + 2] ){
		// 	printf("Magnetometer\n");
		// 	printf("%f, %f, %f\n", imu_values_[3*3*i+6], imu_values_[3*3*i+7], imu_values_[3*3*i+8]);
	 // 	}
		
		// printf("\n");
	 // }
	
}

/* END OF FILE */

#include <ros/ros.h>
#include <ros/rate.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <eigen3/Eigen/Eigen>
// ROS cuustom msg
#include <qb_interface/inertialSensor.h>
#include <qb_interface/inertialSensorArray.h>

#include <qb_interface/cubeRef.h>
#include <qb_interface/cubePos.h>
#include <string>
#include <fstream>
#include <array>
#include <memory>

bool systemGood = true;
ros::Time accReadTime, gyroReadTime, encReadTime;
bool encRead = false;
double battery_level = 0.0;

std::string hostname_vision;
std::string IP_vision;
//tutte le acc provenienti dalle imu (NON MYO)
void callback_imu_acc(const qb_interface::inertialSensorArray::ConstPtr& msg)
{
  if(!isnan(msg->m[0].x) && !isnan(msg->m[0].y) && !isnan(msg->m[0].z) && !isnan(msg->m[1].x) && !isnan(msg->m[1].y) && !isnan(msg->m[1].z))
  {
    accReadTime = ros::Time::now();
  }
  


}

void callback_imu_gyro(const qb_interface::inertialSensorArray::ConstPtr& msg)
{
  if(!isnan(msg->m[0].x) && !isnan(msg->m[0].y) && !isnan(msg->m[0].z) && !isnan(msg->m[1].x) && !isnan(msg->m[1].y) && !isnan(msg->m[1].z))
  {
    gyroReadTime = ros::Time::now();
  }
    

}

void callback_meas(const qb_interface::cubePos::ConstPtr& msg)
{
  if(!isnan(msg->p_1[0]) && !isnan(msg->p_2[0]))
  {
    encRead = true;
    encReadTime = ros::Time::now(); 
    battery_level = msg->p_L[0];
  }
}

// Funzione per eseguire un comando e ottenere l'output
std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}


void getVisionNameIp(){

  //--------------HOSTNAME VISION----------------
  char hostname[1024];
  gethostname(hostname, 1024);
  std::string hostname_str(hostname);
  // Trova la prima parola dell'hostname (prima del trattino "-")
  hostname_vision = hostname_str.substr(0, hostname_str.find('-'))+"-vision";



  //--------------IP VISION----------------
  // Esegui il comando hostname -I e ottieni l'output
  std::string output = exec("hostname -I");
  // Trova il primo spazio
  std::size_t spacePos = output.find(' ');    
  // Estrarre il primo indirizzo IP (prima dell'ultimo numero prima dello spazio ed aggiungo 1)
  IP_vision = output.substr(0, spacePos-1)+"1";
}

//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
	std::string robot_name = std::getenv("ROBOT_NAME");

  double rateHZ = 800;
  std::string fault_sns = "";
  ros::Duration imu_max_latency(0.3);  //300 msec.
  ros::Duration enc_max_latency(1.0);  //500 msec.
  
  ros::init(argc, argv, "System_Check");
  ros::NodeHandle nh;
  nh.getParam("system_check_frequency", rateHZ);

  accReadTime = ros::Time::now();
  gyroReadTime = ros::Time::now();
  encReadTime = ros::Time::now();

  ros::Publisher pub_comm = nh.advertise<qb_interface::cubeRef>("/"+robot_name+"/wheels/qb_interface_node/qb_class/cube_ref", 1);  //base motor ref (duty cycle [-100,100])
  
  //Topic you want to subscribe
  ros::Subscriber sub_imu_acc = nh.subscribe("/"+robot_name+"/imu/qb_class_imu/acc", 1, callback_imu_acc);
  ros::Subscriber sub_imu_gyro = nh.subscribe("/"+robot_name+"/imu/qb_class_imu/gyro", 1, callback_imu_gyro);    
  ros::Subscriber sub_enc = nh.subscribe("/"+robot_name+"/wheels/qb_interface_node/qb_class/cube_measurement", 1, callback_meas);
  
  ros::Rate r(rateHZ);
  int cnt = 0;
  std::cout << "[INFO] Start to spin system_check_node" << std::endl;
  ros::spinOnce();
  std::string kill_lqr = "rosnode kill /"+robot_name+"/wheels/lqr";
  bool send = false;
  static ros::Time lastBatteryCheckTime = ros::Time::now();
  static int battery_low_count = 0;
  std::ofstream system_check_file;
  std::string path;
  nh.getParam("/"+robot_name+"/SystemCheckPath", path);
  system_check_file.open (path, std::ios_base::app);


  getVisionNameIp();

  while(ros::ok())
  {


    // IMUs check
    if (ros::Time::now()-accReadTime > imu_max_latency || ros::Time::now()-gyroReadTime > imu_max_latency){

      fault_sns = "IMUs";

      systemGood = false;

      // Kill LQR node if exists
      std::vector<std::string> nodes;
      std::string name;
      if(ros::master::getNodes(nodes)){  
        for (unsigned int i = 0; i < nodes.size(); i++){
          name = nodes.at(i);
          if (name == "/"+robot_name+"/wheels/lqr"){
            int res = system(kill_lqr.c_str());              //stop
          }
        }
      }

      // Give commands to stop wheels
      double com_R = 0.0;
      double com_L = 0.0;
      qb_interface::cubeRef comm_pub;

      comm_pub.p_1.push_back(-com_R);
      comm_pub.p_2.push_back(com_L);
      pub_comm.publish(comm_pub);
    }

    //Check battery voltage
    if(encRead)
    {

      //after 2 minutes checks the battery_level
      if (ros::Time::now() - lastBatteryCheckTime >= ros::Duration(10.0)) {
        lastBatteryCheckTime = ros::Time::now();
        // Add your code here to check the battery level and execute the if condition
        if (battery_level < 21800.0) {
          battery_low_count++;
          if (battery_low_count >= 5) {
              std::string command = "ffplay -nodisp -autoexit ~/AlterEGO_v2/EGO_GUI/config/low_battery.mp3";

              std::string sshCommand = "ssh " +hostname_vision+"@"+IP_vision+" \"" + command + "\"";
              int result = system(sshCommand.c_str());
              std::cout<<"[INFO] Battery low. Playing sound"<<std::endl;
              system_check_file <<"\n[LQR] LQR ERROR: BATTERY LOW\n";
              system_check_file.close();
              battery_low_count = 0;
          }
        } else {
          battery_low_count = 0;
        }
      }
    }

    // Enc check
    if (ros::Time::now()-encReadTime > enc_max_latency){
      fault_sns = "Encoders";
      systemGood = false;
    }


  



    if (!systemGood && cnt == 1)
    {
      std::cout << "SYSTEM FAULT. No communication with " << fault_sns << std::endl;
      system_check_file <<"\n[LQR] LQR ERROR: "<<fault_sns<<" NOT FOUND\n";
      system_check_file.close();
      fault_sns = "";
      cnt=2;
    }
    if(systemGood && cnt == 0){
      system_check_file <<"[LQR] LQR ON\n";
      system_check_file.close();
      cnt=1;      
    }
    
    
    
    ros::spinOnce();
    r.sleep();
        
  }// end while()
return 0;
}
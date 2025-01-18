#include <ros/ros.h>
#include <ros/rate.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <eigen3/Eigen/Eigen>
#include <string>

ros::Time temperatureReadTime;
bool isTemperatureReadingActive = false;
bool do_reset = false;
bool pub_reset_done_flag = false;

void callback_temperature(const std_msgs::Float64::ConstPtr &msg)
{
  isTemperatureReadingActive = true;
  if (!isnan(msg->data) && (msg->data != 0.0))
  {
    temperatureReadTime = ros::Time::now();
  }
}

void callback_reset(const std_msgs::Bool::ConstPtr &msg)
{
  if (msg->data)
  {
    do_reset = true;
  }
  else
  {
    pub_reset_done_flag = false;
  }
}

//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
  double rateHZ = 50;
  unsigned int phase = 0;
  int res = 0;
  ros::Duration temperature_max_latency(10.0); // 10 sec.

  ros::init(argc, argv, "System_Check");
  ros::NodeHandle nh;

  temperatureReadTime = ros::Time::now();

  // Topic you want to subscribe
  ros::Subscriber sub_temperature = nh.subscribe("temperature", 1, callback_temperature);

  ros::Subscriber sub_reset = nh.subscribe("reset_comm", 1, callback_reset);
  ros::Publisher pub_reset_done = nh.advertise<std_msgs::Bool>("reset_comm_done", 1);

  ros::Rate r(rateHZ);

  std::vector<std::string> nodes;
  std::string name;
  std::string robot_name = std::getenv("ROBOT_NAME");
  std::string ith_node;
  std::string node_to_kill;
  std::cout << "[INFO] Start to spin reset_robot_node" << std::endl;
  ros::spinOnce();

  while (ros::ok())
  {

    // Temperature check
    if (!do_reset && isTemperatureReadingActive && (ros::Time::now() - temperatureReadTime > temperature_max_latency))
    {
      isTemperatureReadingActive = false;
      ith_node = "/" + robot_name + "/face_expressions";
      if (ros::master::getNodes(nodes))
      {
        for (unsigned int i = 0; i < nodes.size(); i++)
        {
          name = nodes.at(i);

          if (name == ith_node)
          {
            node_to_kill = "rosnode kill " + ith_node;
            res = system(node_to_kill.c_str()); // Stop Face Expressions
            usleep(3000000);
            res = system("roslaunch alterego_robot face_expressions.launch &");
            usleep(2000000);
            temperatureReadTime = ros::Time::now();
            isTemperatureReadingActive = true;
          }
        }
      }
    }

    if (do_reset)
    {

      switch (phase)
      {

      case 0: // Kill all nodes

        ith_node = "/" + robot_name + "/left/qb_manager " + robot_name + "/right/qb_manager";
        node_to_kill = "rosnode kill " + ith_node;
        res = system(node_to_kill.c_str()); // Stop Body Activation
        usleep(1000000);

        ith_node = "/" + robot_name + "/force_estimation";
        node_to_kill = "rosnode kill " + ith_node;
        res = system(node_to_kill.c_str()); // Stop Force Estimation
        usleep(1000000);

        ith_node = "/" + robot_name + "/head/head_manager " + robot_name + "/left/arm_inv_kin_main " + robot_name + "/left/arm_inv_dyn " + robot_name + "/pitch_correction " + robot_name + "/right/arm_inv_kin_main " + robot_name + "/right/arm_inv_dyn";
        node_to_kill = "rosnode kill " + ith_node;
        res = system(node_to_kill.c_str()); // Stop Body Movement
        usleep(1000000);

        ith_node = "/" + robot_name + "/left/vibrotactile_fb " + robot_name + "/right/vibrotactile_fb";
        node_to_kill = "rosnode kill " + ith_node;
        res = system(node_to_kill.c_str()); // Stop Haptic Feedback
        usleep(1000000);

        ith_node = "/" + robot_name + "/face_expressions";
        node_to_kill = "rosnode kill " + ith_node;
        res = system(node_to_kill.c_str()); // Stop Face Expressions
        usleep(3000000);

        //   res = system("rosnode kill /inbound_data /socket"); //Stop Pilot Communication
        //   usleep(1000000);

        phase = 1;

        break;
      // Execute just one thing at each cycle
      case 1: // Restart Body Activation
        res = system("roslaunch alterego_robot body_activation.launch &");
        usleep(5000000);
        phase = 2;
        break;
      case 2: // Restart Weigth Estimation
        res = system("roslaunch optoforce_sensor optoforce_sensor.launch &");
        usleep(6000000);
        phase = 3;
        break;
      case 3: // Restart Body Movement
        res = system("roslaunch alterego_robot body_movement.launch &");
        usleep(2000000);
        phase = 4;
        break;
      case 4: // Restart Haptic Feedback
        res = system("roslaunch alterego_robot vibrotactile_fb.launch &");
        usleep(2000000);
        phase = 5;
        break;
      case 5: // Restart Face Expression
        res = system("roslaunch alterego_robot face_expressions.launch &");
        usleep(2000000);
        temperatureReadTime = ros::Time::now();
        phase = 6;
        break;
        // case 6:   // Restart Pilot Communication
        //   res = system("roslaunch alterego_robot pilot.launch &");
        //   usleep(2000000);
        //   phase = 7;
        //   break;

      default: // Final phase
        do_reset = false;
        pub_reset_done_flag = true;
        phase = 0; // Listen for a new reset command
        break;
      }
    }

    if (pub_reset_done_flag)
    {
      std_msgs::Bool msg_rd;
      msg_rd.data = true;
      pub_reset_done.publish(msg_rd);
    }

    ros::spinOnce();
    r.sleep();

  } // end while()
  return 0;
}
#include <ros/ros.h>
#include <alterego_state_publisher.h>

int main(int argc, char **argv)
{
  double rateHZ = 0;

  ros::init(argc, argv, "alterego_state_publisher");

  ros::NodeHandle nh;	
  
  std::string ns_name;
	ns_name = ros::this_node::getNamespace();

  nh.getParam(ns_name+"/state_publisher_frequency", rateHZ);
  std::cout << "State publisher frequency: " <<rateHZ<<std::endl;


  alterego_state_publisher Obj(rateHZ); // to declare the object of the class ego_dance (object is the variable of a class that is a datatype)
  Obj.Init();
  
  ros::Rate loop_rate(rateHZ);
  // ros::spinOnce();

  while (ros::ok())
  {
    
    Obj.LowerBodyState();
    Obj.UpperBodyState();
    
    Obj.Publish();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/TwistStamped.h>
#include "boost/thread.hpp"

#define vel_step 0.1
#define lin_max 0.8
#define ang_max 0.4

float _fv(0.0); // Forward velocity
float _rv(0.0); // Rotational velocity
bool running = true;
using namespace std;
void keyboardloop()
{
    string input;
    cout << "Keyboard Input: " << endl;
    cout << "[w]: Forward direction increment" << endl;
    cout << "[s]: Backward direction increment" << endl;
    cout << "[a]: Left angular increment" << endl;
    cout << "[d]: Right angular increment" << endl;
    cout << "[k] or [z]: stop the robot!" << endl;
    while (running)
    {
        cin >> input;
        if (input == "w")
            _fv += (_fv > lin_max) ? 0.0 : vel_step;
        else if (input == "s")
            _fv += (_fv < -lin_max) ? 0.0 : -vel_step;
        else if (input == "a")
            _rv += (_rv > ang_max) ? 0.0 : -vel_step;
        else if (input == "d")
            _rv += (_rv < -ang_max) ? 0.0 : vel_step;
        else if ((input == "k") || (input == "z"))
        {
            _fv = 0;
            _rv = 0;
        }
        cout << "\nlin: " << _fv << "\tAng: " << -_rv << std::endl;
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "kb_controller");
    ros::NodeHandle _nh;
    ros::Publisher _vel_pub;
    _vel_pub = _nh.advertise<geometry_msgs::TwistStamped>("wheels/segway_des_vel", 1);
    ros::Rate r(400);
    std::vector<double> preset{0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8};
    std::vector<double> minus_preset{-0.8, -0.8, -0.8, -0.8, -0.8, -0.8, -0.8, -0.8, -0.8, -0.8, -0.8, -0.8};
    std_msgs::Float64MultiArray ref1;
    ref1.data.resize(6);
    ref1.data = preset;
    std_msgs::Float64MultiArray ref2;
    ref2.data.resize(6);
    ref2.data = minus_preset;

	geometry_msgs::TwistStamped cmd_vel;

    boost::thread ctrl_loop_t(&keyboardloop);
    while (ros::ok())
    {
		cmd_vel.header.stamp = ros::Time::now();
		cmd_vel.twist.linear.x = _fv;
		cmd_vel.twist.linear.y = 0;
		cmd_vel.twist.linear.z = 0;
		cmd_vel.twist.angular.x = 0;
		cmd_vel.twist.angular.y = 0;
		cmd_vel.twist.angular.z = -_rv;
		_vel_pub.publish(cmd_vel);
        r.sleep();
    }
    running = false;
    return 0;
}
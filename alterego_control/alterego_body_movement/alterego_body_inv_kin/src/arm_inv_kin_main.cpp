#include <arm_inv_kin.h>

int main(int argc, char **argv)
{
    double rateHZ = 100;

    ros::init(argc, argv, "kinematic_control");
    ros::NodeHandle n;
    std::string ns;
    ns = ros::this_node::getNamespace();

    ros::NodeHandle nh;


    invKinEgo obj(&nh, ns);
    obj.run(rateHZ);
}
#include <iostream>
#include "optoforce_sensor/opto.h"
#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include <unistd.h>
#include <string.h>

void msSleep(unsigned long p_uMillisecs)
{
    usleep(p_uMillisecs * 1000);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "optoforce_sensor_node");
    OptoDAQ daq;
    OptoPorts ports;
    msSleep(2500); // We wait some ms to be sure about OptoPorts enumerated PortList

    ros::NodeHandle nh = ros::NodeHandle("~");
    double spin_rate = 60;
    ros::Rate rate(spin_rate);

    int iSpeed = 30;                                                                                                             // Speed in Hz
    int iFilter = 15;                                                                                                           // Filter in Hz
    std::vector<double> cn_values = {450.47, 492.7, 65.175, 457.43, 466.23, 71.88, 446.54, 459.19, 69.98, 440.34, 469.9, 64.65}; // sensors 84 85 86 and 87
    SensorConfig sensorConfig;
    sensorConfig.setSpeed(iSpeed);
    sensorConfig.setFilter(iFilter);

    //     bool bConfig = false;
    //     bConfig = daq.sendConfig(sensorConfig);
    //     sleep(2.0);
    //     if (bConfig == false) {
    //         daq.close();
    //         ROS_ERROR_STREAM("Could not set config");
    //         ros::shutdown();
    //         return 0;
    //     }

    u_int8_t sens_params_set = 1;

    OPort *portlist = ports.listPorts(true);
    daq.open(portlist[0]);
    daq.zeroAll();
    bool bConfig = false;
    bConfig = daq.sendConfig(sensorConfig);
    sleep(1.0);
    while ((bConfig == false) && ros::ok())
    {
        ROS_ERROR_STREAM("Could not set config");
        bConfig = daq.sendConfig(sensorConfig);
        sleep(1.0);
        //         ros::shutdown();
        //         return 0;
    }

    double fx_gain, fy_gain, fz_gain; // sensitivity gain in counts/N

    int speed, filter;

    OptoPackage *pack3D = 0;
    int size = daq.readAll(pack3D, false);
    int sensSize = daq.getSensorSize();
    std::cout << "STARTING \n"
              << sensSize;
    double temp_x, temp_y, temp_z, temp_abs;

    daq.zeroAll();
    while (ros::ok())
    {

        // std::cout << "test" << std::endl;
        size = daq.readAll(pack3D, false);
        // std::cout << "SIZE " << size << std::endl;

        if (size == 0)
            continue;
        for (int k = 0; k < sensSize; k++)
        {
            temp_x = pack3D[k * size + size - 1].x / cn_values[k * 3];
            temp_y = pack3D[k * size + size - 1].y / cn_values[k * 3 + 1];
            temp_z = pack3D[k * size + size - 1].z / cn_values[k * 3 + 2];
            temp_abs = sqrt(temp_x * temp_x + temp_y * temp_y + temp_z * temp_z);
            std::cout << std::setprecision(2) << "Sensor number: " << k << "|| x: \t" << temp_x << "\t y: \t" << temp_y << "\t z: \t" << temp_z << std::setprecision(4)<<"\t |a|: \t" << temp_abs << std::endl; // std::cout << "Sensor " << k << " datas:" << std::endl;

            std::cout << "(Size: " << size << ")" << std::endl;

            // for (int i = 0; i < size; i++)
            // {
            //     std::cout << "x: " << pack3D[k * size + i].x << " y: " << pack3D[k * size + i].y << " z: " << pack3D[k * size + i].z << std::endl;
            // }
            std::cout << "End \n";
        }
        rate.sleep();
        // int size = daq.read(pack3D, false); // Reading Sensor #0 (up to 16 Sensors)
        // std::cout << "x: " << pack3D.x << " y: " << pack3D.y << " z: " << pack3D.z << std::endl;
    }

    daq.close();
    return 0;
}
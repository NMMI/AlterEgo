#!/usr/bin/env python3
import serial.tools.list_ports
import os
path = os.path.dirname(os.path.abspath(__file__))


def detect_lidar_usb_port():

    lidar_port = None
    left_arm_port = None
    right_arm_port = None
    wheels_port = None  
    imu_port = None
    face_port = None
    
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(port)
        if "CP2102" in port.description:
            lidar_port = port.device
        if "QBDUMMY 166" in port.description or "QBDUMMY 170" in port.description or "QBDUMMY 168" in port.description or "QBDUMMY 181" in port.description:
            left_arm_port = port.device
        if "QBDUMMY 167" in port.description or "QBDUMMY 173" in  port.description or  "QBDUMMY 164" in port.description or "QBDUMMY 18" in port.description:
            right_arm_port = port.device
        if "QBMMP" in port.description:
            wheels_port = port.device
        if "LB_0004119" in port.description:
            imu_port = port.device
        if "TTL232R" in port.description:
            face_port = port.device

    return lidar_port, left_arm_port, right_arm_port, wheels_port, imu_port, face_port



def update_robot_configuration(file_path, lidar_port, left_arm_port, right_arm_port, wheels_port, imu_port, face_port):
    # Read the file into memory
    with open(file_path, 'r') as file:
        lines = file.readlines()

    # Replace the line
    for i, line in enumerate(lines):
        if 'imu:' in line:
            lines[i+1] = f"  port: {imu_port}\n"

        if 'lidar:' in line:
            lines[i+1] = f"  serial_port: {lidar_port}\n"
        
        if 'left:' in line:
            lines[i+1] = f"  port: {left_arm_port}\n"

        if 'right:' in line:
            lines[i+1] = f"  port: {right_arm_port}\n"
        
        if 'wheels:' in line:
            lines[i+1] = f"  port: {wheels_port}\n"

        if 'face_expressions:' in line:
            lines[i+1] = f"  port: {face_port}\n"

    # Write the contents back to the file
    with open(file_path, 'w') as file:
        file.writelines(lines)

if(__name__ == "__main__"):

    lidar_port, left_arm_port, right_arm_port, wheels_port, imu_port, face_port = detect_lidar_usb_port()

    # Call the function to update the configuration file
    #get the path of this file

    path += '/robot_configuration.yaml'
    update_robot_configuration(path, lidar_port, left_arm_port, right_arm_port, wheels_port, imu_port, face_port)


    
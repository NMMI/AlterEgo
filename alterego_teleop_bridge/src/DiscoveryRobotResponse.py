#!/usr/bin/env python3

import rospy
import socket
import time
import os
import subprocess

def main():
    rospy.init_node('discovery_robot_response', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz

    # Get the robot name from the environment variable
    robot_name = os.getenv("ROBOT_NAME", "UnknownRobot")

    # Get the host IP address
    host_ip = subprocess.check_output(['hostname', '-I']).decode().split()[0]

    # Broadcast address and port
    broadcast_ip = "192.168.0.255"  # Replace with your network's broadcast address
    target_port = 8888

    # Message to broadcast
    message = f"Hello from {robot_name} at {host_ip}!"
    rospy.loginfo(f"Broadcasting message: {message}")

    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    while not rospy.is_shutdown():
        try:
            # Send the message as a broadcast
            sock.sendto(message.encode(), (broadcast_ip, target_port))
            rospy.loginfo("Message broadcasted successfully.")

            # Wait for a response (optional)
            sock.settimeout(10)  # Timeout for receiving responses
            try:
                response, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
                rospy.loginfo(f"Response received from {addr}: {response.decode()}")
            except socket.timeout:
                rospy.logwarn("No response received within 5 seconds.")

        except Exception as e:
            rospy.logerr(f"Error occurred: {e}")

        rate.sleep()

    sock.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

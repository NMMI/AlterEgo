#!/usr/bin/env python3

import socket
import time
import os
import socket
import subprocess

# Get the IP address based on the ROBOT_NAME environment variable
robot_name = os.getenv("ROBOT_NAME")
target_ip = "192.168.0.94"
# Get the IP address of the host machine
host_ip = subprocess.check_output(['hostname', '-I']).decode().split()[0]

target_port = 8888
message = f"Hello from {robot_name} at {host_ip}!"
print(f"Sending message: {message}")
# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    # Send the message
    sock.sendto(message.encode(), (target_ip, target_port))
    # print(f"Message sent to {target_ip}:{target_port}")

    # Receive the response
    sock.settimeout(10)  # Set a timeout of 3 seconds for receiving a response
    try:
        response, _ = sock.recvfrom(1024)  # Extract the response data
        # print(f"Response received: {response.decode()}")  # Decode and print the response
        # Check if the response has arrived
    except socket.timeout:
        print("No response received within 10 seconds.")



sock.close()

# AlterEgo-Base and Full robot Simulator
This Repo contains the AlterEgo ROS-package (Version 2).


The following work has been tested on March 17th 2023 on Ubuntu 20.04 with ROS-Noetic.
You will find the relative AlterEgo-Vision repository in the AlterEGO_vision_XPrize branch.

## XPrize Version
This project contains the modified version with longest arms, used in the ANA Avatar XPrize Finals

# Build and test

## First installation

For semplicity we have adopted a ROBOT_NAME environment variable as namespace for all the robot that we have produced.

So, you need to source in every terminal the following line 
```
export ROBOT_NAME=robot_alterego_sim
```
Otherwise you can also add it into the ```.bashrc```
## Add the submodules
```
cd AlterEGO
git submodule init
git submodule update
```

## Install and Build the RBDL
if not previously installed then: 
```
cd ~/(your_catkin_path)/src/AlterEgo_vs/utils/rbdl
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D RBDL_BUILD_ADDON_URDFREADER=ON -D RBDL_USE_ROS_URDF_LIBRARY=OFF ../
make
sudo make install
```
### Install dependencies
```
sudo apt install ros-noetic-realsense2-camera
sudo apt install ros-noetic-realsense2-description
sudo apt-get install ros-noetic-move-base
sudo apt-get install ros-noetic-amcl
sudo apt-get install ros-noetic-ddynamic-reconfigure
sudo apt-get install ros-noetic-rplidar-ros
sudo apt-get install ros-noetic-pointcloud-to-laserscan
sudo apt-get install ros-noetic-octomap-server
sudo apt-get install ros-noetic-ira-laser-tools
```
### Install dependencies for realsense
:https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide

## Build the solution
Move to source dir `cd ~/(your_catkin_path)/`. Then
```
catkin build std_msgs_stamped ego_msgs qb_interface
catkin build

```
# Run the simulation


Display the simulator with gazebo and rviz
```
#Change the version if you want to display the version 2(small), 3(tall)
roslaunch alterego_robot main.launch version:=2
```


## SLAM


Test slam with realsense lidar:
```
roslaunch navigation_stack create_map.launch
```
Move the robot with the keyboard for creating a good map:
```
roslaunch teleop_keyboard_cpp teleop.launch
```

Save the generated map 
```
#REMEMBER TO CHANGE "robot_alterego3" with the namespace of the computer
rosrun map_server map_saver -f office map:=/(your name_space pc)/map
```

Close the node 



## Planning and obstacle avoidance
Move the robot with move base and the ted local planner:
```
roslaunch navigation_stack autonomous_nav.launch
```
or add param nav = true to main.launch
```
roslaunch alterego_robot main.launch version:=3 nav:=true
```

## Info on how to setup the virtual cam in simulation
With the ros-virtual-cam we can simulate the camera plugin as a dev/ttyUSB device and send the stream via UDP and use the pilot oculus framework also in simulation


https://github.com/jgoppert/ros-virtual-cam

## Move IT
Download moveit in ROS
Run the demo.launch and the move_arm.py
There is a test_grasping_pose.py file that simulates the desidered grasping pose.

Write on the topic "centro" "destra" "sinistra" to set the desidered hand pose in a specific position

Download ycb-tools
we have download the https://github.com/sea-bass/ycb-tools.git inside the utils folder
And followed the respective ReadMe

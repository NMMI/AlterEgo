# AlterEgo-Base and Full robot Simulator
This Repo contains the AlterEgo ROS-package for both alterego small and tall.


The following work has been tested on March 14th 2024 on Ubuntu 20.04 with ROS-Noetic.
You will find the relative GUI repository in the vision branch.

![Alterego](https://github.com/IIT-SoftBots/AlterEGO_v2/blob/devel/alterego_description/images/alterego_real_sim.jpg)

![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)
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


![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)
## Create conda env
These four commands download the latest 64-bit version of the Linux installer, rename it to a shorter file name, silently install, and then delete the installer:
```
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm ~/miniconda3/miniconda.sh
```
After installing, close and reopen your terminal application or refresh it by running the following command:
```
source ~/miniconda3/bin/activate
```
To initialize conda on all available shells, run the following command:
```
conda init --all
```
Then deactivate the autostart activation of the base env
```
conda config --set auto_activate_base false
```

![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)
## Face Recognition and Tracking
If you want to use the face tracking and recognition, lets create the virtual env from the bash file ```create_env.sh``` inside 
```
cd ~/(your_catkin_path)/src/AlterEgo_v2/alterego_control/alterego_face_recognition 
./create_env.sh
cd ~/(your_catkin_path)
catkin build alterego_face_recognition alterego_face_tracking
```


![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)

## Build the entire solution
Move to source dir `cd ~/(your_catkin_path)/`. Then
```
catkin build
```

![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)
# Run the simulation


Display the simulator with gazebo and rviz
```
#Change the version if you want to display the version 2(small), 3(tall) or 4(small with kickstand)
roslaunch alterego_gazebo main.launch version:=2
```



![Alteregos](https://github.com/IIT-SoftBots/AlterEGO_v2/blob/devel/alterego_description/images/alteregos.png)
![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)

## SLAM


Test slam with realsense lidar:
```
roslaunch alterego_navigation create_map.launch
```
Move the robot with the keyboard for creating a good map:
```
roslaunch alterego_teleop_keyboard teleop.launch
```

Save the generated map 
```
#REMEMBER TO CHANGE "robot_alterego3" with the namespace of the computer
rosrun map_server map_saver -f office map:=/(your name_space pc)/map
```

Close the node 



![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)

## Planning and obstacle avoidance
Move the robot with move base and the ted local planner:
```
roslaunch alterego_navigation autonomous_nav.launch
```
or add param nav = true to main.launch
```
roslaunch alterego_gazebo main.launch version:=3 nav:=true
```


![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)
## Info on how to setup the virtual cam in simulation
With the ros-virtual-cam we can simulate the camera plugin as a dev/ttyUSB device and send the stream via UDP and use the pilot oculus framework also in simulation


https://github.com/jgoppert/ros-virtual-cam


![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)
## Move IT
Download moveit in ROS
Run the demo.launch 




![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)
<!-- Mantainers -->
<h2 id="Mantainers"> Mantainers</h2>
<p>
  :man: <b>Giovanni Rosato</b> <br>
  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Email: <a>giovanni.rosato@iit.it</a> <br>
  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; GitHub: <a href="https://github.com/GianniRos">@GianniRos</a> <br>
  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Linkedin: <a href="https://www.linkedin.com/in/giovanni-rosato-6284bb161/">@giovanni-rosato-linkedin</a> <br>
</p>
<p>
  :man: <b>Eleonora Sguerri</b> <br>
  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Email: <a>eleonora.sguerri@iit.it</a> <br>
  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; GitHub: <a href="https://github.com/EleSgu">@EleSgu</a> <br>
  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Linkedin: <a href="https://www.linkedin.com/in/eleonora-sguerri-202a4a217/">@eleonora-sguerri-linkedin</a> <br>
</p>
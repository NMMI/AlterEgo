# AlterEgo Real Software and Simulator

To clone with SSH use:
```
git clone --recursive git@github.com:NMMI/AlterEgo_Gazebo_Simulator.git

```

To clone with HTTPS use:

``` 
shell
git clone --recursive https://github.com/NMMI/AlterEgo_Gazebo_Simulator.git
```

# Build and test
# Add the submodules
```
cd AlterEgo_Gazebo_Simulator
git submodule init
git submodule update
```

# Install and Build the RBDL
if not previously installed then: 
```
cd ~/catkin_ws/src/AlterEgo_Gazebo_Simulator/utils/rbdl
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D RBDL_BUILD_ADDON_URDFREADER=ON -D RBDL_USE_ROS_URDF_LIBRARY=OFF ../
make
sudo make install
```
# Needed packages
```
sudo apt-get install ros-melodic-serial
sudo apt-get install ros-melodic-controller
sudo apt-get install ros-melodic-rplidar-ros


```
Move to source dir `cd ~/catkin_ws/`. Then
```
catkin_make
```

To test:
```
roslaunch ego_description ego_gazebo.launch 
```

# Params that you can add for testing 
True/false in the followings
```
adv_plugin:="value"
compliant_plugin:="value"
use_rviz:="value"
use_sim_time:="value"
pause:="value"
inv_kin:="value"
```

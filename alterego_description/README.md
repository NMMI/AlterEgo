# AlterEgo description
This Repo contains the AlterEgo ROS-package for both alterego small and tall.


![Alteregos](https://github.com/IIT-SoftBots/AlterEGO_v2/blob/devel/alterego_description/images/alteregos.png)

![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)
# Params
In this example we see an abstract of the ego_spawn.launch.

Inside the definition of the param ```robot_description``` we can modify the visualization of the robot. 
We can decide to have fixed hands instead of the plugin, a fixed base if we want to stay still respect to the world or enabling and disabling the streocamera or the realsense.

```
<arg name="AlterEgoVersion" default="3" />

<arg name="urdf_dir" value="$(find alterego_description)/urdf/ego_robot_gazebo.urdf.xacro" />


    <!-- Run a python script to the send a service call to gazebo_ros to spawn a URDF robot -->
    <param name="robot_description"
        command="$(find xacro)/xacro '$(arg urdf_dir)' 
        VERSION:=$(arg AlterEgoVersion)
        ENABLE_CAMERA:=false
        ENABLE_FLOATING_BASE:=false
        ENABLE_FIXED_BASE:=false
        ENABLE_LASER:=false
        ENABLE_REALSENSE:=false
        ENABLE_FIXED_HANDS:=true" />

    <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
        args="-urdf -model ego_robot -param robot_description -x 0 -y 0 -z 0.13 -R 0.0 -P 0.0 -Y 0" />

```

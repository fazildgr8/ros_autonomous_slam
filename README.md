# ros_autonomous_slam

This consist of a ROS package which uses the Navigation Stack to autonomously explore an unknown environment with help of GMAPPING and constructs an Map of the explored environment. Finally a pathplanning algorithm (A*) is used in the newly generated map to reach the goal.The Gazebo simulator is used for the simulation of the Turtlebot3 Waffle Pi robot. Various algorithms have been integrated for the Autonomously exploring the region and constructing the map with help of the 360 degree Lidar sensor. Different environments can be swapped within launch files to generate the map of the environment. As an update an Self Repairing A* algorithm which is known as dynamic path planning in volatile enviroments will be implemented and updated soon. Keep following :) 

### There are three Main steps to be executed in this project. Follow the execution of the different Launch files to acheive the required tasks of each step. (Look below for Project prerequisites and setup)

## Step 1 : Place the Robot in the Environment within Gazebo
Set your environment variable to the model robot to be used.
```
export TURTLEBOT3_MODEL=waffle_pi
source ~/.bashrc
```
Execute the given launch to open Gazebo with the given world file and place the robot Turtlebot3 Waffle pi model in it.
```
roslaunch ros_autonomous_slam turtlebot3_world.launch
```
Keep this process running always and execute other commands in a different terminal.
## Step 2 : Perform Autonomous exploration of the environment and generate the Map
```
roslaunch ros_autonomous_slam autonomous_explorer.launch
```
Run the Autonomous Explorer launch file which executes two tasks for us at the same time.
1. It starts the **SLAM** node in the Navigation stack with a custom modified RVIZ file to monitor the mapping of the environment.
2. It simultaneously starts the **Autonomous explorer** which is a Python based controller to move around the map grazing all the areas whcih helps the **SLAM** Node to complete the mapping.
**Monitor the Mapping process in RVIZ window** and sit back and relax unitll our robot finishes mapping XD .
Once you are satisfied with the constructed map, Save the map.
```
rosrun map_server map_saver -f my_map
```
The **my_map.pgm** and **my_map.yaml** gets saved in your worspace directory. Copy these to files to the package **maps** folder (catkin_ws\src\ros_autonomous_slam\maps).
Now your new map which is basically a occupancy grid is constructed !
## Step 3 : Perform pathplanning and go to goal in the environment

## Demo
![Virtual Pen Handwriting Detection](demo.gif)

## Prerequisites and setup for the Project
### ROS Installation
I used Ubuntu 18 OS with ros Melodic Version. Check the ROS official documentation for the Installation
[ROS Installation](http://wiki.ros.org/melodic/Installation/Ubuntu)

### Gazebo ROS Installation
The main Gazebo Simulator which is an stand alone application must be installed. Go through the documentation
[Gazebo Installation](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install).
Test the working of Gazebo and its version with 
```
gazebo
which gzserver
which gzclient
```
After Installing the Gazebo, the Gazebo ROS Package must be installed seperately
```
sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
```
Replace `melodic` with your version of ROS everwhere in this tutorial.
### Turtlebot3 packages
The Turtlebot3 ROS packages can be either downloaded and built from source files in your workspace
or else directly installed from the linux terminal. Either way works, I would recommend downloading the source files and building it in your workspace.
```
cd catkin_ws/src
git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3
git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations
cd ..
catkin_make
source /devel/setup.bash
```
#### Direct Installation
```
source /opt/ros/melodic/setup.bash
sudo apt-get install ros-melodic-turtlebot3-msgs
sudo apt-get install ros-melodic-turtlebot3
```
### Navigation Stack
The Navigation stack can also be downloaded as souce files to your workspace and built.
```
cd catkin_ws/src
git clone -b melodic-devel https://github.com/ros-planning/navigation
cd ..
catkin_make
source /devel/setup.bash
```



# ros_autonomous_slam

This consist of a ROS package which uses the Navigation Stack to autonomously explore an unknown environment with help of GMAPPING and constructs an Map of the explored environment. Finally a pathplanning algorithm (A*) is used in the newly generated map to reach the goal.The Gazebo is used for the simulation of the Turtlebot3 Waffle Pi robot. Various algorithms have been integrated for the Autonomously exploring the region and constructing the map with help of the 360 degree Lidar sensor. Different environments can be swapped within launch files to generate the map of the environment. As an update an Self Repairing A* algorithm which is known as dynamic path planning in volatile enviroments will be implemented and updated soon. Keep following :) 

## Prerequisites for the Project
### ROS Installation
### Gazebo ROS Installation
### Turtlebot3 packages
### Navigation Stack


## There are three Main steps to be executed in this project. Follow the execution of the different Launch files to acheive the reuired tasks of each step.

# Step 1 : Open the Gazebo with the required environment and Place the Robot
# Step 2 : Perform Autonomous exploration of the environment and generate the Map
# Step 3 : Perform an pathplanning and go to goal in the environment

## Demo
![Virtual Pen Handwriting Detection](demo.gif)

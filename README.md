[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

# Obstacle Avoiding Turtlebot

## Description

This repository covers the basic programming of gazebo. We implement a roomba type
behaviour in a turtlebot model. The simulation is done in gazebo.

## Overview

We create a package (turtle_walker) which handles the implementation of the obstacle
avoidance motion behaviour of the turtlebot model. Also, the implemntation of the code 
is done using OOP concepts. The turtlebot model publishes the laser sensor data 
which gives the distance of the robot from the obstacle. This data is subscribed by 
our node and using this the motion instructions are given to the robot. Thus, a
collition free motion of the turtlebot is achieved.

## Dependencies

- Ubuntu 16.04
- ROS Kinetic
- Gazebo (if not already installed with ROS)
- Turtlebot
- python (if not already installed)
- catkin_pkg (if not already installed while installing ROS)
For installing catkin_pkg and directly adding it to PYTHONPATH.
```
pip install catkin_pkg
```
Check if the catkin_pkg path is added to PYTHONPATH by using the following
command
```
echo $PYTHONPATH
```

## Installation

ROS Installation:
Install the ROS Kinetic for Ubuntu and it's dependencies using the [link](http://wiki.ros.org/kinetic/Installation/Ubuntu)

Gazebo Installation:
For the installation of Gazebo follow the [link](http://gazebosim.org/tutorials?tut=install_ubuntu)

To download the Turtlebot simulink model use the following command;
```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```

## Build Instructions

Run the following commands for building and running the Publisher and Subscriber
Nodes:

To make the catkin workspace:

--skip command 1(sudo rm -R ~/catkin_ws) if no such folder is found

```

sudo rm -R ~/catkin_ws
source /opt/ros/kinetic/setup.bash 
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
cd src/
git clone https://github.com/KrishnaBhatu/turtle_walker
cd ..
catkin_make
```

Now the package is ready to use

# Run instruction
[NOTE:] In order to better observe the functionality of the obstacle avoidance, move the obstacles in front of the moving
        turtlebot at some distance and then the bot will detect the obstacle and change it's path.
In the same terminal run the following command to execute the simulation
```
source devel/setup.bash
roslaunch turtle_walker goTurtle.launch
```
OR
If we want to record the bag file along with execution then run the following command
```
source devel/setup.bash
roslaunch turtle_walker goTurtle.launch run_rosbag:=true
```
The bag file will be created in the results folder.

Wait for gazebo to launch and then we will see the simulation of the turtlebot moving
in an environment avoiding the obstacles places in it.

# To Run the bag file
The bag file is used to store the log data from the publisher in a .bag file. The .bag file can be used to playback the previous messages that were published.

Close all nodes (ctrl+c) and close all terminals

Now for viewing the bag file;
First save the bag file while running the simulation by using the following command;
Open a new terminal,
```
source /opt/ros/kinetic/setup.bash
roslaunch turtle_walker goTurtle.launch run_rosbag:=true
```

To verify the .bag file run the following command;
```
cd <path to repository>/results
rosbag info goTurtle.bag
```
Also, Gazebo should be closed before running this.
Then,
Open another terminal

```
roscore
```

Open another terminal

```
cd <path to repository>/results
rosbag play goTurtle.bag
```

Now, to verify that the bag file has stored log data then run the following 
command(while the rosbag play is still running in a differnt terminal);
Open a new terminal;
```
source /opt/ros/kinetic/setup.bash
rostopic echo /cmd_vel_mux/input/navi
```



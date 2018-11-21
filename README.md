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
Output of this command will be:
```
path:        goTurtle.bag
version:     2.0
duration:    24.2s
start:       Dec 31 1969 19:00:00.59 (0.59)
end:         Dec 31 1969 19:00:24.79 (24.79)
size:        9.6 MB
messages:    18973
compression: none [13/13 chunks]
types:       bond/Status                           [eacc84bf5d65b6777d4c50f463dfb9c8]
             dynamic_reconfigure/Config            [958f16a05573709014982821e6822580]
             dynamic_reconfigure/ConfigDescription [757ce9d44ba8ddd801bb30bc456f946f]
             gazebo_msgs/LinkStates                [48c080191eb15c41858319b4d8a609c2]
             gazebo_msgs/ModelStates               [48c080191eb15c41858319b4d8a609c2]
             geometry_msgs/Twist                   [9f195f881246fdfa2798d1d3eebca84a]
             nav_msgs/Odometry                     [cd5e73d190d741a2f92e81eda573aca7]
             rosgraph_msgs/Clock                   [a9c97c1d230cfc112e270351a944ee47]
             rosgraph_msgs/Log                     [acffd30cd6b6de30f120938c17c593fb]
             sensor_msgs/Imu                       [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/JointState                [3066dcd76a6cfaef579bd0f34173e9fd]
             sensor_msgs/LaserScan                 [90c7ef2dc6895d81024acba2ac42f369]
             std_msgs/String                       [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage                    [94810edda583a504dfda3829e70d7eec]
topics:      /clock                                            2422 msgs    : rosgraph_msgs/Clock                  
             /cmd_vel_mux/active                                  1 msg     : std_msgs/String                      
             /cmd_vel_mux/input/navi                            243 msgs    : geometry_msgs/Twist                  
             /cmd_vel_mux/parameter_descriptions                  1 msg     : dynamic_reconfigure/ConfigDescription
             /cmd_vel_mux/parameter_updates                       1 msg     : dynamic_reconfigure/Config           
             /depthimage_to_laserscan/parameter_descriptions      1 msg     : dynamic_reconfigure/ConfigDescription
             /depthimage_to_laserscan/parameter_updates           1 msg     : dynamic_reconfigure/Config           
             /gazebo/link_states                               2409 msgs    : gazebo_msgs/LinkStates               
             /gazebo/model_states                              2409 msgs    : gazebo_msgs/ModelStates              
             /gazebo/parameter_descriptions                       1 msg     : dynamic_reconfigure/ConfigDescription
             /gazebo/parameter_updates                            1 msg     : dynamic_reconfigure/Config           
             /gazebo_gui/parameter_descriptions                   1 msg     : dynamic_reconfigure/ConfigDescription
             /gazebo_gui/parameter_updates                        1 msg     : dynamic_reconfigure/Config           
             /joint_states                                     2422 msgs    : sensor_msgs/JointState               
             /laserscan_nodelet_manager/bond                     46 msgs    : bond/Status                           (2 connections)
             /mobile_base/commands/velocity                     230 msgs    : geometry_msgs/Twist                  
             /mobile_base/sensors/imu_data                     2442 msgs    : sensor_msgs/Imu                      
             /mobile_base_nodelet_manager/bond                   92 msgs    : bond/Status                           (3 connections)
             /odom                                             2450 msgs    : nav_msgs/Odometry                    
             /rosout                                            270 msgs    : rosgraph_msgs/Log                     (9 connections)
             /rosout_agg                                        253 msgs    : rosgraph_msgs/Log                    
             /scan                                              227 msgs    : sensor_msgs/LaserScan                
             /tf                                               3048 msgs    : tf2_msgs/TFMessage                    (2 connections)
             /tf_static                                           1 msg     : tf2_msgs/TFMessage

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



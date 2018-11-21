/**
  * BSD 3-Clause License
  * Copyright (c) 2018, KrishnaBhatu
  * All rights reserved.
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  *
  * Redistributions of source code must retain the above copyright notice, this
  * list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  * Neither the name of the copyright holder nor the names of its
  * contributors may be used to endorse or promote products derived from
  * this software without specific prior written permission.
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *  @copyright (c) BSD
  *  @file    roomba.cpp
  *  @author  Krishna Bhatu
  *
  *  @brief Source code for class definition of Roomba 
  */
#include "roomba.hpp"

/// Default constructor
Roomba::Roomba() {
	/** Initialize the boolean flag to false, which will be later trigged 
	  * if obstacle is detected.
	  */
        collisionPossible = false;
}
/// Default destructor
Roomba::~Roomba() {
	/**
	  * When the object is destroyed the turtlebot is stopped by putting
	  * velocities in all direction zero.
	  */
        pos.linear.x = 0.0;
        pos.linear.y = 0.0;
        pos.linear.z = 0.0;
        pos.angular.x = 0.0;
        pos.angular.y = 0.0;
        pos.angular.z = 0.0;
        pub.publish(pos);
}

/**
  * Callback function which recieves the data of object distance from the laser
  * sensor and updates the object detection flag if the condition is achieved
  */
void Roomba::checkObstacle(const sensor_msgs::LaserScan::ConstPtr& data) {
        for (auto d : data->ranges) {
                if (d < 1.0) {
                        collisionPossible = true;
                        break;
                }
                collisionPossible = false;
        }
}
/// Main funciton for turtlebot obstacle avoidance
void Roomba::goTurtle() {
	/**
  	  * The advertise() function defines the role of node to the ROS master.
  	  * The first parameter is the topic on which the node publishes.
  	  * The second parameter is the size of buffer for output messages.
  	  */ 
        pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi",
                                               1000);
	/**
  	  * The subscriber() function defines the role of node to the ROS master
  	  * The first parameter is the topic on which the node subscribes.
  	  * The second parameter is the size of buffer for input messages.
  	  * The third parameter is the function called when there is message on
  	  * topic. 
  	  * The fourth parameter is the referance to the node object.
  	  */
        sub = n.subscribe <sensor_msgs::LaserScan> ("/scan", 50,
                                                  &Roomba::checkObstacle, this);
	/**
  	  * Frequency at which it loops (10Hz)
  	  */
        ros::Rate loop_rate(10);
        while (ros::ok()) {
                if (collisionPossible) {
                        ROS_INFO_STREAM("Obstacle Detected! TURN");
     			/** If obstacle is detected then stop the bot by making 
  	          	  * linear velocity zero and turn by making angular
  	          	  *  velocity 1
  	          	  */
                        pos.linear.x = 0.0;
                        pos.angular.z = -1.0;
                } else {
                        ROS_INFO_STREAM("Move Forward!");
     		        /** If obstacle is not detected then move the bot by 
  	          	  * making linear velocity 1 and don't turn by making
  	                  * angular velocity zero
  	                  */
                        pos.linear.x = 0.5;
                        pos.angular.z = 0.0;
                }
     		/**
      		  *  The publish() function is used to send messages.
      		  */
                pub.publish(pos);
                ros::spinOnce();
     		/**
      		  *  Sleep the publishing for the next iteration to maintain
      		  *  10Hz publish rate.
      		  */
                loop_rate.sleep();
        }
}


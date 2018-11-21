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
  *  @file    roomba.hpp
  *  @author  Krishna Bhatu
  *
  *  @brief Code for class declaration of Roomba 
  */
#ifndef INCLUDE_ROOMBA_HPP_
#define INCLUDE_ROOMBA_HPP_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

class Roomba {
 private:
  /// Variable used for publishing velocity message
  geometry_msgs::Twist pos;
  /// Variable for checking if there is an object in front if the
  /// turtlebot
  bool collisionPossible;
  /**
    * This fully initializes the node on which we will publish and
    * subscribe topics.
    */ 
  ros::NodeHandle n;
  /// A publisher type
  ros::Publisher pub;
  /// A subscriber type
  ros::Subscriber sub;

 public:
  /**
    * @brief Default constructor
    */
  Roomba();
  /**
    * @brief Default destructor
    */
  ~Roomba();
  /**
    * @brief A callback function which is called when a new 
    *	  message arrives at /scan topic
    * @param msg It is a boost shared pointer that points to 
    *	   message on /scan topic
    * @return void
    */
  void checkObstacle(const sensor_msgs::LaserScan::ConstPtr& data);
   /**
     * @brief The main function of the smartTurtle which
     *        repetitively publish velocity messages according
     *        to the information from the subscriber which is a
     *       laser sensor.
     * @return void
     */
  void goTurtle();
};
#endif  // INCLUDE_ROOMBA_HPP_

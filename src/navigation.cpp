/**
 * @file navigation.hpp
 * @author Ajinkya Parwekar
 * @author Karan Sutradhar
 * @author Mahmoud Dahmani
 * @brief The navigation.hpp file for Indoor Sports Court Ball Collection Robot project.
 * It contains Collection class methods definitions.
 * @Copyright "Copyright 2020" <Ajinkya Parwekar>
 * @Copyright "Copyright 2020" <Karan Sutradhar>
 * @Copyright "Copyright 2020" <Mahmoud Dahmani>
 * 
 * @section LICENSE
 *  
 * MIT License
 * Copyright (c) 2020 Ajinkya Parwekar, Karan Sutradhar, Mahmoud Dahmani
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "navigation.hpp"
#include "detection.hpp"
#include "avoidance.hpp"

Navigation::Navigation() {
  ROS_INFO_STREAM("Initiliazing the robot...");
  /// Initializing the linear and angular velocities in m/s and rad/s resp
  xVelLin = 0.3;
  zVelAng = 0.45;
  /// Making the previous velocities as current velocities
  prevVelLin = xVelLin;
  prevVelAng = zVelAng;
  /// Publishing on the mover topic for the velocities of the robot
  velocity = nh.advertise<geometry_msgs::Twist>
       ("/cmd_vel", 10);
  ROS_INFO_STREAM("The process is complete for navigation");
}

Navigation::Navigation(float velLin, float velAng) {
  ROS_INFO_STREAM("Initiliazing the robot...");
  /// Initializing the linear and angular velocities in m/s and rad/s resp
  xVelLin = velLin;
  zVelAng = velAng;
  /// Making the previous velocities as current velocities
  prevVelLin = xVelLin;
  prevVelAng = zVelAng;
  /// Publishing on the mover topic for the velocities of the robot
  velocity = nh.advertise<geometry_msgs::Twist>
        ("/cmd_vel", 10);
  ROS_INFO_STREAM("The process is complete for navigation");

}

float Navigation::moveAhead(float velLin) {
  move.linear.x = velLin;
  move.angular.z = 0.0;
  return move.linear.x;
}

float Navigation::turnDirection(float velAng) {
  move.linear.x = 0.0;
  move.angular.z = velAng;
  return move.angular.z;
}

void Navigation::robotMovement(Avoidance& avoidanceObj) {
  /// Setting the rate of publishing msg
  ros::Rate loop_rate(pubRate);
  while (ros::ok()) {
    if (avoidanceObj.checkWalls()) {
      /// The robot starts turing avoiding the walls
      turnDirection(zVelAng);
      /// Checking if there is a difference in velocities
      checkChangeInVelocity();
    } else {
        /// The robot starts moving ahead avoiding the walls
        moveAhead(xVelLin);
        /// Checking if there is a difference in velocities
        checkChangeInVelocity();
    }

    /// Publish the velocities
    velocity.publish(move);
    /// Handle callback
    ros::spinOnce();
    /// Make the system sleep to maintain loop rate
    loop_rate.sleep();
  }
}

bool Navigation::resetRobotVelocity() {
  ROS_INFO_STREAM("Resetting the robot config...");
  /// Reseting the linear velocities of the robot
  move.linear.x = 0.0;
  move.linear.y = 0.0;
  move.linear.z = 0.0;
  /// Resting the angular velocities of the robot
  move.angular.x = 0.0;
  move.angular.y = 0.0;
  move.angular.z = 0.0;
  /// Publish the reset velocities
  velocity.publish(move);
  ROS_INFO_STREAM("The resetting finished");
  return true;
}

bool Navigation::checkChangeInVelocity() {
  /// Verify that there is a difference in velocities from past data
  if (move.linear.x != prevVelLin and move.angular.z != prevVelAng) {
    ROS_DEBUG_STREAM("There is a change is robot velocity");
    /// upgrading the robot velocities
    move.linear.x = prevVelLin;
    move.angular.z = prevVelAng;
    return true;
  }

  return false;
}


Navigation::~Navigation() {
  if (resetRobotVelocity()) {
     ROS_INFO_STREAM("The velocity of the robot will be reset");
  }
}


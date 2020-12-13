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


#pragma once

#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "detection.hpp"

class Navigation {
 private:
  // Communication with the ROS system
  ros::NodeHandle nh;

  // Subscribe to laser scan topic
  ros::Subscriber sensorLaser;

  // Inilalize variable to store if object is detected
  ros::Publisher velocity;

  // msg variable that handles robot speeds
  geometry_msgs::Twist velocitymsg;

  // Defining minimun distance from the wall to avoid collision
  float thresholdDist;

  // Defining linear velocity in x direction
  float xVelLin;

  // Defining angular velocity in z direction
  float zVelAng;

  // Defining variables to store previous velocities
  float prevVelLin, prevVelAng;

  // Defining publishing rate
  const int pubRate = 200;

 public:
  // obstacle variable that defines presence of obstacles
  bool obstacles;
  /**
   * @brief Base Constructor for the Navigation class.
   * @param None.
   * @return None.
   */
  Navigation();
  /**
   * @brief Base Constructor for the Navigation class.
   * @param Linear velocity
   * @param Angular velocity
   * @return None.
   */
  Navigation(float velLin, float velAng);
  /**
   * @brief   Make the Robot move ahead
   * @param   linear velocity in x direction
   * @return  linear velocity of the robot
   */
  float moveAhead(float velLin);
  /**
   * @brief   change the direction
   * @param   angular velocity in z direction
   * @return  angular velocity for the robot
   */
  float turnDirection(float velAng);
  /**
   * @brief   Controling the motion of the robot
   * @param   none
   * @return  none
   */
  void robotMovement();
  /**
   * @brief   reset velocity of the robot
   * @param   none
   * @return  returns true, otherwise false
   */
  bool resetRobotVelocity();
  /**
  * @brief  checks if there is any modification in velocity
  * @param  none
  * @return true if changed, otherwise false
  */
  bool checkChangeInVelocity();
  /**
   * @brief sensor callback to subscribe the laser sensor callback topic
   * @param sensor_msgs::ScanLaser
   * @return None.
   */
  void laserSensorCallback(const sensor_msgs::LaserScan::ConstPtr& sensorData);
  /**
   * @brief Function to explore the world environment and set speed for the robot without collision
   * @param None.
   * @return geometry_msgs , to move robot without collision
   */
  geometry_msgs::Twist explore();
  /**
  * @brief  function to avoid walls
  * @param  Threshold distance from the walls
  * @return none
  */
  void avoidObstacle(float thresholdDist);
  /**
   * @brief   Checks the walls are present within threshold distance
   * @param   none
   * @return  TRue if found, otherwise false
   */
  bool checkWalls();
  /**
   * @brief Destructor for the Navigation class.
   * @param None.
   * @return None.
   */
  ~Navigation();
};

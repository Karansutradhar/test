/**
 * @file avoidance.cpp
 * @author Ajinkya Parwekar
 * @author Karan Sutradhar
 * @author Mahmoud Dahmani
 * @brief The avoidance.cpp file for Indoor Sports Court Ball Collection Robot project.
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
#include "avoidance.hpp"

Avoidance::Avoidance(){
  ROS_INFO_STREAM("Implementing the setup for obstacle avoidance for the robot");
  /// Declare value of balls/walls detected as false
  obstacles = false;
  /// Initialize minimun distance from the wall to avoid collision in meters
  thresholdDist = 0.75;
  /// Subscribing on scan topic from the laser sensor
  sensorLaser = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000,
              &Avoidance::laserSensorCallback, this);
  ROS_INFO_STREAM("The process is complete for avoidance");
}

Avoidance::Avoidance(float thresholdDistance){
  ROS_INFO_STREAM("Implementing the setup for obstacle avoidance for the robot");
  /// Declare value of balls/walls detected as false
  obstacles = false;
  /// Initialize minimun distance from the wall to avoid collision in meters
  thresholdDist = thresholdDistance;
  /// Subscribing on scan topic from the laser sensor
  sensorLaser = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000,
              &Avoidance::laserSensorCallback, this);
  ROS_INFO_STREAM("The process is complete for avoidance");
}

void Avoidance::laserSensorCallback(const sensor_msgs::LaserScan::ConstPtr& sensorData){
  /// Scanning the sensor laser data to get the distance of the robot from the obstacle
  for (const float &limit: sensorData->ranges) {
    if (limit <= thresholdDist) {
      ROS_INFO_STREAM("The distance is " << limit);
      setAvoidedObstacle(true);
      return;
    }
  }
  setAvoidedObstacle(false);
}

bool Avoidance::checkWalls(){
  /// Check if there is a wall ahead
  if (getAvoidedObstacle()) {
    ROS_WARN_STREAM("Walls ahead");
    return true;
  }
  return false;
}

Avoidance::~Avoidance(){}


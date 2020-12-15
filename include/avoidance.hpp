/**
 * @file avoidance.hpp
 * @author Ajinkya Parwekar
 * @author Karan Sutradhar
 * @author Mahmoud Dahmani
 * @brief The avoidance.hpp file for Indoor Sports Court Ball Collection Robot project.
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

class Avoidance {
 private:
  // Communication with the ROS system
  ros::NodeHandle nh;

  // Subscribe to laser scan topic
  ros::Subscriber sensorLaser;

  // Defining minimun distance from the wall to avoid collision
  float thresholdDist;

  // obstacle variable that defines presence of obstacles
  bool obstacles;
 
 public:
  /**
   * @brief Base Constructor for the Navigation class.
   * @param None.
   * @return None.
   */
  Avoidance();
  /**
   * @brief Base Constructor for the Navigation class.
   * @param Linear velocity
   * @param Angular velocity
   * @return None.
   */
  explicit Avoidance(float thresholdDistance);
  /**
   * @brief   Make the Robot move ahead
   * @param   linear velocity in x direction
   * @return  linear velocity of the robot
   */
  void laserSensorCallback(const sensor_msgs::LaserScan::ConstPtr& sensorData);
  /**
  * @brief  function to get balls detected
  * @param  none
  * @return Status of the ball detection
  */
  bool getAvoidedObstacle() const {
    return obstacles;
  }
  /**
  * @brief  function to set balls/walls detected
  * @param  Status of the ball detectionls
  * @return none
  */
  void setAvoidedObstacle(bool detectedObstacles) {
    obstacles = detectedObstacles;
  }
  /**
   * @brief   Checks the walls are present within threshold distance
   * @param   none
   * @return  True if found, otherwise false
   */
  bool checkWalls();
  /**
   * @brief Destructor for the Navigation class.
   * @param None.
   * @return None.
   */
  ~Avoidance();
};

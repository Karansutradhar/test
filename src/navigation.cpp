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
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Twist.h"
#include "navigation.hpp"
#include "detection.hpp"


Navigation::Navigation(){
	xVelLin = 3.0;
	zVelAng = 0.75;
}

Navigation::Navigation(float velLin, float velAng){
	xVelLin = velLin;
	zVelAng = velAng;
}

float Navigation::moveAhead(float velLin){
	return 3.0;
}

float Navigation::turnDirection(float velAng){
	return 0.75;
}

void Navigation::robotMovement(){
}

bool Navigation::resetRobotVelocity(){
	return true;
}

bool Navigation::checkChangeInVelocity(){
	return true;
}

void Navigation::laserSensorCallback(const sensor_msgs::LaserScan::ConstPtr& sensorData){}

geometry_msgs::Twist Navigation::explore(){
}

void Navigation::avoidObstacle(float thresholdDist){
}

bool Navigation::checkWalls(){
	return false;
}
  
Navigation::~Navigation(){}


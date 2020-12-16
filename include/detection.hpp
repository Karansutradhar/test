/**
 * @file detection.hpp
 * @author Ajinkya Parwekar
 * @author Karan Sutradhar
 * @author Mahmoud Dahmani
 * @brief The detection.hpp file for Indoor Sports Court Ball Collection Robot project.
 * It contains Detection class methods definitions.
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
#include <vector>
#include <string>
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"


/**
 * @brief Detection of the colored objects.
 */
class Detection {
 private:
  // color code RGB
  ros::NodeHandle nh;
  // subscriber to obhect to the data of laser sensor
  ros::Subscriber colorObjSub;
  // cros images stores as cv images
  cv::Mat imgHsv;
  cv::Mat imgMask;
  // identifying the colored objects limiting points
  cv::Rect colorObjlimits;
  // setting upper and lower limit of color
  const cv::Scalar lowerLimit = {0, 45, 170};
  const cv::Scalar upperLimit = {255, 255, 255};
  // defining image size
  cv::Size imgSize;
  // image contours
  std::vector<std::vector<cv::Point> > imgContours;
  // to check color is detected
  bool isObjDected;

 public:
  // an attribute stoes the images
  cv:: Mat imgStorage;
  /**
   * @brief Base Constructor for the Detection class.
   * @param None.
   * @return None.
   */
  Detection();
  /**
   * @brief Conversion of ROS images to CV images
   * @param image data from camera
   * @return None.
   */
  void imgConversion(const sensor_msgs::Image::ConstPtr& imgData);
  /**
   * @brief Function detect colored objects
   * @param gaussian filterd image
   * @return true if color is detected, otherwise false
   */
  bool detectObjs(cv::Mat objects);
  /**
   * @brief Function to modify gaussian filter on the image
   * @param changed open cv image
   * @return blurred image
   */
  cv::Mat filterImage(cv::Mat imgFiltered);
  /**
   * @brief set limiting condition for objects
   * @param Circular limits to the colored objects
   * @return None
   */
  void setObjLimits(cv::Rect limits) {
    colorObjlimits = limits;
  }
  /**
   * @brief get limiting condition for objects
   * @param None
   * @return Circular limits consist of the colored object
   */
  cv::Rect getObjLimits() const {
    return colorObjlimits;
  }
  /**
   * @brief setting object detected
   * @param Status of object detected
   * @return None
   */
  void setIsObjDetected(bool objs) {
    isObjDected = objs;
  }
  /**
   * @brief getting object detected
   * @param None
   * @return True if object is detected, otherwise false
   */
  bool getIsObjDetected() const {
    return isObjDected;
  }
 /**
   * @brief Base Destructor for the Detection class.
   * @param None.
   * @return None.
   */
  ~Detection();
};

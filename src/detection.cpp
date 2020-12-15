/**
 * @file detection.cpp
 * @author Ajinkya Parwekar
 * @author Karan Sutradhar
 * @author Mahmoud Dahmani
 * @brief The detection.cpp file for Indoor Sports Court Ball Collection Robot project.
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


#include <iostream>
#include <vector>
#include <string>
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "detection.hpp"

Detection::Detection() {
  ROS_INFO_STREAM("Here we initilize the balls detection");
  /// Subscribe to robot camera to get the data
  colorObjSub = nh.subscribe("/camera/rgb/image_raw", 1,
    &Detection::imgConversion, this);
  ROS_INFO_STREAM("Detection initilization completed");
}

void Detection::imgConversion(const sensor_msgs::Image::ConstPtr& imgData) {
  /// Create an object that bridges the ROS and OpenCV image conversion
  cv_bridge::CvImagePtr cv_img;
  try {
    cv_img = cv_bridge::toCvCopy(imgData, sensor_msgs::image_encodings::BGR8);
    imgStorage = cv_img->image;
    /// Wait for 20ms
    cv::waitKey(20);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR_STREAM("Problem in bridging " << e.what());
    return;
}
}

bool Detection::detectObjs(cv::Mat objects) {
  /// Converting image from BGR to HSV
  cv::cvtColor(objects, imgHsv, CV_BGR2HSV);
  /// Detecting hsv in the limits or not
  cv::inRange(imgHsv, lowerLimit, upperLimit, imgMask);
  /// Modifying size of masked image using image data
  imgSize = objects.size();
  imgMask(cv::Rect(0, 0, imgSize.width, 0.8*imgSize.height)) = 0;
  /// Drawing contours for better display
  cv::findContours(imgMask, imgContours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
  /// Checking for contours in the image
  if (imgContours.size() != 0) {
    auto sizeContour = 0;
    auto maxContourArea = 0;
    auto count = 0;
    while (count < imgContours.size()) {
      /// Finding contours in specific area
      if (sizeContour < imgContours[count].size()) {
         maxContourArea = count;
         sizeContour = imgContours[count].size();
      }
      count++;
    }
    /// Setting boundary condition
    setObjLimits(cv::boundingRect(imgContours[maxContourArea]));
    /// Using cv::Rect and drawing the boundary rectangle
    rectangle(objects, getObjLimits(), cv::Scalar(0, 255, 0), 2);
  }
  /// Masking the image for further usage in codes
  imgMask(cv::Rect(0, 0, 0.3*imgSize.width, imgSize.height)) = 0;

  if (cv::countNonZero(imgMask) == 0) {
    setIsObjDetected(true);
  } else {
     setIsObjDetected(false);
  }
  cv::namedWindow("HSV Image View");
  cv::namedWindow("Navigation View");
  imshow("HSV Image View", imgHsv);
  imshow("Navigation View", objects);
  return getIsObjDetected();
}

cv::Mat Detection::filterImage(cv::Mat imgFiltered) {
  cv::Mat outputData;
  /// Applying the openCV method GaussianBlur
  cv::GaussianBlur(imgFiltered, outputData, cv::Size(3, 3), 0.1, 0.1);
  return outputData;
}

Detection::~Detection() {}


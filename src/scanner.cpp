/**
 * @file scanner.cpp
 * @author Ajinkya Parwekar
 * @author Karan Sutradhar
 * @author Mahmoud Dahmani
 * @brief The scanner.cpp file for Indoor Sports Court Ball Collection Robot project.
 * It contains code implement the object detection algorithm using HSV color detection
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


#include "ros/ros.h"
#include "detection.hpp"

/**
 * @brief main function for the project.
 * @param argc
 * @param argv
 * @return int
 */

int main(int argc, char **argv) {
  /// Initializing scanning node
  ros::init(argc, argv, "scanner");
  /// Initializing object of class detection
  Detection detectionObj;

  while (ros::ok()) {
    /// Checks for if the image is empty
    if (!detectionObj.imgStorage.empty()) {
      /// detectionObj method is applied to detect colored balls in the world
        detectionObj.detectObjs(detectionObj.filterImage(detectionObj.imgStorage));
    }
  ros::spinOnce();
  }
  /// Closing both the windows of HSV Image View and Navigator View
  cv::destroyWindow("HSV Image View");
  cv::destroyWindow("Navigator View");
  return 0;
}

/**
 * @file test.cpp
 * @author Ajinkya Parwekar
 * @author Karan Sutradhar
 * @author Mahmoud Dahmani
 * @brief The test.cpp file for Indoor Sports Court Ball Collection Robot project.
 * It contains unit test cases for all the class moethods.
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


#include <gtest/gtest.h>
#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <navigation.hpp>
#include <detection.hpp>

/**
 * @brief This test checks if the filterImage function works as expected
 * @param Test1 is the name of the group of tests
 * @param filterImageFunctionTest is the specific name to check the filterImage function
 */
TEST(Test1, filterImageFunctionTest) {
    Detection detectionObj;
    bool filterCheck = true;
    // Checking if file is empty or not
    if (!detectionObj.imgStorage.empty()) {
        cv::Mat filerImg = detectionObj.filterImage(
    		detectionObj.imgStorage);
    	if(filerImg.size() == detectionObj.imgStorage.size()) {
    	// smoothing will help the images not to have same size
    		filterCheck = false;
    	}
    }
    EXPECT_TRUE(filterCheck);
}

/**
 * @brief This test checks if the detectObjs function works as expected
 * @param Test1 is the name of the group of tests
 * @param detectObjsFunctionTest is the specific name to check the detectObjs function
 */

TEST(Test1, detectObjsFunctionTest) {
    Detection detectionObj;
    bool objDetected;
    if (!detectionObj.imgStorage.empty()) {
    	objDetected = detectionObj.detectObjs(
    		detectionObj.filterImage(detectionObj.imgStorage));
    }
    //condition where object not detected
    EXPECT_FALSE(objDetected);
}

/**
 * @brief This test checks if the setObjLimits function works as expected
 * @param Test1 is the name of the group of tests
 * @param setObjLimitsFunctionTest is the specific name to check the setObjLimits function
 */
TEST(Test1, setObjLimitsFunctionTest) {
    Detection detectionObj;
    // definig the limit of the object
    cv::Rect limits = {0, 2, 4, 6};
    detectionObj.setObjLimits(limits);
    EXPECT_EQ(detectionObj.getObjLimits(), limits);
}

/**
 * @brief This test checks if the setIsObjDetected and getIsObjDetected function works as expected
 * @param Test1 is the name of the group of tests
 * @param IsObjDetectedFunctionTest is the specific name to check the setIsObjDetected and getIsObjDetected function
 */

TEST(Test1, IsObjDetectedFunctionTest) {
    Detection detectionObj;
    detectionObj.setIsObjDetected(true);
    EXPECT_TRUE(detectionObj.getIsObjDetected());
}

/**
 * @brief This test checks if the setIsObjDetected and getIsObjDetected function works as expected
 * @param Test1 is the name of the group of tests
 * @param IsObjNotDetectedFunctionTest is the specific name to check the setIsObjDetected and getIsObjDetected function
 */

TEST(Test1, IsObjNotDetectedFunctionTest) {
    Detection detectionObj;
    detectionObj.setIsObjDetected(false);
    EXPECT_FALSE(detectionObj.getIsObjDetected());
}

/**
 * @brief This test checks if the moveAhead function works as expected
 * @param Test1 is the name of the group of tests
 * @param moveAheadFunctionTest is the specific name to check the moveAhead function
 */

TEST(Test1, moveAheadFunctionTest) {
    // Defining linear and angular velocities for the robot
    float linVelocity = 3.0;
    float angVelocity = 0.75;
    Navigation navigationObj(linVelocity, angVelocity);
    EXPECT_EQ(linVelocity, navigationObj.moveAhead(linVelocity));
}

/**
 * @brief This test checks if the turnDirection function works as expected
 * @param Test1 is the name of the group of tests
 * @param turnDirectionFunctionTest is the specific name to check the turnDirection function
 */

TEST(Test1, turnDirectionFunctionTest) {
    // Defining linear and angular velocities for the robot
    float linVelocity = 3.0;
    float angVelocity = 0.75;
    Navigation navigationObj(linVelocity, angVelocity);
    EXPECT_EQ(angVelocity, navigationObj.turnDirection(angVelocity));
}

/**
 * @brief This test checks if the resetRobotVelocity function works as expected
 * @param Test1 is the name of the group of tests
 * @param resetRobotVelocityFunctionTest is the specific name to check the resetRobotVelocity function
 */

TEST(Test1, resetRobotVelocityFunctionTest) {
    Navigation navigationObj;
    EXPECT_TRUE(navigationObj.resetRobotVelocity());
}

/**
 * @brief This test checks if the checkChangeInVelocity function works as expected
 * @param Test1 is the name of the group of tests
 * @param checkChangeInVelocityFunctionTest is the specific name to check the checkChangeInVelocity function
 */

TEST(Test1, checkChangeInVelocityFunctionTest) {
    Navigation navigationObj;
    EXPECT_TRUE(navigationObj.checkChangeInVelocity());
}

/**
 * @brief This test checks if the avoidObstacle function works as expected
 * @param Test1 is the name of the group of tests
 * @param avoidObstacleFunctionTest is the specific name to check the avoidObstacle function
 */

TEST(Test1, avoidObstacleFunctionTest) {
    Navigation navigationObj;
    EXPECT_FALSE(navigationObj.checkWalls());
}

/**
 * @brief This test checks if the checkWalls function works as expected
 * @param Test1 is the name of the group of tests
 * @param checkWallsFunctionTest is the specific name to check the checkWalls function
 */

TEST(Test1, checkWallsFunctionTest) {
    // Define Threshold distance from the walls
    float tDistance = 0.20;
    Navigation navigationObj;
    navigationObj.avoidObstacle(tDistance);
    EXPECT_FALSE(navigationObj.checkWalls());
}
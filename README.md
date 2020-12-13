# Indoor Sports Court Ball Collection Robot
[![Build Status](https://travis-ci.org/dahhmani/collection_robot.svg?branch=master)](https://travis-ci.org/dahhmani/collection_robot)
[![Coverage Status](https://coveralls.io/repos/github/dahhmani/collection_robot/badge.svg?branch=master)](https://coveralls.io/github/dahhmani/collection_robot?branch=master)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

## Overview and Description

This project aims to solve a common problem in the sports industry, namely tidying up the sports court (that may be used for multiple sessions at the same day) and getting it back to its original state. Our objective is to automate the process of collecting all the used balls that will be scattered all over the court at the end of each practise session. We intend to implement a ball collector robot that operates inside an indoor sports court. The robot will navigate around the environment at various locations detecting objects. Then it will reach the object to pick and place the object at a specified location. We will use object detection, mapping, obstacle avoidance, and optimal path to increase the efficiency of object collection. This robot is used to search and collect objects from unknown locations and get back to the starting position.

We intend on creating a robust set of test cases with:

    cmake
    gtest
    rostest


## Authors

Phase-I

Ajinkya Parwekar (Driver)
Karan Sutradhar (Navigator)
Mahmoud Dahmani (Design Keeper)

Phase-II

Karan Sutradhar (Driver)
Mahmoud Dahmani (Navigator)
Ajinkya Parwekar (Design Keeper)

## License

Standard MIT License Clause

## Agile Iterative Process

[Agile Iterative Process Google Spreadsheet](https://docs.google.com/spreadsheets/d/1vfRqBHIk1xVdc3z9uviF9HBWrTOSSFLzZFpkkpyqecQ/edit?usp=sharing)

## Sprint Planning Notes
[Sprint Planning Notes For Phase 1](https://docs.google.com/document/d/1q7eatA6GpOcHOXkSIqnTkPcHaBq6XO4dyp4ps58LC04/edit?usp=sharing)

[Sprint Planning Notes For Phase 2](https://docs.google.com/document/d/1Y0JmpGLg45UZfyCOKwgZgbEW_Spy7mX2NEYs3VM1IGU/edit?usp=sharing)

## Project Dependencies

- Creation and implemenation of this ROS package was on ROS Melodic Ubuntu 18.04 (Linux) and Gazebo 9.0.0 version.
- Catkin is used for building this package. CMake (build system)
- OpenCV >= 4.4
- Follow the C++ 11 standard style of coding.
- ROS Melodic
- A standard turtlebot3 pkg needs to be installed in order to run this project.


### Install ROS Melodic

In order to Install ROS Melodic follow the following ROS.org [link](http://wiki.ros.org/melodic/Installation/Ubuntu)

### Installation link for turtlebot3 package:

The instructions to install the standard turtlebot3 ROS package can be found [here](https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros/).

### Standard dependencies

  - roscpp

  - rospy 

  - move_base_msgs

  - gmapping slam packages

  - std_msgs

  - sensor_msgs

  - geometry_msgs

  - rostest

  - rosbag

  - tf

## How to run tests

```
cd ~/catkin_ws/
source devel/setup.bash
catkin_make run_tests collection_robot

```

## Steps to run the program
```
git clone --recursive https://github.com/dahhmani/collection_robot.git
cd <path to repository>
mkdir build
cd build
cmake ..
make
Run tests: ./test/cpp-test
Run program: ./app/shell-app

```
## Cpplint check
```
cd  <path to repository>
cpplint $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./docs/" -e "^./results" )

```

Cppcheck check
```
cd <path to repository>
cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" )

```

## Doxygen File generation
```
sudo apt-get install doxygen
doxygen -g
Open Doxygen file and source file in "INPUT" prameter and add the include and app folder
Add "*.hpp *.cpp" in the "FILE_PATTERNS" parameter in the doxygen file
Run "doxygen ./Doxyfile" in te terminal
Open html folder
open index.html
```

## Building for code coverage
```
sudo apt-get install lcov
cmake -D COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug ../
make
make code_coverage
```
This generates a index.html page in the build/coverage sub-directory that can be viewed locally in a web browser.


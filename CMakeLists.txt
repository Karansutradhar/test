cmake_minimum_required(VERSION 3.2.1)
project(collection_robot)

## Compile as C++11, supported in ROS Melodic and newer
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  move_base_msgs
  cv_bridge
  image_transport
)

## Declare a catkin package
catkin_package()

## Build target library
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Add executables
add_executable(collection src/avoidance.cpp src/mover.cpp src/navigation.cpp)

add_executable(scanner src/detection.cpp src/scanner.cpp)

## Add target link libraries
target_link_libraries(collection ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(scanner ${catkin_LIBRARIES} ${OpenCV_LIBS})

## Enable catkin test and add files for the same
if(CATKIN_ENABLE_TESTING)
  set(CMAKE_BUILD_TYPE Debug)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")

  add_rostest_gtest(
   collection_robot_test
   test/test.launch
   test/main.cpp
   test/test.cpp
   src/detection.cpp
   src/navigation.cpp
   src/avoidance
   )

  target_link_libraries(
   collection_robot_test 
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS})

  add_dependencies(
  	collection_robot_test
    collection
    scanner
    ${catkin_EXPORTED_TARGETS})
endif()


cmake_minimum_required(VERSION 3.0.2)
project(upower_ros)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    rospy
    sensor_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    sensor_msgs
)

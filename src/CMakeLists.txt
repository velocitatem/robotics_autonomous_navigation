cmake_minimum_required(VERSION 3.0.2)
project(robotics_autonomous_navigation)

set(CATKIN_TOPLEVEL TRUE)

if(EXISTS "/opt/ros/noetic/share/catkin/cmake/toplevel.cmake")
  include("/opt/ros/noetic/share/catkin/cmake/toplevel.cmake")
else()
  message(FATAL_ERROR "catkin toplevel.cmake not found. Build inside the ROS Noetic Docker image.")
endif()

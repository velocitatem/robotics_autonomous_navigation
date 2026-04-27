cmake_minimum_required(VERSION 3.0.2)
project(rosbot_navigation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy sensor_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/ir_safety_stop.py
    scripts/laser_nav_safety.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

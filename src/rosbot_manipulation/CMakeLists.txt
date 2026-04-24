cmake_minimum_required(VERSION 3.0.2)
project(rosbot_manipulation)

find_package(catkin REQUIRED COMPONENTS
  rosbot_competition_msgs
  rospy
  std_msgs
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS rosbot_competition_msgs rospy std_msgs std_srvs
)

catkin_install_python(
  PROGRAMS
    scripts/manipulation_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

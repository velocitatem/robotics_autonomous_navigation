cmake_minimum_required(VERSION 3.0.2)
project(rosbot_dashboard)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  nav_msgs
  rosbot_competition_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    nav_msgs
    rosbot_competition_msgs
    rospy
    sensor_msgs
    std_msgs
    tf2_ros
)

catkin_install_python(
  PROGRAMS
    scripts/dashboard_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

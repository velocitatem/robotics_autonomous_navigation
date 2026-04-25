cmake_minimum_required(VERSION 3.0.2)
project(rosbot_mission_control)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  rosbot_competition_msgs
  rospy
  std_msgs
  std_srvs
  tf2_ros
  smach
  smach_ros
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
    move_base_msgs
    rosbot_competition_msgs
    rospy
    std_msgs
    std_srvs
    tf2_ros
    smach
    smach_ros
)

catkin_install_python(
  PROGRAMS
    scripts/mission_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

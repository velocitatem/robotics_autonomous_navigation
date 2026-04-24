cmake_minimum_required(VERSION 3.0.2)
project(rosbot_perception)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  message_filters
  rosbot_competition_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    image_transport
    message_filters
    rosbot_competition_msgs
    rospy
    sensor_msgs
    std_msgs
    tf2_geometry_msgs
    tf2_ros
)

catkin_install_python(
  PROGRAMS
    scripts/perception_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

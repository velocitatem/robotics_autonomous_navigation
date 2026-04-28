cmake_minimum_required(VERSION 3.0.2)
project(rosbot_navigation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs rospy sensor_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/ir_safety_stop.py
    scripts/laser_nav_safety.py
    scripts/scan_normalizer.py
    scripts/static_map_proxy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

cmake_minimum_required(VERSION 3.0.2)
project(rosbot_simulation)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  rosbot_competition_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS
    gazebo_msgs
    geometry_msgs
    rosbot_competition_msgs
    rospy
    sensor_msgs
    std_msgs
    std_srvs
)

catkin_install_python(
  PROGRAMS
    scripts/generate_aruco_markers.py
    scripts/spawn_objects.py
    scripts/sim_manipulation_node.py
    scripts/sim_ir_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

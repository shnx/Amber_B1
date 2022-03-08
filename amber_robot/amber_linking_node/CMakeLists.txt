cmake_minimum_required(VERSION 2.8.3)
project(amber_linking_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  rospy 
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
)

catkin_install_python(PROGRAMS src/amber_link/amber_link.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

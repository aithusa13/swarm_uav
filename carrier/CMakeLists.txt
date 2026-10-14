cmake_minimum_required(VERSION 3.0.2)
project(carrier)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  mavros_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs mavros_msgs
)

catkin_install_python(PROGRAMS
  scripts/carrier.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

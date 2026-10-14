cmake_minimum_required(VERSION 3.0.2)
project(area_scanning)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  mavros_msgs
  tf2
)

catkin_package(
  CATKIN_DEPENDS
    roscpp rospy std_msgs nav_msgs sensor_msgs
    geometry_msgs mavros_msgs tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(drone_lib
  src/Drone.cpp
)
target_link_libraries(drone_lib ${catkin_LIBRARIES})

add_executable(area_scan src/area_scan.cpp)
target_link_libraries(area_scan
  drone_lib
  ${catkin_LIBRARIES}
)
add_dependencies(area_scan ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  scripts/person_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

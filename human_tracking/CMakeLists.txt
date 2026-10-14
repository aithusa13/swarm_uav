cmake_minimum_required(VERSION 3.0.2)
project(human_tracking)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  sensor_msgs
  mavros_msgs
  std_msgs
  roscpp
  rospy
  message_generation
)

add_message_files(
  FILES
  PersonBBox.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS cv_bridge sensor_msgs mavros_msgs std_msgs roscpp rospy message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(drone_lib
  src/Drone.cpp
)
target_link_libraries(drone_lib ${catkin_LIBRARIES})
add_dependencies(drone_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(follow_person src/follow_person.cpp)
target_link_libraries(follow_person
  drone_lib
  ${catkin_LIBRARIES}
)
add_dependencies(follow_person ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  scripts/person_tracker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

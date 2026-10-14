cmake_minimum_required(VERSION 3.0.2)
project(kamikaze_dive)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  mavros_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs mavros_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(dive_lib
  src/DiveController.cpp
)
target_link_libraries(dive_lib ${catkin_LIBRARIES})

add_executable(main_node src/main_node.cpp)
target_link_libraries(main_node
  dive_lib
  ${catkin_LIBRARIES}
)
add_dependencies(main_node ${catkin_EXPORTED_TARGETS})

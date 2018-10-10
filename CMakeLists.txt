cmake_minimum_required(VERSION 2.8.3)
project(rviz_display)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometric_shapes
  geometry_msgs
  roscpp
  roslib
  visualization_msgs
  pcl_ros
)

find_package(PCL 1.7 REQUIRED COMPONENTS common io)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES rviz_display
   CATKIN_DEPENDS geometric_shapes geometry_msgs roscpp roslib visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
  add_library(${PROJECT_NAME}
    src/rviz_display.cpp
    include/${PROJECT_NAME}/rviz_display.h
)

add_dependencies(rviz_display ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rviz_display ${catkin_LIBRARIES})


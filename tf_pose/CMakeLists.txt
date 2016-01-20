cmake_minimum_required(VERSION 2.8.3)
project(tf_pose)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tf_pose
#  CATKIN_DEPENDS roscpp rospy tf
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable (tf_pose tf_pose.cpp)
target_link_libraries (tf_pose ${catkin_LIBRARIES})



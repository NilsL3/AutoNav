cmake_minimum_required(VERSION 2.8.3)
project(controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp rospy)

catkin_package()

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable (navcontrol navcontrol.cpp)
target_link_libraries (navcontrol ${catkin_LIBRARIES})








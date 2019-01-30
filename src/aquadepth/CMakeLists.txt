cmake_minimum_required(VERSION 2.8.3)
project(aquadepth)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -Wall -O2 -Wno-deprecated")

find_package(catkin REQUIRED COMPONENTS
  aquacore
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
)

SET ( Boost_USE_MULTITHREADED ON )
find_package(Boost REQUIRED COMPONENTS system thread)


################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
    cfg/DepthFilter.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aquadepth
  CATKIN_DEPENDS aquacore dynamic_reconfigure roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(depth_filter src/DepthFilterNode.cpp)
target_link_libraries(depth_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(depth_filter ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

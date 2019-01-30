cmake_minimum_required(VERSION 2.8.3)
project(aqua_gazebo)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++14 -O3")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  aquacore
  aqua_description
  aqua_gait
  roscpp
  gazebo_ros
  sensor_msgs
  std_msgs
  genmsg 
  message_generation
  dynamic_reconfigure
)

add_message_files(
  FILES
  HydrodynamicsParams.msg
  HydrodynamicsParamsList.msg
  ThrustParams.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  aqua_gazebo
)

# generate dyn reconf
generate_dynamic_reconfigure_options(
    cfg/Hydrodynamics.cfg    
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES aqua_gazebo
  CATKIN_DEPENDS 
  aquacore
  aqua_gait
  aqua_description 
  roscpp
  sensor_msgs
  std_msgs
  gazebo_ros
  message_runtime
  dynamic_reconfigure
  message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${aqua_gait_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(aqua_hardware_emulator src/aqua_hardware_emulator.cpp)
target_link_libraries(aqua_hardware_emulator ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(aqua_hardware_emulator aquacore_gencpp)

add_library(aqua_hydrodynamics_plugin src/aqua_hydrodynamics_plugin.cpp)
target_link_libraries(aqua_hydrodynamics_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(aqua_hydrodynamics_plugin aquacore_gencpp ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)

cmake_minimum_required(VERSION 2.8.3)
project(aqua_gait)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -Wall -O2 -Wno-unused-result")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  aquacore
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(QT5_PATH "/opt/Qt5.2.1/5.2.1/gcc_64/" CACHE STRING "Qt5 installation path (i.e. containing ./bin/qmake)")
set(Qt5Widgets_DIR "${QT5_PATH}/lib/cmake/Qt5Widgets/")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_AUTOMOC ON)
find_package(Qt5Widgets)
add_definitions(${Qt5Widgets_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}") # NOTE: double-quote needed here; not recommended below
set(QT_INCLUDE_DIRS ${Qt5Widgets_INCLUDE_DIRS})
set(QT_LIBRARIES ${Qt5Widgets_LIBRARIES})

## Generate services in the 'srv' folder
add_service_files(
  FILES
  UpdateTargetRPYD.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
#  DEPENDENCIES
#   sensor_msgs#   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS aquacore roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${Qt5Widgets_INCLUDES}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/Gaits.cpp
)

add_executable(test_gaits_term src/test_gaits_term.cpp)
target_link_libraries(test_gaits_term
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(hover_midoff_node src/hover_midoff_node.cpp)
target_link_libraries(hover_midoff_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
add_dependencies(hover_midoff_node ${catkin_EXPORTED_TARGETS})

if (${Qt5Widgets_FOUND})
  qt5_wrap_cpp(test_gaits_gui_MOC include/aqua_gait/QTestGaitsGUI.hpp)
  add_executable(test_gaits_gui src/test_gaits_gui.cpp ${test_gaits_gui_MOC})
  target_link_libraries(test_gaits_gui
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    ${QT_LIBRARIES}
  )
  add_dependencies(test_gaits_gui ${catkin_EXPORTED_TARGETS})
  qt5_use_modules(test_gaits_gui Widgets)
endif()

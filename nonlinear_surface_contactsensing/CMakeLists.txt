cmake_minimum_required(VERSION 2.8.3)
project(nonlinear_surface_contactsensing)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  actionlib_msgs
  actionlib
)
find_package(Eigen3 REQUIRED)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)

catkin_package(
  #DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
  INCLUDE_DIRS include
  #LIBRARIES #Todo
  CATKIN_DEPENDS #Todo
)

set(SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/nonlinear_surface_contactsensing.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/stl_loader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ftsensor_read_can.cpp
)
set(HEADS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/nonlinear_surface_contactsensing.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/stl_loader.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ftsensor_read_can.hpp
)


add_library(ns_contactsensing_lib ${SRCS} ${HEADS})

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(main_ftread src/main_ftsensor_read.cpp)
target_link_libraries(main_ftread
   ns_contactsensing_lib
   ${catkin_LIBRARIES}
)

add_executable(main_ns_contactsensing src/main_contactsensing.cpp)
target_link_libraries(main_ns_contactsensing
   ns_contactsensing_lib
   ${catkin_LIBRARIES}
)

#SET_TARGET_PROPERTIES(main_ftsensor_can PROPERTIES
# COMPILE_DEFINITIONS "qh_QHpointer")

#find_library(
#    QHULL
#    NAMES libqhull.so
 #   HINTS /usr/local/lib
 #   REQUIRED)
#target_link_libraries(hand_tutorial qhull ${QHULL})


#SET(CMAKE_BUILD_TYPE Release)
#SET(CMAKE_BUILD_TYPE Debug)

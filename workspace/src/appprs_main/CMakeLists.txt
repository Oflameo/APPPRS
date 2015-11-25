cmake_minimum_required(VERSION 2.8.3)
project(appprs_main)
set(CMAKE_CXX_FLAGS "-std=c++0x")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  pcl_ros
  tf
  std_msgs
  nav_msgs
  sensor_msgs
  joy
  genmsg
  laser_geometry
)

find_package( OpenCV REQUIRED )
find_package( Boost REQUIRED )

option(WITH_OPENMP  "Build with OpenMP support for multithreading"    ON)




SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

#add_service_files(
#   FILES
#     worldToRobot.srv
#     PcntrlPP.srv
#     localPP_SRV.srv
#   )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES appprs_main
  CATKIN_DEPENDS roscpp pcl_ros tf
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Declare a cpp library
 add_library(appprs_main

   src/singleparticle.cpp
   src/particlefilter.cpp
   
 )
target_link_libraries(
appprs_main  ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${boost_LIBRARIES}
)

add_executable(particle_filter_main src/particle_filter_main.cpp)
target_link_libraries(particle_filter_main
${OpenCV_LIBRARIES}  
${catkin_LIBRARIES}
${PCL_COMMON_LIBRARIES}
${PCL_IO_LIBRARIES}
${boost_LIBRARIES}
appprs_main)
add_dependencies(particle_filter_main appprs_main_generate_gencpp)






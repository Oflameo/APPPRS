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

#add_service_files(
#   FILES
#     worldToRobot.srv
#     PcntrlPP.srv
#     localPP_SRV.srv
#   )
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

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
  ${catkin_INCLUDE_DIRS},
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Declare a cpp library
 add_library(appprs_main
   circleFit/CircleFitByTaubin.cpp
 )
target_link_libraries(appprs_main
appprs_main  ${OpenCV_LIBRARIES}
             ${catkin_LIBRARIES}
)

add_executable(extract_features node/extract_features.cpp)
target_link_libraries(extract_features ${catkin_LIBRARIES}
${PCL_COMMON_LIBRARIES}
${PCL_IO_LIBRARIES})
add_dependencies(extract_features appprs_main_generate_gencpp)


add_executable(circle_tester circleFit/TestAlgebraicCircleFits.cpp)
target_link_libraries(circle_tester 
${catkin_LIBRARIES} 
${OpenCV_LIBRARIES}
)
add_dependencies(circle_tester appprs_main_generate_gencpp)


## ROBO STATS WORK:  CAN'T BUILD WITH C++11 ENABLED!!!
##add_executable(scan_to_point_cloud node/scan_to_point_cloud.cpp)
##target_link_libraries(scan_to_point_cloud ${catkin_LIBRARIES})
##add_dependencies(scan_to_point_cloud appprs_main_generate_gencpp)

##add_executable(point_cloud_filter node/point_cloud_filter.cpp)
##target_link_libraries(point_cloud_filter ${catkin_LIBRARIES}
##${PCL_COMMON_LIBRARIES}
##${PCL_IO_LIBRARIES})
##add_dependencies(point_cloud_filter appprs_main_generate_gencpp)
## END ROBO STATS WORK

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS appprs_main appprs_main_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_appprs_main.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

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


add_service_files(
   FILES
     worldToRobot.srv
     PcntrlPP.srv
#     localPP_SRV.srv
   )
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
)

# Declare a cpp library
 add_library(appprs_main
   src/GoalPositionUpdater.cpp
   src/localPP.cpp
   src/p_control_path_planner.cpp
 )
target_link_libraries(appprs_main
  ${catkin_LIBRARIES}
)

add_executable(goal_updater_node node/goal_updater_node.cpp)
target_link_libraries(goal_updater_node
  ${catkin_LIBRARIES}
  appprs_main
)


add_executable(servoTester src/servoTester.cpp)
target_link_libraries(servoTester ${catkin_LIBRARIES})
add_dependencies(servoTester appprs_main_generate_messages_cpp)

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker appprs_main_generate_messages_cpp)

add_executable(MiddleMan src/MiddleMan.cpp)
target_link_libraries(MiddleMan ${catkin_LIBRARIES})

#add_executable(PIDSteerCntrl src/PIDSteerCntrl.cpp)
#target_link_libraries(PIDSteerCntrl ${catkin_LIBRARIES})
#add_dependencies(PIDSteerCntrl appprs_main_generate_messages_cpp)

##add_executable(world_to_robot_frame src/world_to_robot_frame.cpp)
##target_link_libraries(world_to_robot_frame ${catkin_LIBRARIES})
##add_dependencies(world_to_robot_frame appprs_main_generate_gencpp)

add_executable(arduinoListener node/arduinoListener.cpp)
target_link_libraries(arduinoListener ${catkin_LIBRARIES})
add_dependencies(arduinoListener appprs_main_generate_gencpp)


add_executable(scan_to_point_cloud node/scan_to_point_cloud.cpp)
target_link_libraries(scan_to_point_cloud ${catkin_LIBRARIES})
add_dependencies(scan_to_point_cloud appprs_main_generate_gencpp)


add_executable(point_cloud_filter node/point_cloud_filter.cpp)
target_link_libraries(point_cloud_filter ${catkin_LIBRARIES}
${PCL_COMMON_LIBRARIES}
${PCL_IO_LIBRARIES})
add_dependencies(point_cloud_filter appprs_main_generate_gencpp)

#add_executable(P_control_path_planner src/P_control_path_planner.cpp)
#target_link_libraries(P_control_path_planner ${catkin_LIBRARIES})
#add_dependencies(P_control_path_planner appprs_main_gencpp)

#add_executable(localPP src/localPP.cpp)
#target_link_libraries(localPP ${catkin_LIBRARIES})
#add_dependencies(localPP appprs_main_gencpp)





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

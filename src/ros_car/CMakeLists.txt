cmake_minimum_required(VERSION 2.8.3)
project(ros_car)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  gazebo_plugins
  std_msgs
  geometry_msgs  
  sensor_msgs
  joy
  tf
  nav_msgs
  ackermann_msgs
  actionlib
  move_base_msgs
  visualization_msgs
  yocs_msgs
  yocs_math_toolkit
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

pkg_search_module(yaml-cpp REQUIRED yaml-cpp)
if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif()

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${yaml-cpp_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_car_gazebo
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
# )

catkin_package(
# INCLUDE_DIRS include
# DEPENDS 
#	  roscpp
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(ros_car_gazebo
#   src/${PROJECT_NAME}/ros_car_gazebo.cpp
# )

add_library(ros_car_drive_plugin src/gazebo_ros_car_drive.cpp)
add_library(waypoint_provider_lib src/waypoint_provider.cpp)
add_library(waypoint_provider_yaml_parser_lib src/yaml_parser.cpp)

## Declare a C++ executable
# add_executable(ros_car_gazebo_node src/ros_car_gazebo_node.cpp)

add_executable(ros_car_teleop_node src/ros_car_teleop.cpp)
add_executable(ros_car_odometry_publisher_node src/ros_car_odometry_publisher.cpp)
add_executable(ros_car_ackermann_publisher_node src/ros_car_ackermann_publisher.cpp)
add_executable(waypoint_provider src/waypoint_provider_main.cpp)
add_executable(waypoints_navi_node src/waypoints_navi.cpp src/waypoints_navi_main.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(ros_car_gazebo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(waypoint_provider_lib yocs_msgs_gencpp)
add_dependencies(waypoint_provider_yaml_parser_lib yocs_msgs_gencpp)
add_dependencies(waypoint_provider waypoint_provider_lib waypoint_provider_yaml_parser_lib)
add_dependencies(waypoints_navi_node yocs_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(ros_car_gazebo_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(ros_car_drive_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(ros_car_teleop_node ${catkin_LIBRARIES})
target_link_libraries(ros_car_odometry_publisher_node ${catkin_LIBRARIES})
target_link_libraries(ros_car_ackermann_publisher_node ${catkin_LIBRARIES})
target_link_libraries(waypoint_provider_lib ${catkin_LIBRARIES})
target_link_libraries(waypoint_provider_yaml_parser_lib ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES})
target_link_libraries(waypoint_provider waypoint_provider_lib waypoint_provider_yaml_parser_lib)
target_link_libraries(waypoints_navi_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#set_target_properties(ros_car_arduino_odom_msg_node PROPERTIES LINKER_LANGUAGE CXX)

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
# install(TARGETS ros_car_gazebo ros_car_gazebo_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ros_car_gazebo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

cmake_minimum_required(VERSION 3.0.2)
project(girona_utils)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(CMAKE_CXX_STANDARD 11)
#   set(CMAKE_CXX_STANDARD_REQUIRED ON)
#   set(CMAKE_CXX_EXTENSIONS OFF)
#   # this next line shouldn't be necessary, but doesn't always get added by cmake (e.g., for clang++-5)
#   add_definitions(-std=c++11)

set(CMAKE_MODULE_PATH
  "${CMAKE_MODULE_PATH}"
  "${CMAKE_CURRENT_SOURCE_DIR}")
  
include(macros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(fcl REQUIRED)
message("fcl prints")
# message("1 ${fcl_FOUND}")
# message("2 ${FCL_INCLUDE_DIRS}")
# message("3 ${FCL_LIBRARY_DIRS}")
# message("4 ${FCL_LIBRARIES}")
# message("5 ${FCL_VERSION}")

list(APPEND CATKIN_SOURCES
    roscpp
    rospy
    nav_msgs
    tf
    tf2
    tf2_ros
    eigen_conversions
    message_filters
    cola2_msgs
    std_msgs
    geometry_msgs
    octomap_msgs
    octomap_ros
    rviz_visual_tools
    iauv_kinematic_control
  )

find_package(catkin REQUIRED COMPONENTS ${CATKIN_SOURCES} message_generation actionlib)

catkin_python_setup()

add_message_files(
  FILES
  # PathStatus.msg
)

# add_service_files(
#   FILES
# )

add_action_files(
  DIRECTORY action 
  FILES 
  PID.action
  Pursuit.action)

generate_messages(
 DEPENDENCIES
 geometry_msgs
 std_msgs
 actionlib_msgs
 nav_msgs
)

catkin_package(
  LIBRARIES ${LIBRARY_NAME}
  CATKIN_DEPENDS ${CATKIN_SOURCES}
  DEPENDS
  INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
  )

###########s
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

list(APPEND INCLUDES
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
  )

include_directories(SYSTEM ${INCLUDES})

list(APPEND LIBRARIES
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${EIGEN_LIBRARIES}
    ${FCL_LIBRARIES}
)


add_script(pid_action scripts)
add_script(pursuit_controller scripts)
# add_script(path_following_server scripts)
# add_script(path_following_server_tp scripts)
# add_script(odom2tf scripts/utils)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination


catkin_install_python(PROGRAMS
  # scripts/utils/path_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

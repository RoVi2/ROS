cmake_minimum_required(VERSION 2.8.3)
project(path_planning)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
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
add_service_files(
	FILES
	addToQueue.srv 
	clearJointQueue.srv 
	getJointConfig.srv
	setJointConfig.srv
	startPA10.srv
	startQueue.srv
)

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
#  LIBRARIES path_planning
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# We use the settings that robwork studio uses
SET(CMAKE_BUILD_TYPE Release)
SET(RW_ROOT $ENV{RW_ROOT})
SET(RWSTUDIO_ROOT "$ENV{RWS_ROOT}")

# Use RobWork
SET(RobWork_DIR ${RW_ROOT}/cmake) 
FIND_PACKAGE(RobWork REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

#Include default settings for constructing a robwork dependent project
FIND_PACKAGE(
  RobWork
  QUIET
  HINTS
    "${RW_ROOT}")
#The following is a workaround for the old version of RobWork. Remove once no longer needed (everyone switched to a RobWork version newer than April 2012)
IF(NOT ROBWORK_FOUND)
  MESSAGE(STATUS "Could not find RobWork with the new method. Trying the old one.")
  INCLUDE(${RW_ROOT}/build/FindRobWork.cmake)
ENDIF(NOT ROBWORK_FOUND)

#Include default settings for constructing a robworkst dependent project
FIND_PACKAGE(
  RobWorkStudio
  QUIET
  HINTS
    "${RWSTUDIO_ROOT}")
#The following is a workaround for the old version of RobWorkSt. Remove once no longer needed (everyone switched to a RobWork version newer than April 2012)
IF(NOT ROBWORKSTUDIO_FOUND)
  MESSAGE(STATUS "Could not find RobWorkStudio with the new method. Trying the old one.")
  INCLUDE(${RWSTUDIO_ROOT}/build/FindRobWorkStudio.cmake)
ENDIF(NOT ROBWORKSTUDIO_FOUND)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} ${ROBWORKSTUDIO_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} ${ROBWORKSTUDIO_LIBRARY_DIRS} ${catkin_LIBRARIES})

# if we want to use ui files add them here
SET(UIS_FILES src/SampleWidget.ui )

QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})

QT4_WRAP_CPP(MocSrcFiles src/SamplePlugin.hpp src/SampleWidget.hpp
OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED
)
# Rcc the files:

#QT4_WRAP_CPP(sources $

#{moc-sources}
#OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)

#QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
set(SrcFiles
	src/path_planning_node.cpp
)


set(ROS_BUILD_STATIC_LIBS false)
set(ROS_BUILD_SHARED_LIBS true)

## Declare a cpp executable
add_executable(path_planning_node src/path_planning_node.cpp)
add_dependencies(path_planning_node path_planning_generate_messages_cpp)
target_link_libraries(path_planning_node ${catkin_LIBRARIES} ${ROBWORK_LIBRARIES})

list(APPEND CMAKE_CXX_FLAGS "-std=c++0x  -g -O3 -ftest-coverage -fprofile-arcs")
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes


## Specify libraries to link a library or executable target against


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
# install(TARGETS path_planning path_planning_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_path_planning.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

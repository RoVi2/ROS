cmake_minimum_required(VERSION 2.8)
project(pa10_plugin)

SET(RW_ROOT "$ENV{RW_ROOT}")
SET(RWSTUDIO_ROOT "$ENV{RWS_ROOT}")

# Find packages.
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    pa10_dummy
)

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

catkin_package()

# Search directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(${catkin_INCLUDE_DIRS} ${ROBWORKSTUDIO_INCLUDE_DIRS} ${ROBWORK_INCLUDE_DIRS})
link_directories(${ROBWORK_LIBRARY_DIRS} ${ROBWORKSTUDIO_LIBRARY_DIRS})

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall") # -pedantic
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2") # -Wextra -Werror

# Files
set(sources src/pa10plugin.cpp
            src/rosnodethread.cpp)
set(headers src/pa10plugin.h
            src/rosnodethread.h)
set(forms ui/pa10plugin.ui)
# set(resources ${CMAKE_CURRENT_SOURCE_DIR}/resources/*.qrc)

# Qt4 pre-processing
QT4_WRAP_CPP(headers_moc ${headers})
QT4_WRAP_UI(forms_headers ${forms})
# QT4_ADD_RESOURCES(resources_rcc ${resources})

# Plugin target
add_library(${PROJECT_NAME} MODULE ${sources} ${headers} ${headers_moc} ${forms_headers} ${resources_rcc})
add_dependencies(${PROJECT_NAME} pa10_dummy)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${ROBWORK_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})
# install(TARGETS ros_rws_plugin RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

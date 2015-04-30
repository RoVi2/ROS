cmake_minimum_required(VERSION 2.8)
project(pa10_plugin)

# Find libraries.
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
)
find_package(RobWork REQUIRED)
find_package(RobWorkStudio REQUIRED)

# ROS service files.
add_service_files(FILES
    addToQueue.srv
    clearJointQueue.srv
    getJointConfig.srv
    setJointConfig.srv
    startPA10.srv
    startQueue.srv
)

# Generate added messages and services.
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

# Search directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(${catkin_INCLUDE_DIRS} ${ROBWORKSTUDIO_INCLUDE_DIRS} ${ROBWORK_INCLUDE_DIRS})
link_directories(${ROBWORK_LIBRARY_DIRS} ${ROBWORKSTUDIO_LIBRARY_DIRS})

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall") # -pedantic
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2") # -Wextra -Werror

# Files
set(sources ${CMAKE_CURRENT_SOURCE_DIR}/src/pa10plugin.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/nodethread.cpp)
set(headers ${CMAKE_CURRENT_SOURCE_DIR}/src/pa10plugin.h
            ${CMAKE_CURRENT_SOURCE_DIR}/src/nodethread.h)
set(forms ${CMAKE_CURRENT_SOURCE_DIR}/ui/pa10plugin.ui)
# set(resources ${CMAKE_CURRENT_SOURCE_DIR}/resources/*.qrc)

# Qt4 pre-processing
QT4_WRAP_CPP(headers_moc ${headers})
QT4_WRAP_UI(forms_headers ${forms})
# QT4_ADD_RESOURCES(resources_rcc ${resources})

# Plugin target
add_library(pa10_plugin MODULE ${sources} ${headers} ${headers_moc} ${forms_headers} ${resources_rcc})
add_dependencies(pa10_plugin pa10_plugin_generate_messages_cpp)
target_link_libraries(pa10_plugin ${QT_LIBRARIES} ${catkin_LIBRARIES} ${ROBWORK_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})
# install(TARGETS ros_rws_plugin RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

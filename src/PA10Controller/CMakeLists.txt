cmake_minimum_required(VERSION 2.8.3)
project(pa10controller)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS serial roscpp rospy std_msgs message_generation)

## Declare ROS messages and services
add_service_files(
	FILES 
	addToQueue.srv 
	clearJointQueue.srv 
	getJointConfig.srv 
	setJointConfig.srv 
	startPA10.srv 
	startQueue.srv
)

include_directories(include ${catkin_INCLUDE_DIRS})
set(LibraryList pthread)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(INCLUDE_DIRS)

# The shared library to build:
add_executable(pa10_client src/pa10_client.cpp)
target_link_libraries(pa10_client ${catkin_LIBRARIES})
add_dependencies(pa10_client system thread)

add_executable(pa10_server src/pa10_server.cpp)
target_link_libraries(pa10_server serial boost_program_options ${catkin_LIBRARIES})
add_dependencies(pa10_server system thread serial)

# %EndTag(FULLTEXT)%

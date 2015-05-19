cmake_minimum_required(VERSION 2.8.3)
project(pa10controller)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS serial roscpp rospy std_msgs message_generation)

find_package(Boost COMPONENTS thread program_options REQUIRED)

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

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

# The shared library to build:
add_executable(pa10_client src/pa10_client.cpp)
add_dependencies(pa10_client pa10controller_generate_messages_cpp)
target_link_libraries(pa10_client ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pa10_server src/pa10_server.cpp)
add_dependencies(pa10_server pa10controller_generate_messages_cpp)
target_link_libraries(pa10_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})

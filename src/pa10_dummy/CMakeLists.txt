cmake_minimum_required(VERSION 2.8.3)
project(pa10_dummy)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    serial
    std_msgs
    message_generation
)

find_package(Boost COMPONENTS thread program_options REQUIRED)

# Declare ROS messages and services
add_service_files(
    FILES
    addToQueue.srv
    clearJointQueue.srv
    getJointConfig.srv
    setJointConfig.srv
    startPA10.srv
    startQueue.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

add_executable(pa10_dummy src/pa10_server.cpp)
add_dependencies(pa10_dummy serial pa10_dummy_generate_messages_cpp)
target_link_libraries(pa10_dummy ${catkin_LIBRARIES} ${Boost_LIBRARIES})

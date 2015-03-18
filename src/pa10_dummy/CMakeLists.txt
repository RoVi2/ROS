# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(pa10_dummy)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  message_generation
)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
add_service_files(
FILES
addToQueue.srv 
clearJointQueue.srv 
getJointConfig.srv
setJointConfig.srv
startPA10.srv
startQueue.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## Generate added messages and services
#catkin_package(
#   INCLUDE_DIRS include
#  CATKIN_DEPENDS message_runtime)


catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


## Declare a catkin package


include_directories( ${catkin_INCLUDE_DIRS})

#### rx60 driver ###
#set(SrcFiles
#  src/rx60_controller/RX60Driver.cpp
#  src/rx60_controller/RX60TCPSocket.cpp)

#link_directories( )

set(LibraryList
  pthread)
#  curses)

# The shared library to build:

# Link the standard static libraries with with the shared library:
#target_link_libraries(RX60 ${LibraryList})


#add_executable(pa10_client src/pa10_client.cpp)
#target_link_libraries(pa10_client ${catkin_LIBRARIES})
#add_dependencies(pa10_client system thread)

add_executable(pa10_dummy src/pa10_server.cpp)
add_dependencies(pa10_dummy pa10_dummy_generate_messages_cpp)
target_link_libraries(pa10_dummy serial boost_program_options ${catkin_LIBRARIES})
add_dependencies(pa10_dummy system thread serial)

#target_link_libraries(talker ${catkin_LIBRARIES})
#add_executable(test-RX60 src/rx60_controller/test.cpp ${SrcFiles})
#target_link_libraries(test-RX60 ${catkin_LIBRARIES})
#add_dependencies(test-RX60 system thread)

#add_executable(nodetest-RX60 src/rx60_controller/nodetest.cpp ${SrcFiles})
#target_link_libraries(nodetest-RX60 ${catkin_LIBRARIES})
#add_dependencies(nodetest-RX60 system thread)

### end of rx60 driver ###

# %EndTag(FULLTEXT)%

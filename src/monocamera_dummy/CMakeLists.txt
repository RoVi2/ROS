# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(monocamera_dummy)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
 sensor_msgs
  cv_bridge
  roscpp
  rospy
  std_msgs
  image_transport
  message_generation
)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(
#FILES
#)

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

#link_directories( )

#set(LibraryList
	#  pthread)
#  curses)

add_executable(monocamera_dummy src/monocamera_dummy.cpp)
target_link_libraries(monocamera_dummy ${catkin_LIBRARIES})
#add_dependencies(monocamera_dummy)

# %EndTag(FULLTEXT)%

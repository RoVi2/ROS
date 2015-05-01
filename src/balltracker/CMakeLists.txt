cmake_minimum_required(VERSION 2.8.3)
project(balltracker)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    cv_bridge
    image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES balltracker
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -Wextra -Werror")

add_executable(balltracker_node src/balltracker_node.cpp)

target_link_libraries(balltracker_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

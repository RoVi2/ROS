cmake_minimum_required(VERSION 2.8)
project(balltracker)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    image_transport
)

find_package(Boost COMPONENTS system thread timer REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -Wextra -Werror")

set(SOURCES
    src/balltracker_node.cpp
    src/syncedimages.cpp
)

set(HEADERS
    src/syncedimages.h
)

add_executable(balltracker_node
    ${SOURCES}
    ${HEADERS}
)

target_link_libraries(balltracker_node
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

cmake_minimum_required(VERSION 2.8.9)
project(timesync)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure roscpp)
set(CMAKE_CXX_FLAGS "-std=c++14")

set(timesync_SRCS
        src/HoltWinters.cpp
        src/TimestampSynchronizer.cpp)

# Declare this project as a catkin package
catkin_package(CATKIN_DEPENDS dynamic_reconfigure roscpp
               INCLUDE_DIRS include
               LIBRARIES timesync)

include_directories(include/timesync ${catkin_INCLUDE_DIRS})

# Define a library target called hello_world
add_library(timesync ${timesync_SRCS})
target_link_libraries(timesync ${catkin_LIBRARIES})

add_executable(example src/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES} timesync)

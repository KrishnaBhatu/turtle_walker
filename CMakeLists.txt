cmake_minimum_required(VERSION 2.8.3)
project(turtle_walker)

## For compiling c++11 functions
add_compile_options(-std=c++11)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
## Find catkin macros and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs move_base_msgs)

## Declare catkin package 
catkin_package()

include_directories( include ${catkin_INCLUDE_DIRS})

add_executable(goTurtle src/main.cpp src/roomba.cpp)
target_link_libraries(goTurtle ${catkin_LIBRARIES})


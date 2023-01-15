cmake_minimum_required(VERSION 3.16.3)
project(ros_motion_planning)

find_package(roscpp)
#set(CMAKE_CXX_STANDARD 11)

include_directories(include src/motion_planning_library/src/cpp/geometry/include/)
include_directories(${roscpp_INCLUDE_DIRS})

add_subdirectory(src)

#add_library(visualizer
#        src/visualizer/ROS_transform_tree_visualizer.cpp
#  )

#target_link_libraries(visualizer PUBLIC ${roscpp_LIBRARIES} geometry)
#target_include_directories(visualizer PUBLIC /opt/ros/noetic/include) # Allows items linking to this library to find the headers for ROS items.

#add_subdirectory(tests)

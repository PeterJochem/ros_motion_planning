cmake_minimum_required(VERSION 3.16.3)
project(ros_motion_planning)
set(CMAKE_CXX_STANDARD 20)

find_package(roscpp)
#find_package(geometry)
#include_directories(${geometry_INCLUDE_DIRS}
set(geometry_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src/motion_planning_library/src/cpp/geometry/include/" CACHE INTERNAL "")

include_directories(include src/motion_planning_library/src/cpp/geometry/include/)
include_directories(include src/motion_planning_library/src/cpp/robot/include/)
include_directories(include src/motion_planning_library/src/cpp/collision_checking/include/)
include_directories(include src/motion_planning_library/src/cpp/planning/include/)
include_directories(include src/ros_conversions/include/)
include_directories(${roscpp_INCLUDE_DIRS})

add_subdirectory(src)
add_subdirectory(tests)

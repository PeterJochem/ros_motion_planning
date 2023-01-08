cmake_minimum_required(VERSION 3.16.3)
project(motion_planning_library)

#set(CMAKE_CXX_STANDARD 11)

add_subdirectory(src/cpp)

#add_executable(visually_inspect_transform_tree visually_inspect_static_transform_tree.cpp)

#target_link_libraries(
#    visually_inspect_transform_tree
#    geometry
#)
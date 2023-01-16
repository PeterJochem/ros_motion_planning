


add_executable(a visually_inspect_static_transform_tree.cpp)
target_include_directories(a PUBLIC /opt/ros/noetic/include)
#target_link_libraries(visually_inspect_transform_tree ${roscpp_LIBRARIES} visualizer geometry)
target_link_libraries(a visualizer ${roscpp_LIBRARIES} ros_conversions)

#/home/pj/ros_motion_planning/build/src/ros_conversions_1/libros_conversions.a)



add_library(visualizer
        ROS_transform_tree_visualizer.cpp
)

#find_package(ros_conversions)
#include_directories(${ros_conversions_INCLUDE_DIRS}
if(NOT DEFINED ${ros_conversions_INCLUDE_DIRS})
  set(ros_conversions_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../ros_conversions/include" CACHE INTERNAL "")
endif(NOT DEFINED ${ros_conversions_INCLUDE_DIRS})

include_directories(${ros_conversions_INCLUDE_DIRS})

target_link_libraries(visualizer PUBLIC ${roscpp_LIBRARIES} geometry)
target_include_directories(visualizer PUBLIC /opt/ros/noetic/include)

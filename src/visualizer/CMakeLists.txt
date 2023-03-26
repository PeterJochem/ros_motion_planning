


add_library(visualizer
        ROS_transform_tree_visualizer.cpp
        ROS_robot_visualizer.cpp
        ROS_mesh_visualizer.cpp
        ROS_robot_trajectory_visualizer.cpp
)

#find_package(ros_conversions)
#include_directories(${ros_conversions_INCLUDE_DIRS}
if(NOT DEFINED ${ros_conversions_INCLUDE_DIRS})
  set(ros_conversions_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../ros_conversions/include" CACHE INTERNAL "")
endif(NOT DEFINED ${ros_conversions_INCLUDE_DIRS})


if(NOT DEFINED ${planning_INCLUDE_DIRS})
  set(planning_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../motion_planning_library/src/cpp/planning/include/" CACHE INTERNAL "")
endif(NOT DEFINED ${planning_INCLUDE_DIRS})

include_directories(${ros_conversions_INCLUDE_DIRS})

target_link_libraries(visualizer PUBLIC ${roscpp_LIBRARIES} geometry robot)
target_include_directories(visualizer PUBLIC /opt/ros/noetic/include)

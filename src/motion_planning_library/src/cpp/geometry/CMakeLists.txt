include_directories(include)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GTest REQUIRED)
find_package(roscpp)
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${roscpp_INCLUDE_DIRS})



add_library(geometry
  src/transform/transform.cpp 
  src/transform/utilities.cpp  
  include/transform/node.hpp 
  src/transform/static_transform_tree.cpp
  src/transform/transform_tree.cpp
  src/primitives/vector.cpp
  src/primitives/axis.cpp
  src/transform/frame.cpp
  )

target_link_libraries(geometry PUBLIC Eigen3::Eigen PUBLIC ${roscpp_LIBRARIES})

# Test
target_include_directories(geometry PUBLIC ${EIGEN3_INCLUDE_DIR} ) 


target_include_directories(geometry PUBLIC /opt/ros/noetic/include) # Allows items linking to this library to find the headers for ROS items.

set(BUILD_TESTS True)
if (BUILD_TESTS)
  add_subdirectory(tests)
endif()

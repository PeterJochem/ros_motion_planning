
find_package(GTest REQUIRED)

set(test_sources 
    test_transform.cpp 
    test_node.cpp 
    test_transform_tree.cpp
    test_utilities.cpp
    test_static_transform_tree.cpp
)

add_executable(test_geometry_library ${test_sources})


target_link_libraries(
    test_geometry_library
    ${GTEST_LIBRARIES} 
    ${GTEST_MAIN_LIBRARIES} 
    pthread
    geometry
)

include(GoogleTest)
gtest_discover_tests(test_geometry_library)

include_directories(include)
#find_package(geometry)


add_library(robot
            src/robot.cpp 
            src/UR5.cpp
            src/joint.cpp 
            src/link.cpp
            )


include_directories(../geometry/include)
target_link_libraries(robot PUBLIC geometry)
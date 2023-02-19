#include <iostream>
#include "visualizer/ROS_transform_tree_visualizer.hpp"
#include "transform/static_transform_tree.hpp"
#include "robots/UR5.hpp"
#include "visualizer/ROS_robot_visualizer.hpp"



void simple_tree() { 

    using namespace geometry;

    Frame root = Frame("world");
    StaticTransformTree tree = StaticTransformTree(root);

    Frame a = Frame("a");
    Frame b = Frame("b");
    Frame c = Frame("c");

    Transform root_to_a = Transform(root, a, 1., 0., 0., 0., 0., 0.);
    Transform a_to_b = Transform(a, b, 2., 0., 0., 0., 0., 0.);
    Transform b_to_c = Transform(b, c, 3., 0., 0., 0., 0., 0.);

    tree.add(root_to_a);
    tree.add(a_to_b);
    tree.add(b_to_c);

    
    ROSTransformTreeVisualizer visualizer = ROSTransformTreeVisualizer(&tree, "/tf");

    visualizer.visualize();
    
    ros::spin();
}

void visualize_UR5() {

    using namespace geometry;

    // Create UR5
    // Put all the frames of the UR5 into the transform tree.
    Robot::UR_5 my_robot = Robot::UR_5();
    
    ROSRobotVisualizer visualizer = ROSRobotVisualizer(&my_robot, "/tf");

    visualizer.visualize();

    int i  = 0;
    while (true) {

         visualizer.set_joint_angles({i/5., 0., 0., 0., 0., 0.});
         i++;
         sleep(1.5);
    }

    ros::spin();
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "visualizer");
    visualize_UR5();
    return 0;
}
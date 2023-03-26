#include <iostream>
#include "visualizer/ROS_transform_tree_visualizer.hpp"
#include "transform/static_transform_tree.hpp"
#include "robots/UR5.hpp"
#include "visualizer/ROS_robot_visualizer.hpp"
#include "collision_checking/fcl_robot_internal_collision_checker.hpp"


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

    
    ROSTransformTreeVisualizer visualizer = ROSTransformTreeVisualizer(&tree, "/tf", 1.);

    visualizer.visualize();
    
    ros::spin();
}

void visualize_UR5() {

    using namespace geometry;

    // Create UR5
    // Put all the frames of the UR5 into the transform tree.
    Robot::UR_5 my_robot = Robot::UR_5();

    // construct transform tree
    Frame root = Frame("world");
    StaticTransformTree tree = StaticTransformTree(root);

    auto transforms = my_robot.get_ordered_transforms();
    for (int i = 0; i < transforms.size(); i++) {
        tree.add(transforms[i]);
    }

    FCLRobotInternalCollisionChecker checker = FCLRobotInternalCollisionChecker(my_robot, &tree);
    ROSRobotVisualizer visualizer = ROSRobotVisualizer(&my_robot, "/tf", 1.);

    visualizer.visualize();
    int i  = 0;

    while (true) {

        //visualizer.set_joint_angles({0., 0., 0., i/5., i/5., 0.});
        visualizer.set_joint_angles({0., -0.5, -1., 0., 1.57, 0.0});

         // need to update the TransformTree
         auto transforms = my_robot.get_ordered_transforms();
         for (int i = 0; i < transforms.size(); i++) {
            
            auto transform = transforms[i];
            tree.set_transform(transform, transform.getPosition(), transform.getEulerAngles());
         }
         
         
         i++;
         if (checker.check()) {
          //  std::cout << "\n\nCollision \n\n" << std::endl;
         }
         else {
          //  std::cout << "No collision" << std::endl;
         }
    }

    ros::spin();
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "visualizer");
    visualize_UR5();
    return 0;
}
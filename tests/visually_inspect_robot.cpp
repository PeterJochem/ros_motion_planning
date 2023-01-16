#include <iostream>>

#include "visualizer/ROS_transform_tree_visualizer.hpp"
#include "transform/static_transform_tree.hpp"



int main(int argc, char** argv) {

    std::cout << "Hello World!" << std::endl;

    using namespace geometry;


    ros::init(argc, argv, "my_node_name");


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


    return 0;
}
#include "visualizer/ROS_robot_visualizer.hpp"



geometry::ROSRobotVisualizer::ROSRobotVisualizer() {

}


geometry::ROSRobotVisualizer::ROSRobotVisualizer(Robot::Robot1* robot, std::string topic_name, float publishing_rate) {

    this->robot = robot;
    this->topic_name = topic_name;
    this->publishing_rate = publishing_rate;

    Frame root = Frame("world");
    this->tree = new StaticTransformTree(root);
    
    std::vector<geometry::Transform> transforms = robot->get_ordered_transforms();
    for (std::vector<geometry::Transform>::iterator itr = transforms.begin(); itr != transforms.end(); itr++) {
        this->tree->add(*itr);
    }

    this->tree_visualizer = new ROSTransformTreeVisualizer(tree, "/tf", publishing_rate);
    this->mesh_visualizer = new ROSMeshVisualizer(this->robot->get_links(), "/robot_meshes", publishing_rate);
}


void geometry::ROSRobotVisualizer::visualize() {

    this->transform_tree_publishing_thread = new std::thread(&ROSTransformTreeVisualizer::visualize, this->tree_visualizer);
    this->mesh_publishing_thread = new std::thread(&ROSMeshVisualizer::visualize, this->mesh_visualizer);
}

void geometry::ROSRobotVisualizer::set_joint_angles(std::vector<float> angles) {

    robot->set_joint_angles(angles);

    std::vector<geometry::Transform> transforms = robot->get_ordered_transforms();
    for (std::vector<geometry::Transform>::iterator itr = transforms.begin(); itr != transforms.end(); itr++) {
        this->tree->set_transform(*itr, itr->getPosition(), itr->getEulerAngles());
    }
}
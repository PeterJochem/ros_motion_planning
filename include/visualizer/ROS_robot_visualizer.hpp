#ifndef ROS_ROBOT_VISUALIZER
#define ROS_ROBOT_VISUALIZER
#include <vector>
#include <thread>
#include <ctime>
#include <thread> 
#include "ros/ros.h"
#include "visualizer/ROS_transform_tree_visualizer.hpp"
#include "visualizer/ROS_mesh_visualizer.hpp"
#include "robots/robot.hpp"
#include "transform/static_transform_tree.hpp"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace geometry {

class ROSRobotVisualizer {

    public:
        ROSRobotVisualizer();
        ROSRobotVisualizer(Robot::Robot1*, std::string topic_name, float publishing_rate);
        void visualize();
        void stop_visualization();
        void set_joint_angles(std::vector<float> angles);
        void publish_meshes();

    private:
        std::string topic_name;
        std::thread* transform_tree_publishing_thread;
        std::thread* mesh_publishing_thread;
        Robot::Robot1* robot;
        geometry::ROSTransformTreeVisualizer* tree_visualizer;
        geometry::ROSMeshVisualizer* mesh_visualizer;
        float publishing_rate;
        StaticTransformTree* tree;
        void publish();
        void stop_publishing();
};
}
#endif
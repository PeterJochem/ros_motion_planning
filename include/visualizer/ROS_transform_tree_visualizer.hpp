#ifndef ROS_TRANSFORM_VISUALIZER
#define ROS_TRANSFORM_VISUALIZER
#include <vector>
#include <thread>
#include <ctime>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf2_msgs/TFMessage.h"
#include "transform/transform_tree_visualizer.hpp"


namespace geometry {

class ROSTransformTreeVisualizer: TransformTreeVisualizer {

    public:
        ROSTransformTreeVisualizer(TransformTree* tree, std::string topic_name, float publishing_rate);
        void visualize();
        void stop_visualization();

    private:
        std::string topic_name;
        std::thread publishing_thread;
        float publishing_rate;
        void publish();
        void stop_publishing();
};
}
#endif
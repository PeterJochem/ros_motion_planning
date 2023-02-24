#ifndef ROS_MESH_VISUALIZER
#define ROS_MESH_VISUALIZER
#include <vector>
#include <thread>
#include <ctime>
#include "robots/robot.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"


namespace geometry {

class ROSMeshVisualizer {

    public:
        ROSMeshVisualizer(std::vector<Robot::Link>, std::string topic_name);
        void visualize();
        void stop_visualization();

    private:
        std::string topic_name;
        std::vector<Robot::Link> links;
        std::thread publishing_thread; 
        void publish();
        void stop_publishing();
};
}
#endif
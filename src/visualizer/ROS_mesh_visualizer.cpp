#include "visualizer/ROS_mesh_visualizer.hpp"
#include "ros_conversions/ros_conversions.hpp"


geometry::ROSMeshVisualizer::ROSMeshVisualizer(std::vector<Robot::Link> links, std::string topic_name, float publishing_rate): links(links), topic_name(topic_name), publishing_rate(publishing_rate) {

}

void geometry::ROSMeshVisualizer::visualize() {

    ros::NodeHandle nh;
    ros::Publisher publisher = nh.advertise<visualization_msgs::MarkerArray>(topic_name, 1);

    visualization_msgs::MarkerArray message;        
        
    while (true) {

        // Remove the old message
        message.markers.clear();
        auto marker = visualization_msgs::Marker();
        marker.action = marker.DELETEALL;
        message.markers.push_back(marker);
        publisher.publish(message);
        message.markers.clear();

        for (int i = 0; i < links.size(); i++) {
            
            auto marker = ros_conversions::to_ros(links[i].get_visual_mesh());
            message.markers.push_back(marker);
        }
        
        publisher.publish(message);
        sleep(publishing_rate);
    }
}
#include "visualizer/ROS_transform_tree_visualizer.hpp"
#include "ros_conversions_1/conversions.hpp"

namespace geometry {

    ROSTransformTreeVisualizer::ROSTransformTreeVisualizer(TransformTree* tree, std::string topic_name): topic_name(topic_name) {
        this->tree = tree;
    }

    void ROSTransformTreeVisualizer::visualize() {

        // Example of publishing a list of items in cpp
        // https://www.theconstructsim.com/ros-qa-045-publish-subscribe-array-vector-message/

        ros::NodeHandle nh;
        ros::Publisher pub = nh.advertise<tf2_msgs::TFMessage>(topic_name, 1);

        tf2_msgs::TFMessage message;        

        float i = 0.;
        while (true) {
            
            std::vector<Transform> transforms = tree->all_transforms();
            for (int i = 0; i < transforms.size(); i++) {

                // FIX ME - need to use conversions method to do this now
                message.transforms.push_back( conversions::stamp_and_to_ros(transforms[i]));
            }
            
            pub.publish(message);
            
            message.transforms.clear();
            sleep(1.5);
            tree->set_transform(transforms[1], (i/10.0), 0., 0., 0., 0., 0.);
            i++;
        }
        

    }

    void ROSTransformTreeVisualizer::stop_visualization() {
    }
    
    void ROSTransformTreeVisualizer::publish() {
        // Start a thread to publish at some default rate
    }

    void ROSTransformTreeVisualizer::stop_publishing() {
        // Start a thread to publish at some default rate
    }

}

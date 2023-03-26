#ifndef ROS_ROBOT_TRAJECTORY_VISUALIZER
#define ROS_ROBOT_TRAJECTORY_VISUALIZER
#include <vector>
#include <thread>
#include <ctime>
#include <thread>
#include <chrono>
#include "ros/ros.h"
#include "visualizer/ROS_robot_visualizer.hpp"
#include "planning/trajectory.hpp"


namespace geometry {

class ROSRobotTrajectoryVisualizer {

    public:
        ROSRobotTrajectoryVisualizer(Robot::Robot1*, planning::Trajectory trajectory, std::string topic_name, float publishing_rate);
        void visualize();
        void stop_visualization();

    private:
        ROSRobotVisualizer robot_visualizer;
        planning::Trajectory trajectory;
        std::thread* robot_state_publishing_thread;
        std::thread* robot_state_updating_thread;
        //void publish();
        //void stop_publishing();
};
}
#endif
#include "visualizer/ROS_robot_trajectory_visualizer.hpp"

namespace geometry {

    ROSRobotTrajectoryVisualizer::ROSRobotTrajectoryVisualizer(Robot::Robot1* robot, planning::Trajectory trajectory, std::string topic_name, float publishing_rate): trajectory(trajectory) {

        robot_visualizer = ROSRobotVisualizer(robot, topic_name, publishing_rate);
    }
        

    void ROSRobotTrajectoryVisualizer::visualize() {

        // create another thread to set the joint angles.
        // set_joint_angles(std::vector<float> angles);
        this->robot_state_publishing_thread = new std::thread(&ROSRobotVisualizer::visualize, this->robot_visualizer);
        //robot_state_updating_thread = 

        while (true) {
            auto path = trajectory.get_path();
            auto joint_angles = path.get_robot_states();
            auto schedule = trajectory.get_schedule();
            for (int i = 0; i < path.size(); i++) {
                robot_visualizer.set_joint_angles(joint_angles[i].get_configuration());
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                std::cout << 100 * (i/float(path.size())) << "%" << std::endl; 
            }
        }
    }

    void ROSRobotTrajectoryVisualizer::stop_visualization() {
        //robot_visualizer.stop_visualization();
    }

}
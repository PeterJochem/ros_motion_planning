#ifndef TRANSFORM
#define TRANSFORM
#include <ostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <chrono>
#include <unistd.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/Transform.h"
#include "transform/utilities.hpp"
#include "transform/transform.hpp"
#include "transform/frame.hpp"
#include "primitives/vector.hpp"


namespace geometry {

    class Transform {
        public:
            Transform(Frame parent, Frame child, float x, float y, float z, float roll, float pitch, float yaw);
            Transform(Frame parent, Frame child, Vector3D position, Vector3D euler_angles);
            Transform(Frame parent, Frame child, Eigen::Matrix4d matrix);
            Transform();
            friend Transform operator*(const Transform &lhs, const Transform &rhs);
            friend bool operator==(const Transform &lhs, const Transform &rhs);
            friend bool operator!=(const Transform &lhs, const Transform &rhs);
            Vector3D getPosition() const;
            Vector3D getEulerAngles() const;
            float get_x();
            float get_y();
            float get_z();
            float get_roll();
            float get_pitch();
            float get_yaw();
            Frame get_parent() const;
            Frame get_child() const;
            void set_roll(float);
            void set_pitch(float);
            void set_yaw(float);
            static Transform identity(Frame parent, Frame child);
            Eigen::Matrix4d matrix;
            static Transform multiply(std::vector<Transform>); 
            Transform inverse();
            geometry_msgs::Transform to_ros();
            geometry_msgs::TransformStamped stamp_and_to_ros();
        private:
            float roll, pitch, yaw;
            float x, y, z;
            Frame parent, child;
            Eigen::Matrix4d constructMatrix(float x, float y, float z, float roll, float pitch, float yaw);
    };

    // Make this a friend?
    std::ostream& operator<<(std::ostream &os, Transform &transform);
}
#endif
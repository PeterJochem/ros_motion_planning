#ifndef UTILITIES
#define UTILITIES
#include <Eigen/Dense>
#include <math.h>
#include <vector>
#include <bits/stdc++.h>
#include <cmath>
#include <math.h>


namespace utilities {

	Eigen::Matrix3d euler_angles_to_rotation_matrix(float roll, float pitch, float yaw);
	std::tuple<float, float, float> rotation_matrix_to_euler_angles(const Eigen::Matrix3d &matrix);
    std::tuple<float, float, float> parse_position(const Eigen::Matrix4d &matrix);
    std::tuple<float, float, float> parse_euler_angles(const Eigen::Matrix4d &matrix);
    bool nearly_equal(float x, float y, float epsilon=0.0005);
    bool nearly_equal(const Eigen::Matrix3d &actual, const Eigen::Matrix3d &expected);    


    // Make a utiity for loading Eigen matrices from a vector

}

#endif
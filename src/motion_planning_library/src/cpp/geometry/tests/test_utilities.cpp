#include "transform/utilities.hpp"
#include <gtest/gtest.h>

namespace geometry {

using namespace utilities;

TEST(Utilities, test_euler_angles_to_rotation_matrix) {

    float roll = 0.;
    float pitch = 0.;
    float yaw = 0.;

    auto matrix = euler_angles_to_rotation_matrix(roll, pitch, yaw);
    EXPECT_TRUE(matrix.isApprox(Eigen::Matrix3d::Identity(3,3)));
    ASSERT_TRUE(nearly_equal(matrix.determinant(), 1.));
}

TEST(Utilities, test_euler_angles_to_rotation_matrix2) {

    float roll = 1.57;
    float pitch = 0.;
    float yaw = 0.;

    Eigen::Matrix3d matrix = euler_angles_to_rotation_matrix(roll, pitch, yaw);
    Eigen::Matrix3d expected;

    expected << 1., 0., 0., 0., 0.0007963, -0.9999997, 0., 0.9999997, 0.0007963;
    ASSERT_TRUE(nearly_equal(matrix, expected));
    ASSERT_TRUE(nearly_equal(matrix.determinant(), 1.));
}


TEST(Utilities, test_euler_angles_to_rotation_matrix3) {

    float roll = 1.2;
    float pitch = 2.25;
    float yaw = 1.0;

    Eigen::Matrix3d matrix = euler_angles_to_rotation_matrix(roll, pitch, yaw);
    Eigen::Matrix3d expected;
    expected << -0.3394037,  0.5285899,  0.7780732, 0.6967379, -0.4144475,  0.5854824, 0.6319506,  0.7408279, -0.2276236;
    ASSERT_TRUE(nearly_equal(matrix, expected));
    ASSERT_TRUE(nearly_equal(matrix.determinant(), 1.));
}

TEST(Utilities, test_euler_angles_to_rotation_matrix4) {

    float roll = -1.2;
    float pitch = -2.25;
    float yaw = -1.0;

    Eigen::Matrix3d matrix = euler_angles_to_rotation_matrix(roll, pitch, yaw);

    Eigen::Matrix3d expected;    
    expected << -0.3394037, -0.5285899, -0.7780732, 0.0869108,  0.8060130, -0.5854824, 0.9366171, -0.2663378, -0.2276236;
    ASSERT_TRUE(nearly_equal(matrix, expected));
    ASSERT_TRUE(nearly_equal(matrix.determinant(), 1.));
}



}
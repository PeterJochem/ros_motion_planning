#include "transform/transform.hpp"
#include "primitives/vector.hpp"
#include <gtest/gtest.h>
#include <tuple>

using namespace geometry;
using namespace std;

float x = 0.1;
float y = 0.2;
float z = 0.3;
float roll = 0.3;
float pitch = 0.4;
float yaw = 0.5;
string parent_name ="fake parent";
string child_name = "fake child";
Frame parent = Frame(parent_name);
Frame child = Frame(child_name);
Vector3D position = Vector3D(x, y, z);
Vector3D euler_angles = Vector3D(roll, pitch, yaw);
Transform transform1 = Transform(parent, child, x, y, z, roll, pitch, yaw);
Transform transform2 = Transform(parent, child, position, euler_angles);


class TransformConstructorsFixture :public ::testing::TestWithParam<Transform> {
  /* Adapted from online:
     https://www.sandordargo.com/blog/2019/04/24/parameterized-testing-with-gtest
  */
};

TEST_P(TransformConstructorsFixture, test_constructors) {
  
  Transform transform = GetParam();

  EXPECT_EQ(transform.get_parent(), parent);
  EXPECT_EQ(transform.get_child(), child);

  EXPECT_FLOAT_EQ(transform.get_x(), x);
  EXPECT_FLOAT_EQ(transform.get_y(), y);
  EXPECT_FLOAT_EQ(transform.get_z(), z);
  EXPECT_FLOAT_EQ(transform.get_roll(), roll);
  EXPECT_FLOAT_EQ(transform.get_pitch(), pitch);
  EXPECT_FLOAT_EQ(transform.get_yaw(), yaw);
}

INSTANTIATE_TEST_CASE_P(Transform, TransformConstructorsFixture, ::testing::Values(transform1, transform2));


TEST(Transform, test_multiplication) {
  Transform transform3 = transform1 * Transform::identity(parent, child);
  ASSERT_EQ(transform1, transform3);
}

TEST(Transform, test_equal_operator) {
  ASSERT_EQ(transform1, transform1);
  ASSERT_EQ(transform1, transform2);
}
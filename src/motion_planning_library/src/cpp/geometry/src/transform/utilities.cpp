#include "transform/utilities.hpp"


namespace utilities {

    Eigen::Matrix3d roll_matrix(float roll) {
        using namespace std;

        Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero(3,3);

        matrix(0,0) = 1.;
        matrix(1,1) = cos(roll);
        matrix(1,2) = -sin(roll);
        matrix(2,1) = sin(roll);
        matrix(2,2) = cos(roll); 

        return matrix;
    }

    Eigen::Matrix3d pitch_matrix(float pitch) {
        using namespace std;

        Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero(3,3);

        matrix(0,0) = cos(pitch);
        matrix(0,2) = sin(pitch);
        matrix(1,1) = 1.;
        matrix(2,0) = -sin(pitch);
        matrix(2,2) = cos(pitch);

        return matrix;
    }

    Eigen::Matrix3d yaw_matrix(float yaw) {
        using namespace std;

        Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero(3,3);

        matrix(0,0) = cos(yaw);
        matrix(0,1) = -sin(yaw);
        matrix(1,0) = sin(yaw);
        matrix(1,1) = cos(yaw);
        matrix(2,2) = 1.;
        
        return matrix;
    }

    Eigen::Matrix3d euler_angles_to_rotation_matrix(float roll, float pitch, float yaw) {
        
        return roll_matrix(roll) * pitch_matrix(pitch) * yaw_matrix(yaw);
        //return yaw_matrix(yaw) * pitch_matrix(pitch) * roll_matrix(roll);
    }


    std::tuple<float, float, float> rotation_matrix_to_euler_angles(const Eigen::Matrix3d &matrix) {
        /* Adapted from online
           https://www.geometrictools.com/Documentation/EulerAngles.pdf at the bottom of page 4.
        */
        using namespace std;

        float roll, pitch, yaw;

        if (matrix(0,2) < 1) {
            if (matrix(0,2) > -1) {
                roll = atan2(-matrix(1,2), matrix(2,2));
                pitch = asin(matrix(0,2));
                yaw = atan2(-matrix(0,1), matrix(0,0));
            }
            else {
                roll = -atan2(matrix(1,0), matrix(1,1));
                pitch = -M_PI/2;
                yaw = 0.;
            }
        }
        else {
            roll = atan2(matrix(1,0), matrix(1,1));
            pitch = M_PI;
            yaw = 0.;
        }

        return std::make_tuple(roll, pitch, yaw);
    }

    std::tuple<float, float, float> parse_position(const Eigen::Matrix4d &matrix) {
        
        float x = matrix(0,3);
        float y = matrix(1,3);
        float z = matrix(2,3);
        return std::make_tuple(x, y, z);
    }

    std::tuple<float, float, float> parse_euler_angles(const Eigen::Matrix4d &transformation_matrix) {
        
        Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d(3,3);
        for (int row = 0; row < rotation_matrix.rows(); row++) {
            for (int column = 0; column < rotation_matrix.cols(); column++) {
                rotation_matrix(row, column) = transformation_matrix(row, column);
            }
        }

        return rotation_matrix_to_euler_angles(rotation_matrix);
    }
    
    bool nearly_equal(float x, float y, float epsilon) {

        using namespace std;
        return abs(x - y) < (epsilon); // * max(abs(x), abs(y)));
    }

    bool nearly_equal(const Eigen::Matrix3d &actual, const Eigen::Matrix3d &expected) {

        bool same_num_rows = actual.rows() == expected.rows();
        bool same_num_cols = actual.cols() == expected.cols();

        if (!(same_num_rows && same_num_cols)) {
            return false;
        }

        for (int row = 0; row < expected.rows(); row++) {
          for (int column = 0; column < expected.cols(); column++) {
            if (!nearly_equal(actual(row,column), expected(row,column))) {
                return false;
            }
            }
        }
        return true;
    }
}
#include "robots/joint.hpp"


namespace Robot {

    Joint::Joint(geometry::Transform transform, geometry::Axis axis): transform(transform), axis(axis) {
        zero_angle_transform = transform;

    }

    geometry::Transform Joint::get_transform() {
        return transform;
    }

    void Joint::apply_rotation(float radians) {

        if (axis.is_pure_roll()) {

            float new_roll = zero_angle_transform.get_roll() + radians;
            transform.set_roll(new_roll);
        }
        else if (axis.is_pure_pitch()) {

            float new_pitch = zero_angle_transform.get_pitch() + radians;
            transform.set_pitch(new_pitch);
        }
        else if (axis.is_pure_yaw()) {

            float new_yaw = zero_angle_transform.get_yaw() + radians;
            transform.set_yaw(new_yaw);
        }
        else {
            throw std::runtime_error("The axis must be a pure roll, pitch, or yaw.");
        }

        angle = radians;
    }

}
#include "primitives/vector.hpp"


namespace geometry {

    class Axis {
        public:
            Axis(geometry::Vector3D);
            bool is_pure_roll();
            bool is_pure_pitch();
            bool is_pure_yaw();

        private:
            geometry::Vector3D vector;
    };
}
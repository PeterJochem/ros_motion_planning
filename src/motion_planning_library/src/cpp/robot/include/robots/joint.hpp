#include "transform/transform.hpp"
#include "primitives/axis.hpp"



namespace Robot {

    class Joint {

        public:
            Joint(geometry::Transform, geometry::Axis);
            geometry::Transform get_transform();
            void apply_rotation(float raidians);

        private:
            geometry::Transform zero_angle_transform;
            geometry::Transform transform;
            geometry::Axis axis;
            float angle;
    
    };
}
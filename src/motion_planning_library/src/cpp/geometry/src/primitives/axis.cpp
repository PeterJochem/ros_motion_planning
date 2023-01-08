
#include "primitives/axis.hpp"


namespace geometry {

    Axis::Axis(geometry::Vector3D vector): vector(vector) {
        
    }


    bool Axis::is_pure_roll(){
        return geometry::Vector3D(1., 0., 0.) == vector;
    }   
    
    bool Axis::is_pure_pitch() {
        return geometry::Vector3D(0., 1., 0.) == vector;
    }

    bool Axis::is_pure_yaw() {
        return geometry::Vector3D(0., 0., 1.) == vector;
    }




}
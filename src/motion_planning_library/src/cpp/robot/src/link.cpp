#include "robots/link.hpp"


namespace Robot {


    Link::Link(geometry::Transform transform): transform(transform) {
    }

    geometry::Transform Link::get_transform() {
        return transform;
    }

}
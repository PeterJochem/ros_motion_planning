#include "transform/transform.hpp"


namespace Robot {
    
    class Link {
        public:
            Link(geometry::Transform);
            geometry::Transform get_transform();


        private:

            geometry::Transform transform;
            //Mesh collision_mesh;
            //Mesh visual_mesh;



    };
}
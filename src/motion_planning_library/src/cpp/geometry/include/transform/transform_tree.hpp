#ifndef TRANSFORM_TREE
#define TRANSFORM_TREE
#include <vector>
#include "transform/transform.hpp"
#include "transform/frame.hpp"

namespace geometry {

class TransformTree { 

    public:
        TransformTree(Frame);

        virtual void add(Transform) = 0;
        virtual int size() = 0;
        virtual Transform measure_transform(Frame, Frame) = 0;
        virtual void set_transform(Transform, float x, float y, float z, float roll, float pitch, float yaw) = 0;
        virtual void set_transform(Transform, Vector3D position, Vector3D euler_angles) = 0;
        virtual void set_transform(Transform, Eigen::Matrix4d matrix) = 0;
        virtual Transform get_parent_transform(Frame) = 0;
        virtual std::vector<Frame> get_leaf_frames() = 0;
        virtual Frame get_root() = 0;
        virtual std::vector<Transform> all_transforms() = 0;

    protected:
        Frame root;

    private:
        virtual bool does_exist(const Frame& frame) = 0;
        virtual bool does_exist(const Transform& transform) = 0;
        virtual bool is_leaf(const Frame& frame) = 0;

};
}
#endif
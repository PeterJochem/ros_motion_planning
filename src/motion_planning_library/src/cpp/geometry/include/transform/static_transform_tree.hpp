#ifndef STATIC_TRANSFORM_TREE
#define STATIC_TRANSFORM_TREE
#include <vector>
#include <algorithm>
#include "transform/transform.hpp"
#include "transform/transform_tree.hpp"
#include "transform/node.hpp"
#include "transform/frame.hpp"


// One TransformTree tracks what is really the state of the world and can show it in RVIZ.
// N TransformTree's can track the state of the world for collision checking.

// To minimize copying, create a pool of N of these that you can re-use.
// Works great as long as there are no new objects in the world.
// During each planning query, create N TransformTrees and re-use them during that planning query.
// This is to avoid every collision check having to create/copy its own Transform Tree object. 


// A basic planning query would 
// 1) Set the starting joint angles

// 2) For each pair of collision objects in the tree,
// 3) Read the TransformTree to check if object A is in collision with object B. 

// 1) setTransform(transforms for joints)
// 2) for each collision object:
//          Get transform();

namespace geometry {

class StaticTransformTree: public TransformTree {

    public:
        StaticTransformTree(Frame root);
        void add(Transform) override;
        int size();
        
       /**
       * Measure the second frame relative to the first one.
       * @param Frame The frame to do the measurement relative to.
       * @param Frame The frame to measure relative to the other frame.
       * @return The transform between the parent frame and the child frame.
       */
        Transform measure_transform(Frame, Frame) override;
        void set_transform(Transform, float x, float y, float z, float roll, float pitch, float yaw) override;
        void set_transform(Transform, Vector3D position, Vector3D euler_angles) override;
        void set_transform(Transform, Eigen::Matrix4d matrix) override;
        Transform get_parent_transform(Frame) override;
        std::vector<Frame> get_leaf_frames() override;
        Frame get_root() override;
        std::vector<Transform> all_transforms() override; 


    private:
        std::map<Frame, Transform> child_frame_to_transform;
        void insert_into_map(Transform& transform);
        Transform measure_in_root_frame(Frame);

        bool does_exist(const Transform& transform) override;
        bool does_exist(const Frame& frame) override;
        bool is_leaf(const Frame& frame) override;
};
}
#endif
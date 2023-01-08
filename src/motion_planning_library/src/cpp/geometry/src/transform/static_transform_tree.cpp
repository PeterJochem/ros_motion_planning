#include "transform/static_transform_tree.hpp"



namespace geometry {    


    StaticTransformTree::StaticTransformTree(Frame root): TransformTree(root) {
    }
    
    void StaticTransformTree::add(Transform transform) {

    
        if (does_exist(transform)) {
            throw std::runtime_error("The transform already exists.");
        }

        else if (!does_exist(transform.get_parent())) {
            throw std::runtime_error("The parent frame does not exist.");
        }

        else if (does_exist(transform.get_child())) {
            throw std::runtime_error("The child frame already exists.");
        }

        Frame child_frame = transform.get_child();
        child_frame_to_transform[child_frame] = transform;
    }

    bool StaticTransformTree::does_exist(const Transform& transform) {
        

        for (std::map<Frame, Transform>::iterator itr = child_frame_to_transform.begin(); itr != child_frame_to_transform.end(); itr++) {
            if (itr->second == transform) {
                return true;
            }
        }

        return false;
    }

    int StaticTransformTree::size() {
        return child_frame_to_transform.size();
    }

    Transform StaticTransformTree::measure_transform(Frame parent, Frame child) {
        
        if (!(does_exist(child) && does_exist(child))) {
            throw std::runtime_error("A frame does not exist.");
        }

        Transform root_to_parent = measure_in_root_frame(parent);
        Transform root_to_child = measure_in_root_frame(child);

        return root_to_parent.inverse() * root_to_child;

    }

    Transform StaticTransformTree::measure_in_root_frame(Frame frame) {

        if (frame == root) {
            return Transform::identity(frame, frame);
        }
        else if (get_parent_transform(frame).get_parent() == root) {
            return get_parent_transform(frame);
        }

        std::vector<Transform> transforms = std::vector<Transform>();

        Transform parent_transform;
        Frame parent_frame = frame;
        do {
            parent_transform = get_parent_transform(parent_frame);
            parent_frame = parent_transform.get_parent();
            transforms.push_back(parent_transform);
        }
        while (parent_frame != root);

        std::reverse(transforms.begin(), transforms.end());
        return Transform::multiply(transforms); 
    }

    void StaticTransformTree::set_transform(Transform transform, float x, float y, float z, float roll, float pitch, float yaw) {

        if (!does_exist(transform)) {
            throw std::runtime_error("The transform does not exist.");
        }

        Frame child_frame = transform.get_child();
        Frame parent_frame = transform.get_parent();

         // This is stack allocated? No, it is copied into the container.
        child_frame_to_transform[child_frame] = Transform(parent_frame, child_frame, x, y, z, roll, pitch, yaw);
    }
    
    void StaticTransformTree::set_transform(Transform transform, Vector3D position, Vector3D euler_angles) {

        if (!does_exist(transform)) {
            throw std::runtime_error("The transform does not exist.");
        }

        Frame child_frame = transform.get_child();
        Frame parent_frame = transform.get_parent();

        // This is stack allocated? No, it is copied into the container.
        child_frame_to_transform[child_frame] = Transform(parent_frame, child_frame, position, euler_angles);
    }

    void StaticTransformTree::set_transform(Transform transform, Eigen::Matrix4d matrix) {

        if (!does_exist(transform)) {
            throw std::runtime_error("The transform does not exist.");
        }

        Frame child_frame = transform.get_child();
        Frame parent_frame = transform.get_parent();

        // This is stack allocated? No, it is copied into the container.
        child_frame_to_transform[child_frame] = Transform(parent_frame, child_frame, matrix);
    }

    Transform StaticTransformTree::get_parent_transform(Frame child) {

        if (!does_exist(child)) {
            throw std::runtime_error("The frame does not exist.");
        }

        return child_frame_to_transform[child];
    }

    bool StaticTransformTree::does_exist(const Frame& frame) {
        
        return !(child_frame_to_transform.find(frame) == child_frame_to_transform.end()) || frame == root;
    }

    std::vector<Frame> StaticTransformTree::get_leaf_frames() {

        std::vector<Frame> frames = {};
        for (std::map<Frame, Transform>::iterator itr = child_frame_to_transform.begin(); itr != child_frame_to_transform.end(); itr++) {
            Frame frame = itr->first;
            if (is_leaf(frame)) {
                frames.push_back(frame);
            }
        }

        return frames;
    }

    bool StaticTransformTree::is_leaf(const Frame& frame) {

        if (!does_exist(frame)) {
            throw std::runtime_error("The frame does not exist.");
        }
        
        for (std::map<Frame, Transform>::iterator itr = child_frame_to_transform.begin(); itr != child_frame_to_transform.end(); itr++) {
            if (itr->second.get_parent() == frame) {
                return false;
            }
        }

        return true;
    }

    Frame StaticTransformTree::get_root() {
        return root;
    }

    std::vector<Transform> StaticTransformTree::all_transforms() {

        std::vector<Transform> transforms = std::vector<Transform>();
        for (std::map<Frame, Transform>::iterator itr = child_frame_to_transform.begin(); itr != child_frame_to_transform.end(); itr++) {
            transforms.push_back(itr->second);
        }            

        return transforms;
    }

}

#include "transform/static_transform_tree.hpp"
#include "gtest/gtest.h" 
#include <iostream>

namespace geometry {

StaticTransformTree simple_linear_tree() {

    Frame root = Frame("the root");
    StaticTransformTree tree = StaticTransformTree(root);

    Frame a = Frame("a");
    Frame b = Frame("b");
    Frame c = Frame("c");

    Transform root_to_a = Transform(root, a, 0., 0., 0., 0., 0., 0.);
    Transform a_to_b = Transform(a, b, 0., 0., 0., 0., 0., 0.);
    Transform b_to_c = Transform(b, c, 0., 0., 0., 0., 0., 0.);

    tree.add(root_to_a);
    tree.add(a_to_b);
    tree.add(b_to_c);

    return tree;
}

StaticTransformTree binary_tree() {

    Frame root = Frame("the root");
    StaticTransformTree tree = StaticTransformTree(root);

    Frame a = Frame("a");
    Frame b = Frame("b");

    Transform root_to_a = Transform(root, a, 0., 0., 0., 0., 0., 0.);
    Transform root_to_b = Transform(root, b, 0., 0., 0., 0., 0., 0.);

    tree.add(root_to_a);
    tree.add(root_to_b);

    return tree;
}


TEST(TransformTree, test_constructing_a_simple_tree) {

    StaticTransformTree tree = simple_linear_tree();
    EXPECT_EQ(tree.size(), 3);
}


TEST(TransformTree, test_leaf_to_root_traversal_on_a_simple_tree) {


    StaticTransformTree tree = simple_linear_tree();

    std::vector<Frame> leaves = tree.get_leaf_frames();
    EXPECT_EQ(leaves.size(), 1);

    Frame child = leaves[0];
    while (tree.get_parent_transform(child).get_parent() != tree.get_root()) {
        child = tree.get_parent_transform(child).get_parent();
    }

    child = tree.get_parent_transform(child).get_parent();

    EXPECT_EQ(child, tree.get_root());
}


TEST(TransformTree, test_leaf_to_root_traversal_on_a_binary_tree) {


    StaticTransformTree tree = binary_tree();

    std::vector<Frame> leaves = tree.get_leaf_frames();
    EXPECT_EQ(leaves.size(), 2);

    Frame child = leaves[0];
    while (tree.get_parent_transform(child).get_parent() != tree.get_root()) {
        child = tree.get_parent_transform(child).get_parent();
    }

    child = tree.get_parent_transform(child).get_parent();

    EXPECT_EQ(child, tree.get_root());
}

TEST(TransformTree, test_measuring_frames_in_a_tree_with_only_translations) {

    // Construct tree
    Frame root = Frame("the root");
    StaticTransformTree tree = StaticTransformTree(root);

    Frame a = Frame("a");
    Frame b = Frame("b");
    Frame c = Frame("c");

    Transform root_to_a = Transform(root, a, 1., 0., 0., 0., 0., 0.);
    Transform a_to_b = Transform(a, b, 2., 0., 0., 0., 0., 0.);
    Transform b_to_c = Transform(b, c, 3., 0., 0., 0., 0., 0.);

    tree.add(root_to_a);
    tree.add(a_to_b);
    tree.add(b_to_c);

    Transform root_to_c = tree.measure_transform(root, c);
    EXPECT_EQ(root_to_c, Transform(root, c, 6., 0., 0., 0., 0., 0.));

    Transform c_to_root = tree.measure_transform(c, root);
    EXPECT_NE(c_to_root, root_to_c);
    EXPECT_EQ(c_to_root.inverse(), root_to_c);

    Transform a_to_c = tree.measure_transform(a, c);
    EXPECT_EQ(a_to_c, Transform(a, c, 5., 0., 0., 0., 0., 0.));

    EXPECT_EQ(tree.measure_transform(a, b), a_to_b);
}

TEST(TransformTree, test_measuring_frames_in_a_tree_with_only_rotations) {

    // Construct tree
    Frame root = Frame("the root");
    StaticTransformTree tree = StaticTransformTree(root);

    Frame a = Frame("a");
    Frame b = Frame("b");
    Frame c = Frame("c");
    Frame d = Frame("d");

    Transform root_to_a = Transform(root, a, 0., 0., 0., 0.1, 0., 0.);
    Transform a_to_b = Transform(a, b, 0., 0., 0., 0.2, 0., 0.);
    Transform b_to_c = Transform(b, c, 0., 0., 0., 0.4, 0., 0.);
    Transform c_to_d = Transform(c, d, 0., 0., 0., -1.5, 0., 0.);

    tree.add(root_to_a);
    tree.add(a_to_b);
    tree.add(b_to_c);
    tree.add(c_to_d);

    Transform root_to_c = tree.measure_transform(root, c);
    EXPECT_EQ(root_to_c, Transform(root, c, 0., 0., 0., 0.7, 0., 0.));

    Transform root_to_b = tree.measure_transform(root, b);
    EXPECT_EQ(root_to_b, Transform(root, b, 0., 0., 0., 0.3, 0., 0.));

    Transform root_to_d = tree.measure_transform(root, d);
    EXPECT_EQ(root_to_d, Transform(root, d, 0., 0., 0., -0.8, 0., 0.));

    Transform d_to_root = tree.measure_transform(d, root);
    EXPECT_NE(root_to_d, d_to_root);
    EXPECT_EQ(root_to_d, d_to_root.inverse());
}

TEST(TransformTree, test_measuring_frames_in_a_tree_with_both_rotations_and_translations) {

    // Construct tree
    Frame root = Frame("the root");
    StaticTransformTree tree = StaticTransformTree(root);

    Frame a = Frame("a");
    Frame b = Frame("b");
    Frame c = Frame("c");
    Frame d = Frame("d");

    Transform root_to_a = Transform(root, a, 1., 0., 0., 0.1, 0., 0.);
    Transform a_to_b = Transform(a, b, 2., 0., 0., 0.2, 0., 0.);
    Transform b_to_c = Transform(b, c, 3., 0., 0., 0.4, 0., 0.);
    Transform c_to_d = Transform(c, d, 4., 0., 0., -1.5, 0., 0.);

    tree.add(root_to_a);
    tree.add(a_to_b);
    tree.add(b_to_c);
    tree.add(c_to_d);

    Transform root_to_c = tree.measure_transform(root, c);
    EXPECT_EQ(root_to_c, Transform(root, c, 6., 0., 0., 0.7, 0., 0.));

    Transform root_to_b = tree.measure_transform(root, b);
    EXPECT_EQ(root_to_b, Transform(root, b, 3., 0., 0., 0.3, 0., 0.));

    Transform root_to_d = tree.measure_transform(root, d);
    EXPECT_EQ(root_to_d, Transform(root, d, 10., 0., 0., -0.8, 0., 0.));

    Transform d_to_root = tree.measure_transform(d, root);
    EXPECT_NE(root_to_d, d_to_root);
    EXPECT_EQ(root_to_d, d_to_root.inverse());
}




}


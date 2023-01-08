#include "transform/node.hpp"
#include "transform/transform.hpp"
#include <gtest/gtest.h>

namespace geometry {

TEST(Node12345, testDefaultConstructor) {

    Node<int> node = Node<int>();
    //EXPECT_FALSE(node.has_child(""));
    EXPECT_FALSE(node.has_parent());
}


TEST(Node, testSingleChildConstruction) {

    Node<int> parent_node = Node<int>();
    Node<int> child_node = Node<int>(); 
    Node<int> node = Node<int>(&parent_node, &child_node);

    //EXPECT_FALSE(node.has_child(""));
    //ASSERT_TRUE(node.has_child("the child's real name"))
    ASSERT_TRUE(node.has_parent());
}

TEST(Node, testMultipleeChildrenConstruction) {

    // implement me
}
}
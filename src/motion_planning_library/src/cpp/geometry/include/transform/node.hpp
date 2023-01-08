#ifndef TRANSFORM_NODE
#define TRANSFORM_NODE
#include "transform/transform.hpp"
#include <vector>
#include <string>

namespace geometry {

    template <typename T> class Node {

        public:
            Node() {
                parent = nullptr;
                children = std::vector<Node*>();
            }

            Node(Node<T>* parent, Node<T>* child): parent(parent) {

                if (child) {
                    children = {child};
                }
            }
            
            Node(Node<T>* parent, std::vector<Node<T>*> children): parent(parent), children(children) {
                
            }

            bool has_parent() {
                return parent != nullptr;
            }

            void add_child(Node<T>* child) {
                children.push_back(child);
            }


        private:
            Node<T>* parent;
            std::vector<Node<T>*> children;
    };
}
#endif
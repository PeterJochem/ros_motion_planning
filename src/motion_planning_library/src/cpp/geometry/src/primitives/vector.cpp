#include "primitives/vector.hpp"
#include "transform/utilities.hpp"
#include <ostream>


namespace geometry {
    
    Vector3D::Vector3D(float x, float y, float z): x(x), y(y), z(z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    int Vector3D::dimensions() const {
        return 3;
    }

    std::vector<float> Vector3D::coordinates() const {
        return {x, y, z};
    }

    Vector2D::Vector2D(float x, float y): x(x), y(y) {
    }

    int Vector2D::dimensions() const {
        return 2;
    }
    
    std::vector<float> Vector2D::coordinates() const {
        return {x, y};
    }

    bool operator==(const Vector &lhs, const Vector &rhs) {

        if (lhs.dimensions() != rhs.dimensions()) {
            return false;
        }

        const std::vector<float> lhs_coordinates = lhs.coordinates();
        const std::vector<float> rhs_coordinates = rhs.coordinates();

        for (int i = 0; i < lhs_coordinates.size(); i++) {
            if (!utilities::nearly_equal(lhs_coordinates[i], rhs_coordinates[i])) {
                return false;
            }
        }

        return true;
    }
    
    bool operator!=(const Vector &lhs, const Vector &rhs) {
        return !(lhs == rhs);
    }

    std::ostream& operator<<(std::ostream &os, Vector& vector) {
        
        os << "(";
        std::vector<float> coordinates = vector.coordinates();
        for (std::vector<float>::iterator itr = coordinates.begin(); itr != coordinates.end(); itr++) {
            os << *itr;
            if ((itr + 1) != coordinates.end()) {
                os << ", ";
            }
        }

        os << ")";
        return os;
    }

}
#ifndef VECTOR
#define VECTOR
#include <tuple>
#include <vector> 

namespace geometry {

    class Vector {

        public:
            virtual std::vector<float> coordinates() const = 0;
            virtual int dimensions() const = 0;
            friend bool operator!=(const Vector &lhs, const Vector &rhs);
            friend bool operator==(const Vector &lhs, const Vector &rhs);
    };

    class Vector2D: public Vector {
        public:
            Vector2D(float x, float y);
            int dimensions() const;
            std::vector<float> coordinates() const;
        private:
            float x;
            float y;
    };


    class Vector3D: public Vector {
        public:
            Vector3D(float x, float y, float z);
            int dimensions() const;
            std::vector<float> coordinates() const;
        private:
            float x;
            float y;
            float z;
    };

    std::ostream& operator<<(std::ostream &os, Vector& vector);
    // std::ostream& operator<<(std::ostream &os, Vector &vector);
}
#endif
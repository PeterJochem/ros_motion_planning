#ifndef FRAME
#define FRAME
#include <string>

namespace geometry {

    class Frame { 

        public:
            Frame(std::string name);
            Frame();
            std::string get_name() const;
            bool is_unnamed();
            friend bool operator==(const Frame &lhs, const Frame &rhs);
            friend bool operator!=(const Frame &lhs, const Frame &rhs);
            friend bool operator<(const Frame &lhs, const Frame &rhs);
        private:
            std::string name;
    };

     std::ostream& operator<<(std::ostream &os, Frame frame);
}
#endif
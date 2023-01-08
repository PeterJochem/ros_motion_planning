#include "transform/frame.hpp"

namespace geometry {

    Frame::Frame(std::string name): name(name) {
    }

    Frame::Frame() {
        name = "UNNAMED";
    }

    bool operator==(const Frame &lhs, const Frame &rhs) {
        return lhs.get_name().compare(rhs.get_name()) == 0;
    }

    bool operator!=(const Frame &lhs, const Frame &rhs) {
        return !(lhs == rhs);
    }

    bool operator<(const Frame &lhs, const Frame &rhs) {
        return lhs.get_name() < rhs.get_name();
    }

    std::string Frame::get_name() const {
        return name;
    }

    bool Frame::is_unnamed() {
        return name == "UNNAMED";
    }

    std::ostream& operator<<(std::ostream &os, Frame frame) {
        os << frame.get_name();
        return os;
    }
}
#include "MTSPBC_util.hpp"
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <vector>


int orientation(Coord a, Coord b, Coord c) {
    long long area = (b.pos_x - a.pos_x)*(c.pos_y - a.pos_y) - (c.pos_x - a.pos_x)*(b.pos_y - a.pos_y);
    if (area < 0) {
        return -1; // cw
    } else if (area > 0) {
        return 1; // ccw
    }
    return 0;
}


uint32_t distance(const Nodes& a, const Nodes& b) {
    double dx = a.pos.pos_x - b.pos.pos_x;
    double dy = a.pos.pos_y - b.pos.pos_y;
    double result = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    return std::round(result);
}


uint32_t distance(const Coord& a, const Coord& b) {
    double dx = a.pos_x - b.pos_x;
    double dy = a.pos_y - b.pos_y;
    double result = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    return std::round(result);
}


uint32_t unassign(const std::vector<uint32_t>& nodes, std::vector<size_t>& un_nodes) {
    int n_removed {};
    for (auto i : nodes) {
        for (uint32_t j{}; j < un_nodes.size(); j++) {
            if (i == un_nodes[j]) {
                un_nodes.erase(un_nodes.begin() + j);
                n_removed++;
                break;
            }
        }
    }
    return n_removed;
}

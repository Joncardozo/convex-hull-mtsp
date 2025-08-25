#include "MTSPBC_util.hpp"
#include "MTSPBC_ds.hpp"
#include "MTSPBC.hpp"
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <utility>
#include <vector>


int orientation(const Coord& a, const Coord& b, const Coord& c) {
    long long area = (b.pos_x - a.pos_x)*(c.pos_y - a.pos_y) - (c.pos_x - a.pos_x)*(b.pos_y - a.pos_y);
    if (area < 0) {
        return -1; // cw
    } else if (area > 0) {
        return 1; // ccw
    }
    return 0;
}


Coord partial_coordinate(const Coord& m, const Coord& n, const double& dt) {
    Coord mn { m - n };
    double mn_norm { coord_norm(mn) };
    Coord mn_unit { mn / mn_norm };
    return m + mn_unit * dt;
}


double coord_norm(const Coord& coord) { return std::sqrt(std::pow(coord.pos_x, 2) + std::pow(coord.pos_y, 2)); }


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


uint32_t distance(const MTSPBC& solution, const uint32_t& event_index, const uint32_t& moving_vehicle) {
    auto event { solution.get_event(event_index) };
    auto e_time { event.first };
    auto e_vehicle { event.second };
    auto e_node { solution.get_node_at_event(e_vehicle, e_time) };
    if (moving_vehicle == e_vehicle) {
        return 0;
    }
    uint32_t mv_time { 0 };
    auto mv_events { solution.get_vehicle_events(moving_vehicle) };
    uint32_t last_e_mv { };
    uint32_t last_e_mv_i { };
    for (uint32_t t { 0 }; t < mv_events.size(); t++) {
        if (mv_events.at(t) > e_time) {
            break;
        }
        last_e_mv = mv_events.at(t);
        last_e_mv_i = t;
    }
    auto last_e_mv_node { solution.get_tour(moving_vehicle).at(last_e_mv_i) };
    if (last_e_mv_node == solution.get_tour(moving_vehicle).back()) {
        return distance(solution.get_coord(e_node), solution.get_coord(last_e_mv_node));
    }
    auto dt { e_time - last_e_mv };
    auto mv_next_node { solution.get_tour(moving_vehicle).at(last_e_mv_i + 1) };
    auto mv_next_coord { solution.get_coord(mv_next_node) };
    auto mv_last_node { solution.get_tour(moving_vehicle).at(last_e_mv_i) };
    auto mv_last_coord { solution.get_coord(mv_last_node) };
    auto real_position { partial_coordinate(mv_last_coord, mv_next_coord, dt) };
    return distance(real_position, solution.get_coord(e_node));
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

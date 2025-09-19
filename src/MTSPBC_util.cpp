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
    Coord mn { n - m };
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


uint32_t distance(const MTSPBC& solution, const uint32_t event_index, const uint32_t moving_vehicle) {
    auto event { solution.get_event(event_index) };
    auto e_time { event.first };
    auto e_vehicle { event.second };
    auto e_node { solution.get_node_at_event(e_vehicle, e_time) };
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
    if (last_e_mv_i == solution.get_tour(moving_vehicle).size() - 1) {
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


Coord real_position(const MTSPBC& solution, const uint32_t moving_vehicle, const uint32_t last_e_mv, const uint32_t last_e_mv_i, const uint32_t e_time) {
    auto last_e_mv_node_1 { solution.get_tour(moving_vehicle).at(last_e_mv_i) };
    auto dt_1 { e_time - last_e_mv };
    auto mv_next_node_1 { solution.get_tour(moving_vehicle).at(last_e_mv_i + 1) };
    auto mv_next_coord_1 { solution.get_coord(mv_next_node_1) };
    auto mv_last_node_1 { solution.get_tour(moving_vehicle).at(last_e_mv_i) };
    auto mv_last_coord_1 { solution.get_coord(mv_last_node_1) };
    auto real_position_1 { partial_coordinate(mv_last_coord_1, mv_next_coord_1, dt_1) };
    return real_position_1;
}


uint32_t distance(const MTSPBC& solution, const uint32_t event_index, const uint32_t moving_vehicle_1, const uint32_t moving_vehicle_2) {
    auto event { solution.get_event(event_index) };
    auto e_time { event.first };
    auto e_vehicle { event.second };
    auto e_node { solution.get_node_at_event(e_vehicle, e_time) };
    if (moving_vehicle_1 == 2 && moving_vehicle_2 == 4 && e_time == 17 && e_vehicle == 2) {
        int debug {};
    }
    if (moving_vehicle_1 == e_vehicle) {
        return distance(solution, event_index, moving_vehicle_2);
    }
    else if (moving_vehicle_2 == e_vehicle) {
        return distance(solution, event_index, moving_vehicle_1);
    }
    auto mv_events_1 { solution.get_vehicle_events(moving_vehicle_1) };
    auto mv_events_2 { solution.get_vehicle_events(moving_vehicle_2) };
    uint32_t last_e_mv_1 { };
    uint32_t last_e_mv_2 { };
    uint32_t last_e_mv_i_1 { };
    uint32_t last_e_mv_i_2 { };
    for (uint32_t t { 0 }; t < mv_events_1.size(); t++) {
        if (mv_events_1.at(t) > e_time) {
            break;
        }
        last_e_mv_1 = mv_events_1.at(t);
        last_e_mv_i_1 = t;
    }
    for (uint32_t t { 0 }; t < mv_events_2.size(); t++) {
        if (mv_events_2.at(t) > e_time) {
            break;
        }
        last_e_mv_2 = mv_events_2.at(t);
        last_e_mv_i_2 = t;
    }
    if (last_e_mv_i_1 == mv_events_1.size() - 1) {
        uint32_t mv_1_last { solution.get_tour(moving_vehicle_1).back() };
        if (last_e_mv_i_2 == mv_events_2.size() - 1) {
            uint32_t mv_2_last { solution.get_tour(moving_vehicle_2).back() };
            return distance(solution.get_coord(mv_1_last), solution.get_coord(mv_2_last));
        }
        else {
            Coord mv_2_real { real_position(solution, moving_vehicle_2, last_e_mv_2, last_e_mv_i_2, e_time) };
            return distance(solution.get_coord(mv_1_last), mv_2_real);
        }
    }
    else {
        Coord mv_1_real { real_position(solution, moving_vehicle_1, last_e_mv_1, last_e_mv_i_1, e_time) };
        if (last_e_mv_i_2 == mv_events_2.size() - 1) {
            uint32_t mv_2_last { solution.get_tour(moving_vehicle_2).back() };
            return distance(solution.get_coord(mv_2_last), mv_1_real);
        }
        else {
            Coord mv_2_real { real_position(solution, moving_vehicle_2, last_e_mv_2, last_e_mv_i_2, e_time) };
            return distance(mv_1_real, mv_2_real);
        }
    }
    return 0;
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

#include "MTSPBC_chh.hpp"
#include "MTSPBC.hpp"
#include "Cht.hpp"
#include "MTSPBC_util.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <sys/types.h>
#include <vector>
#include <algorithm>


uint32_t add_convex_hull(MTSPBC& solution, const uint32_t vehicle, std::vector<size_t>& un_nodes, const std::vector<Coord>& coord) {              // find the hull for one vehicle
    Cht tour;
    // Find the leftmost unassigned node
    size_t point_left_most_i = un_nodes[0];
    for (size_t idx = 1; idx < un_nodes.size(); idx++) {
        Coord left_most_coord = coord[point_left_most_i];
        Coord current_point = coord[un_nodes[idx]];
        if (current_point.pos_x < left_most_coord.pos_x ||
            (current_point.pos_x == left_most_coord.pos_x &&
             current_point.pos_y < left_most_coord.pos_y)) {
            point_left_most_i = un_nodes[idx];
        }
    }

    // Build vector of Nodes for unassigned nodes (excluding leftmost)
    std::vector<Nodes> nodes;
    Nodes point_left_most;
    point_left_most.index = point_left_most_i;
    point_left_most.pos = coord[point_left_most_i];
    for (size_t i = 0; i < un_nodes.size(); i++) {
        size_t idx = un_nodes[i];
        if (idx != point_left_most_i) {
            Nodes n;
            n.index = idx;
            n.pos = coord[idx];
            nodes.push_back(n);
        }
    }

    // Sort nodes by polar angle with respect to leftmost point
    std::sort(nodes.begin(), nodes.end(), [&point_left_most](const Nodes& a, const Nodes& b) {
        int d = orientation(point_left_most.pos, a.pos, b.pos);
        if (d > 0) return true;
        if (d < 0) return false;
        // Collinear: closer one first
        return distance(point_left_most, a) < distance(point_left_most, b);
    });

    // Graham scan
    std::vector<Nodes> stack;
    stack.push_back(point_left_most);
    if (!nodes.empty()) stack.push_back(nodes[0]);
    for (size_t i = 1; i < nodes.size(); i++) {
        while (stack.size() > 1 &&
               orientation(stack[stack.size()-2].pos, stack[stack.size()-1].pos, nodes[i].pos) <= 0) {
            stack.pop_back();
        }
        stack.push_back(nodes[i]);
    }

    // Add hull to the solution
    std::vector<uint32_t> hull_indices;
    for (const auto& n : stack) {
        solution.push_back(vehicle, static_cast<uint32_t>(n.index));
    }
    return 0;
}


uint32_t find_onion_hull(MTSPBC& solution, std::vector<size_t>& un_nodes, const std::vector<Coord>& coord) {

    uint32_t k_vehicles { solution.get_k_vehicles() };

    // iterativamente, encontra uma rota para cada veículo
    for (uint32_t i{ 0 }; i < k_vehicles; i++) {
        add_convex_hull(solution, i, un_nodes, coord);
        unassign(solution.get_tour(i), un_nodes);
    }
    return 0;
}


uint32_t cheapest_insertion(MTSPBC& solution, std::vector<size_t>& un_nodes, const std::vector<Coord>& coord) {      // find heuristic solution
    if (solution.get_total_obj() == 0) {
        throw std::logic_error("error: cheapest heuristic over empty solution not allowed");
    }
    while(un_nodes.size() > 0) {
        uint32_t k_index {};
        uint32_t position {};
        uint32_t new_cost { 999999 };
        uint32_t unassigned_index {};
        for (auto un { 0 }; un < un_nodes.size(); un++) {
            for (auto k{ 0 }; k < solution.get_k_vehicles(); k++) {
                for (auto i{ 0 }; i < solution.get_tour(k).size(); i++) {
                    std::vector<uint32_t> insertion_hull { solution.get_tour(k) };
                    size_t past_node {};
                    if (i > 0) {
                        past_node = insertion_hull.at(i - 1);
                    }
                    size_t inserted_node { un_nodes[un] };
                    size_t next_node { insertion_hull.at(i) };
                    insertion_hull.insert(insertion_hull.begin() + i, un_nodes[un]);
                    uint32_t temp_cost { solution.get_obj_vehicle(k) };
                    if (i == 0) {
                        temp_cost += solution.get_cost(next_node, inserted_node);
                    }
                    else {
                        temp_cost += solution.get_cost(past_node, inserted_node) + solution.get_cost(next_node, inserted_node);
                    }
                    if (temp_cost < new_cost) {
                        position = i;
                        k_index = k;
                        new_cost = temp_cost;
                        unassigned_index = un;
                    }
                }
            }
        }
        solution.insert_node(k_index, un_nodes[unassigned_index], position);
        unassign(solution.get_tour(k_index), un_nodes);
    }
    return solution.get_total_obj();
}


uint32_t assign_garage(MTSPBC &solution, std::vector<size_t>& un_nodes) {

    std::optional<uint32_t> vehicle_at_depot { std::nullopt };


    for (auto h{ 0 }; h < solution.get_k_vehicles(); h++) {
        auto pos { solution.get_pos_for_node(h, 0) };
        if (pos) {
            vehicle_at_depot = h;
            break;
        }
    }


    for (uint32_t k{}; k < solution.get_k_vehicles(); k++) {
        if (k == vehicle_at_depot.value()) {
            continue;
        }
        uint32_t new_cost{ 999999 };
        uint32_t position{};
        for (uint32_t i{ 0 }; i < solution.get_tour(k).size(); i++) {
            std::vector<uint32_t> insertion_hull { solution.get_tour(k) };
            insertion_hull.insert(insertion_hull.begin() + i, 0);
            size_t past_node {};
            if (i > 0) {
                past_node = insertion_hull.at(i - 1);
            }
            size_t next_node { insertion_hull.at(i) };
            insertion_hull.insert(insertion_hull.begin() + i, 0);
            uint32_t temp_cost { solution.get_obj_vehicle(k) };
            if (i == 0) {
                temp_cost += solution.get_cost(next_node, 0);
            }
            else {
                temp_cost += solution.get_cost(past_node, 0) + solution.get_cost(next_node, 0);
            }
            if (temp_cost < new_cost) {
                position = i;
                new_cost = temp_cost;
            }
        }
        solution.insert_node(k, 0, position);
    }
    if (!un_nodes.empty())
        if (un_nodes[0] == 0)
            un_nodes.erase(un_nodes.begin());
    return solution.get_total_obj();
}
// std::vector<uint32_t> remove_covered_nodes(std::vector<uint32_t> hull);   // remove node from convex hull if the remaining hull covers it

#include "MTSPBC_chh.hpp"
#include "MTSPBC.hpp"
#include "Cht.hpp"
#include "MTSPBC_util.hpp"

#include <cstddef>
#include <cstdint>
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

// std::vector<std::vector<uint32_t>> find_onion_hull();    // find the hull for every vehicle
// std::vector<uint32_t> find_route();                      // find the route for one vehicle
// std::vector<std::vector<uint32_t>> find_solution();      // find heuristic solution
// std::vector<uint32_t> remove_covered_nodes(std::vector<uint32_t> hull);   // remove node from convex hull if the remaining hull covers it

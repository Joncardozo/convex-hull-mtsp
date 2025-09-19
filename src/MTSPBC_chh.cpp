#include "MTSPBC_chh.hpp"
#include "MTSPBC.hpp"
#include "Cht.hpp"
#include "MTSPBCInstance.hpp"
#include "MTSPBC_util.hpp"
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <sys/types.h>
#include <vector>
#include <algorithm>


uint32_t add_convex_hull(MTSPBC& solution, const uint32_t vehicle, std::vector<size_t>& un_nodes, const MTSPBCInstance& instance) {              // find the hull for one vehicle
    Cht tour;
    // Find the leftmost unassigned node
    size_t point_left_most_i = un_nodes[0];
    for (size_t idx = 1; idx < un_nodes.size(); idx++) {
        Coord left_most_coord = instance.coordinate(point_left_most_i);
        Coord current_point = instance.coordinate(un_nodes[idx]);
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
    point_left_most.pos = instance.coordinate(point_left_most_i);
    for (size_t i = 0; i < un_nodes.size(); i++) {
        size_t idx = un_nodes[i];
        if (idx != point_left_most_i) {
            Nodes n;
            n.index = idx;
            n.pos = instance.coordinate(idx);
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


uint32_t find_onion_hull(MTSPBC& solution, std::vector<size_t>& un_nodes, const MTSPBCInstance& instance) {

    uint32_t k_vehicles { solution.get_k_vehicles() };

    // iterativamente, encontra uma rota para cada veículo
    for (uint32_t i{ 0 }; i < k_vehicles; i++) {
        add_convex_hull(solution, i, un_nodes, instance);
        unassign(solution.get_tour(i), un_nodes);
    }
    return 0;
}


uint32_t cheapest_insertion(MTSPBC& solution, std::vector<size_t>& un_nodes, const MTSPBCInstance& instance, const bool closed_tour) {      // find heuristic solution
    if (solution.get_total_obj() == 0) {
        throw std::logic_error("error: cheapest heuristic over empty solution not allowed");
    }
    while(un_nodes.size() > 0) {
        uint32_t k_index {};
        uint32_t position {};
        uint32_t new_cost { 999999 };
        uint32_t unassigned_index {};
        for (auto un { 0 }; un < un_nodes.size(); un++) {
            // uint32_t curr_best_obj { 999999 };
            // uint32_t curr_best_k { 0 };
            for (auto k{ 0 }; k < solution.get_k_vehicles(); k++) {
                // if (solution.get_obj_vehicle(k) < curr_best_obj) {
                //     curr_best_obj = solution.get_obj_vehicle(k);
                //     curr_best_k = k;
                // }
                uint32_t closed_i = (closed_tour) ? 1 : 0;
                for (auto i{ closed_i }; i < solution.get_tour(k).size() - closed_i; i++) {
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
                    if (temp_cost < new_cost) { // && curr_best_k == k) {
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
    for (uint32_t i { 0 }; i < solution.get_k_vehicles(); i++) {
        solution.reverse_tour(i);
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
        if (vehicle_at_depot){
            if (k == vehicle_at_depot.value()) {
                continue;
            }
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


uint32_t close_tours(MTSPBC& solution) {
    for (uint32_t i { 0 }; i < solution.get_k_vehicles(); i++) {
        if (solution.get_complete_tour(i)) {
            continue;
        }
        auto depot_pos { solution.get_pos_for_node(i, 0) };
        if (!depot_pos) {
            throw std::logic_error("error: no depot assigned");
        }
        size_t remove_nodes { depot_pos.value() };
        std::vector<uint32_t> first_part {};
        for (uint32_t j{ 0 }; j < remove_nodes; j++) {
            first_part.push_back(solution.get_node_at_pos(i, 0));
            solution.remove_node(i, 0);
        }
        for (uint32_t j{ 0 }; j < first_part.size(); j++) {
            solution.push_back(i, first_part.at(j));
        }
        solution.push_back(i, 0);
    }
    return 0;
}


uint32_t remove_covered_nodes(MTSPBC& solution, const MTSPBCInstance& instance, const uint32_t vehicle, std::vector<size_t>& un_nodes) {
    std::vector<uint32_t> tour { solution.get_tour(vehicle) };
    if (tour.size() <= 3)
        throw std::logic_error("error: tour is too short (n <= 3)");
    uint32_t depart_node_i { 0 };
    uint32_t arrival_node_i { 1 };
    std::optional<uint32_t> check_node{};
    check_node.reset();
    bool minimal_tour { false };
    bool no_change { false };
    bool at_the_end { false };
    uint32_t in_between_nodes { 0 };
    std::vector<uint32_t> remove_nodes{};
    while (!minimal_tour) {
        if (tour.size() - remove_nodes.size() == 3) {
            minimal_tour = true;
            break;
        }
        if (in_between_nodes == 0 && arrival_node_i != depart_node_i) {
            check_node = arrival_node_i;
            arrival_node_i = (arrival_node_i + 1) % tour.size();
            in_between_nodes += 1;
            continue;
        }
        if (check_node) {
            double LB { instance.get_LB(tour.at(check_node.value()), tour.at(depart_node_i), tour.at(arrival_node_i)) };
            double UB { instance.get_UB(tour.at(check_node.value()), tour.at(depart_node_i), tour.at(arrival_node_i)) };
            if (UB < LB) {
                if (at_the_end) break;
                if (arrival_node_i < depart_node_i) break;
                depart_node_i = arrival_node_i;
                arrival_node_i = (arrival_node_i + 1) % tour.size();
                check_node.reset();
                at_the_end = (depart_node_i == tour.size() - 1) ? true : false;
                in_between_nodes = 0;
            }
            else {
                remove_nodes.push_back(tour.at(check_node.value()));
                check_node = arrival_node_i;
                arrival_node_i = (arrival_node_i + 1) % tour.size();
                in_between_nodes++;
                continue;
            }
        }
        else {
            minimal_tour = true;
        }
    }
    for (auto n : remove_nodes) {
        std::optional<size_t> pos { solution.get_pos_for_node(vehicle, n) };
        if (pos) solution.remove_node(vehicle, pos.value());
    }
    un_nodes.insert(un_nodes.end(), remove_nodes.begin(), remove_nodes.end());
    std::sort(un_nodes.begin(), un_nodes.end());
    return 0;
}


uint32_t maxd_best_3opt(MTSPBC& solution, const MTSPBCInstance& instance) {
    uint32_t best_k_rem { };
    uint32_t best_n_rem { };
    uint32_t best_k_ins { };
    uint32_t best_pos_ins { };
    uint32_t max_distance { solution.get_max_distance() };
    bool has_improved { false };
    while (has_improved) {
        has_improved = false;
        for (uint32_t i { }; i < solution.get_k_vehicles(); i++) {
            for (uint32_t j { 1 }; j < solution.get_tour(i).size() - 1; j++) {
                for (uint32_t k { }; k < solution.get_k_vehicles(); k++) {
                    if (i == k) continue;
                    for (uint32_t l { 1 }; l < solution.get_tour(k).size() - 1; l++) {
                        uint32_t removed_node { solution.get_node_at_pos(i, j) };
                        solution.remove_node(i, j);
                        solution.insert_node(k, removed_node, l);
                        uint32_t tmp_max_distance { solution.get_max_distance() };
                        if (tmp_max_distance < max_distance) {
                            max_distance = tmp_max_distance;
                            has_improved = true;
                        } else {
                            has_improved |= false;
                            solution.remove_node(k, l);
                            solution.insert_node(i, removed_node, j);
                        }
                    }
                }
            }
        }
    }
    return max_distance;
}

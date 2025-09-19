#include "MTSPBC_algorithm.hpp"
#include "MTSPBC.hpp"
#include "MTSPBCInstance.hpp"
#include "MTSPBC_ds.hpp"
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <sys/types.h>
#include <iostream>
#include <utility>


uint32_t opt_2(MTSPBC& solution, const uint32_t k, Edge edge_1, Edge edge_2) {

}


uint32_t opt_3(MTSPBC& solution, const uint32_t k1, Edge k1_1, Edge k1_2, Edge k2_1) {

}


uint32_t opt_4(MTSPBC& solution, const uint32_t k1, const uint32_t k2, Edge k1_1, Edge k1_2, Edge k2_1, Edge k2_2) {

}


bool opt_3_min_dist_event(MTSPBC& solution, const MTSPBCInstance& instance, const uint32_t k_1, const uint32_t k_2, const Edge k_1_edge, const uint32_t k_2_n_i) {
    int old_e_dist { std::accumulate(solution.get_distances().begin(), solution.get_distances().end(), 0) };
    uint32_t k2_remove_node { solution.get_node_at_pos(k_2, k_2_n_i) };
    uint32_t k1_insert_pos { k_1_edge.node_B.first };
    solution.insert_node(k_1, k2_remove_node, k1_insert_pos);
    solution.remove_node(k_2, k_2_n_i);
    int new_e_dist { std::accumulate(solution.get_distances().begin(), solution.get_distances().end(), 0) };
    if (new_e_dist < old_e_dist) {
        return true;
    }
    else {
        solution.insert_node(k_2, k2_remove_node, k_2_n_i);
        solution.remove_node(k_1, k1_insert_pos);
        return false;
    }
}


void minimize_e_dist(MTSPBC& solution, const MTSPBCInstance& instance) {
    bool has_improved { true };
    int32_t stop_improv { 100 };
    begin_loop :
    while (has_improved && stop_improv > 0) {
        has_improved = false;
        std::cout << stop_improv << std::endl;
        for (uint32_t e_index { 5 }; e_index < solution.get_n_events(); e_index++) {
            uint32_t old_dist { solution.dist_at_event(e_index) };
            std::pair<uint32_t, uint32_t> event { solution.get_event(e_index) };
            uint32_t e_node { solution.get_node_at_event(event.second, event.first) };
            if (e_node == 0) continue;
            for (uint32_t k_1 {}; k_1 < solution.get_k_vehicles(); k_1++) {
                if (k_1 == event.second) continue;
                Edge k_1_edge { solution.edge_at_event(k_1, event.first) };
                uint32_t event_index { solution.event_index(event.second, event.first) };
                bool improvement { opt_3_min_dist_event(solution, instance, k_1, event.second, k_1_edge, event_index) };
                if (improvement) {
                    has_improved = true;
                    stop_improv--;
                    goto begin_loop;
                    break;
                }
            }
        }
    }
}


void minimize_e_dist_2(MTSPBC& solution, const MTSPBCInstance& instance) {
    bool has_improved { true };
    int32_t stop_improv { 1000 };
    begin_loop :
    while (has_improved && stop_improv > 0) {
        has_improved = false;
        std::cout << stop_improv << std::endl;
        for (uint32_t i { }; i < solution.get_k_vehicles(); i++) {
            for (uint32_t j { }; j < solution.get_k_vehicles(); j++) {
                if (i == j) continue;
                for (uint32_t m { 1 }; m < solution.n_nodes(i) - 1; m++) {
                    for (uint32_t n { 0 }; n < solution.n_nodes(j) - 1; n++) {
                        Edge i_e1 { solution.edge(i, n) };
                        bool improvement { opt_3_min_dist_event(solution, instance, i, j, i_e1, m) };
                        if (improvement) {
                            has_improved = true;
                            goto begin_loop;
                        }
                    }
                }
            }
        }
    }
}

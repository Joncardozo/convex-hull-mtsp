#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>
#include "Cht.hpp"


class MTSPBC {
    private:
    uint32_t total_obj_;
    uint32_t k_vehicles_;
    uint32_t n_nodes_;
    uint32_t r_radius_;
    std::vector<Cht> tours_;
    std::vector<std::vector<uint32_t>> dist_matrix_;
    std::vector<std::pair<uint32_t, uint32_t>> events_;
    std::vector<uint32_t> max_distance_events_;
    bool feasible_;
    uint32_t max_distance_value_;

    uint32_t compute_obj_();
    uint32_t collect_events_();
    uint32_t compute_max_distances_(uint32_t changed_e_index);
    uint32_t compute_max_distances_();
    bool check_feasibility_();



    public:
    MTSPBC();
    // MSTPBC methods
    uint32_t create_distance_matrix(const std::vector<std::vector<uint32_t>>& matrix);
    uint32_t create_vehicle();
    uint32_t remove_vehicle(const uint32_t vehicle_index);
    uint32_t get_total_obj();
    std::vector<std::pair<uint32_t, uint32_t>> get_events() const;
    bool get_feasibility();
    uint32_t get_max_distance();
    uint32_t get_max_distance_vehicle();
    std::vector<uint32_t> get_distances();
    uint32_t get_n_nodes();
    uint32_t get_k_vehicles();
    uint32_t get_r_radius();

    // Cht wrapper methods
    uint32_t insert_node(const uint32_t vehicle, const uint32_t node, const size_t pos);
    uint32_t remove_node(const uint32_t vehicle, const size_t pos);
    uint32_t push_back(const uint32_t vehicle, const uint32_t node);
    uint32_t push_front(const uint32_t vehicle, const uint32_t node);
    uint32_t pop_back(const uint32_t vehicle);
    uint32_t pop_front(const uint32_t vehicle);
    uint32_t get_obj_vehicle(const uint32_t vehicle) const;
    std::vector<uint32_t> get_tour(const uint32_t vehicle) const;
    size_t get_pos_for_node(const uint32_t vehicle, const uint32_t node);
    uint32_t get_node_at_pos(const uint32_t vehicle, const size_t pos);
    bool get_complete_tour(const uint32_t vehicle) const;
};

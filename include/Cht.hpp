#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

class Cht {
    private:
        uint32_t obj_;
        std::vector<uint32_t> tour_;
        std::vector<uint32_t> events_;
        bool complete_tour_;                    // set true it tour starts and ends at depot (0 node)
        uint32_t compute_events_(const std::vector<std::vector<uint32_t>>& dist_matrix);
        uint32_t compute_obj_insert_(size_t pos_A, size_t pos_B, size_t pos_C, const std::vector<std::vector<uint32_t>>& dist_matrix);
        uint32_t compute_obj_insert_(const bool at_end, const std::vector<std::vector<uint32_t>>& dist_matrix);
        uint32_t compute_obj_remove_(size_t pos_A, size_t pos_B, size_t pos_C, const std::vector<std::vector<uint32_t>>& dist_matrix);
        uint32_t compute_obj_remove_(const bool at_end, const std::vector<std::vector<uint32_t>>& dist_matrix);

    public:
        Cht();
        uint32_t insert_node(const uint32_t node, const size_t pos, const std::vector<std::vector<uint32_t>>& dist_matrix);
        uint32_t remove_node(const size_t pos, const std::vector<std::vector<uint32_t>>& dist_matrix);
        uint32_t push_back(const uint32_t node, const std::vector<std::vector<uint32_t>>& dist_matrix);
        uint32_t push_front(const uint32_t node, const std::vector<std::vector<uint32_t>>& dist_matrix);
        uint32_t pop_back(const std::vector<std::vector<uint32_t>>& dist_matrix);
        uint32_t pop_front(const std::vector<std::vector<uint32_t>>& dist_matrix);
        uint32_t get_obj() const;
        std::vector<uint32_t> get_tour() const;
        size_t get_pos_for_node(const uint32_t node);
        uint32_t get_node_at_pos(const size_t pos);
        size_t n_nodes() const;
        std::vector<uint32_t> get_events() const;
        bool check_complete_tour_();
        bool get_complete() const;
};

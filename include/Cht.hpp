#pragma once


#include "MTSPBCInstance.hpp"
#include "MTSPBC_ds.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>
#include <optional>


class Cht {
    private:
        uint32_t obj_;
        std::vector<uint32_t> tour_;
        std::vector<uint32_t> events_;
        bool complete_tour_;                    // set true it tour starts and ends at depot (0 node)
        uint32_t compute_events_(const MTSPBCInstance& instance);
        // uint32_t compute_events_(const MTSPBCInstance& instance);
        uint32_t compute_events_(const uint32_t inserted_pos, const MTSPBCInstance& instance);
        uint32_t compute_obj_insert_(size_t pos_A, size_t pos_B, size_t pos_C, const MTSPBCInstance& instance);
        uint32_t compute_obj_insert_(const bool at_end, const MTSPBCInstance& instance);
        uint32_t compute_obj_insert_(const uint32_t pos_i, const uint32_t pos_e, const MTSPBCInstance& instance);
        uint32_t compute_obj_remove_(size_t pos_A, size_t pos_B, size_t pos_C, const MTSPBCInstance& instance);
        uint32_t compute_obj_remove_(const bool at_end, const MTSPBCInstance& instance);
        uint32_t compute_obj_remove_(const uint32_t pos_i, const uint32_t pos_e, const MTSPBCInstance& instance);

    public:
        Cht();
        uint32_t insert_node(const uint32_t node, const size_t pos, const MTSPBCInstance& instance);
        uint32_t remove_node(const size_t pos, const MTSPBCInstance& instance);
        uint32_t push_back(const uint32_t node, const MTSPBCInstance& instance);
        uint32_t push_front(const uint32_t node, const MTSPBCInstance& instance);
        uint32_t pop_back(const MTSPBCInstance& instance);
        uint32_t pop_front(const MTSPBCInstance& instance);
        uint32_t insert_subtour(const MTSPBCInstance& instance, const std::vector<uint32_t>& subtour_indices, const uint32_t pos_i, const uint32_t pos_e);
        uint32_t replace_subtour(const MTSPBCInstance& instance, const std::vector<uint32_t>& subtour_indices, const uint32_t pos_i, const uint32_t pos_e);
        uint32_t remove_subtour(const MTSPBCInstance& instance, const uint32_t pos_i, const uint32_t pos_e);
        uint32_t reverse_subtour(const MTSPBCInstance& instance, const uint32_t pos_i, const uint32_t pos_e);
        uint32_t reverse_tour(const MTSPBCInstance& instance);
        [[nodiscard]] uint32_t get_obj() const noexcept;
        [[nodiscard]] std::vector<uint32_t> get_tour() const noexcept;
        [[nodiscard]] std::optional<size_t> get_pos_for_node(const uint32_t node) const;
        [[nodiscard]] uint32_t get_node_at_pos(const size_t pos) const;
        [[nodiscard]] size_t n_nodes() const noexcept;
        [[nodiscard]] uint32_t n_events() const noexcept;
        [[nodiscard]] std::vector<uint32_t> get_events() const noexcept;
        [[nodiscard]] bool get_complete() const noexcept;
        [[nodiscard]] std::optional<uint32_t> get_node_at_event(const uint32_t e_time) const;
        [[nodiscard]] Edge edge(const uint32_t edge_i) const;
        [[nodiscard]] Edge edge_at_event(const uint32_t e_time) const;
        [[nodiscard]] uint32_t event_index(const uint32_t e_time) const;
        bool check_complete_tour_();
};

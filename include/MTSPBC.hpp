#pragma once


#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>
#include "Cht.hpp"
#include "MTSPBC_ds.hpp"
#include "MTSPBCInstance.hpp"


class MTSPBC {
    private:
    const MTSPBCInstance& instance_;
    uint32_t total_obj_;
    uint32_t k_vehicles_;
    uint32_t n_nodes_;
    uint32_t r_radius_;
    std::vector<Cht> tours_;
    std::vector<std::pair<uint32_t, uint32_t>> events_;
    std::vector<uint32_t> max_distance_events_;
    bool feasible_;
    uint32_t max_distance_value_;
    uint32_t compute_obj_();
    uint32_t collect_events_();
    uint32_t collect_events_(const uint32_t& vehicle, const uint32_t& node_index);
    uint32_t compute_max_distances_(uint32_t changed_e_index);
    uint32_t compute_max_distances_();
    bool check_feasibility_();


    public:
    MTSPBC(const MTSPBCInstance& instance);
    // MSTPBC methods
    uint32_t create_vehicle();
    uint32_t remove_vehicle(const uint32_t vehicle_index);
    uint32_t set_radius(const uint32_t r_radius);
    void save_solution(const std::string& filepath, const std::string& tour_filepath);
    [[nodiscard]] uint32_t get_total_obj() const noexcept;
    [[nodiscard]] std::vector<std::pair<uint32_t, uint32_t>> get_events() const noexcept;
    [[nodiscard]] bool get_feasibility() const noexcept;
    [[nodiscard]] uint32_t get_max_distance() const noexcept;
    [[nodiscard]] std::vector<uint32_t> get_distances() const noexcept;
    [[nodiscard]] uint32_t get_n_nodes() const noexcept;
    [[nodiscard]] uint32_t get_k_vehicles() const noexcept;
    [[nodiscard]] uint32_t get_r_radius() const noexcept;
    [[nodiscard]] uint32_t get_cost(const uint32_t node_A, const uint32_t node_B) const;
    [[nodiscard]] std::pair<uint32_t, uint32_t> get_event(const uint32_t e_index) const;
    [[nodiscard]] uint32_t get_n_events() const noexcept;
    [[nodiscard]] Coord get_coord(uint32_t node) const;

    // Cht wrapper methods
    uint32_t insert_node(const uint32_t vehicle, const uint32_t node, const size_t pos);
    uint32_t remove_node(const uint32_t vehicle, const size_t pos);
    uint32_t insert_subtour(const uint32_t vehicle, const std::vector<uint32_t>& subtour_indices, const uint32_t pos_i, const uint32_t pos_e);
    uint32_t replace_subtour(const uint32_t vehicle, const std::vector<uint32_t>& subtour_indices, const uint32_t pos_i, const uint32_t pos_e);
    uint32_t remove_subtour(const uint32_t vehicle, const uint32_t pos_i, const uint32_t pos_e);
    uint32_t reverse_subtour(const uint32_t vehicle, const uint32_t pos_i, const uint32_t pos_e);
    uint32_t push_back(const uint32_t vehicle, const uint32_t node);
    uint32_t push_front(const uint32_t vehicle, const uint32_t node);
    uint32_t pop_back(const uint32_t vehicle);
    uint32_t pop_front(const uint32_t vehicle);
    uint32_t reverse_tour(const uint32_t vehicle);
    [[nodiscard]] uint32_t get_obj_vehicle(const uint32_t vehicle) const;
    [[nodiscard]] std::vector<uint32_t> get_tour(const uint32_t vehicle) const;
    [[nodiscard]] std::optional<size_t> get_pos_for_node(const uint32_t vehicle, const uint32_t node) const;
    [[nodiscard]] uint32_t get_node_at_pos(const uint32_t vehicle, const size_t pos) const;
    [[nodiscard]] uint32_t get_node_at_event(const uint32_t vehicle, const uint32_t e_time) const;
    [[nodiscard]] bool get_complete_tour(const uint32_t vehicle) const;
    [[nodiscard]] std::vector<uint32_t> get_vehicle_events(const uint32_t vehicle) const;
};

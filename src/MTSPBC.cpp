#include "MTSPBC.hpp"
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <algorithm>
#include <utility>
#include <vector>


// constructor
MTSPBC::MTSPBC() {
    total_obj_ = 0;
    n_nodes_ = 0;
    k_vehicles_ = 0;
    r_radius_ = 0;
    feasible_ = false;
}


// MSTPBC methods
//
//
uint32_t MTSPBC::create_distance_matrix(const std::vector<std::vector<uint32_t>>& matrix) {
    dist_matrix_ = matrix;
    return 0;
}


// create vehicle, returns total vehicles
uint32_t MTSPBC::create_vehicle() {
    Cht new_vehicle;
    tours_.push_back(new_vehicle);
    k_vehicles_ = tours_.size();
    return k_vehicles_;
}


uint32_t MTSPBC::remove_vehicle(const uint32_t vehicle_index) {
    if (k_vehicles_ - 1 < vehicle_index) {
        throw std::logic_error("error: vehicle not found");
    }
    auto erase_it { tours_.begin() + vehicle_index };
    tours_.erase(erase_it);
    k_vehicles_ = tours_.size();
    return k_vehicles_;
}


uint32_t MTSPBC::set_radius(const uint32_t r_radius) {
    r_radius_ = r_radius;
    return r_radius;
}


uint32_t MTSPBC::collect_events_() {
    events_.clear();
    for (uint32_t t { 0 }; t < tours_.size(); t++) {
        std::vector<uint32_t> tour_e { tours_.at(t).get_events() };
        std::vector<std::pair<uint32_t, uint32_t>> tour_e_pair {  };
        for (uint32_t i { 0 }; i < tour_e.size(); i++) {
            tour_e_pair.push_back(std::make_pair(tour_e.at(i), t));
        }
        events_.insert(events_.begin(), tour_e_pair.begin(), tour_e_pair.begin() + tour_e_pair.size());
    }
    std::sort(events_.begin(), events_.end());
    return events_.size();
}


uint32_t collect_events_(const uint32_t& vehicle, const uint32_t& node_index) {

    return 0;
}


uint32_t MTSPBC::compute_max_distances_() {

    return 0;
}


//getters
[[nodiscard]] uint32_t MTSPBC::get_total_obj() const noexcept { return total_obj_; }
[[nodiscard]] bool MTSPBC::get_feasibility() const noexcept { return feasible_; }
[[nodiscard]] uint32_t MTSPBC::get_max_distance() const noexcept { return max_distance_value_; }
[[nodiscard]] std::vector<std::pair<uint32_t, uint32_t>> MTSPBC::get_events() const noexcept { return events_;}
[[nodiscard]] std::vector<uint32_t> MTSPBC::get_distances() const noexcept { return max_distance_events_; }
[[nodiscard]] uint32_t MTSPBC::get_n_nodes() const noexcept { return n_nodes_; }
[[nodiscard]] uint32_t MTSPBC::get_k_vehicles() const noexcept { return k_vehicles_; }
[[nodiscard]] uint32_t MTSPBC::get_r_radius() const noexcept { return r_radius_; }
[[nodiscard]] uint32_t MTSPBC::get_cost(const uint32_t node_A, const uint32_t node_B) const { return dist_matrix_.at(node_A).at(node_B); }
[[nodiscard]] std::pair<uint32_t, uint32_t> MTSPBC::get_event(const uint32_t e_index) const {
    if (e_index > events_.size() - 1) {
        throw std::logic_error("error: event out of range");
    }
    return events_.at(e_index);
}
[[nodiscard]] uint32_t MTSPBC::get_n_events() const noexcept { return events_.size(); }


// Cht wrapper methods
uint32_t MTSPBC::insert_node(const uint32_t vehicle, const uint32_t node, const size_t pos) {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    uint32_t old_obj { tours_.at(vehicle).get_obj() };
    uint32_t new_obj { tours_.at(vehicle).insert_node(node, pos, dist_matrix_) };
    total_obj_ += new_obj - old_obj;
    collect_events_();
    return total_obj_;
}


uint32_t MTSPBC::remove_node(const uint32_t vehicle, const size_t pos) {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    uint32_t old_obj { tours_.at(vehicle).get_obj() };
    uint32_t new_obj { tours_.at(vehicle).remove_node(pos, dist_matrix_) };
    total_obj_ -= old_obj - new_obj;
    collect_events_();
    return total_obj_;
}


uint32_t MTSPBC::push_back(const uint32_t vehicle, const uint32_t node) {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    uint32_t old_obj { tours_.at(vehicle).get_obj() };
    uint32_t new_obj { tours_.at(vehicle).push_back(node, dist_matrix_) };
    total_obj_ += new_obj - old_obj;
    collect_events_();
    return total_obj_;
}


uint32_t MTSPBC::push_front(const uint32_t vehicle, const uint32_t node) {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    uint32_t old_obj { tours_.at(vehicle).get_obj() };
    uint32_t new_obj { tours_.at(vehicle).push_front(node, dist_matrix_) };
    total_obj_ += new_obj - old_obj;
    collect_events_();
    return total_obj_;
}


uint32_t MTSPBC::pop_back(const uint32_t vehicle) {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    uint32_t old_obj { tours_.at(vehicle).get_obj() };
    uint32_t new_obj { tours_.at(vehicle).pop_back(dist_matrix_) };
    total_obj_ -= old_obj - new_obj;
    collect_events_();
    return total_obj_;
}


uint32_t MTSPBC::pop_front(const uint32_t vehicle) {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    uint32_t old_obj { tours_.at(vehicle).get_obj() };
    uint32_t new_obj { tours_.at(vehicle).pop_front(dist_matrix_) };
    total_obj_ -= old_obj - new_obj;
    collect_events_();
    return total_obj_;
}


[[nodiscard]] uint32_t MTSPBC::get_obj_vehicle(const uint32_t vehicle) const {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    return tours_.at(vehicle).get_obj();
}


[[nodiscard]] std::vector<uint32_t> MTSPBC::get_tour(const uint32_t vehicle) const {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    return tours_.at(vehicle).get_tour();
}


[[nodiscard]] std::optional<size_t> MTSPBC::get_pos_for_node(const uint32_t vehicle, const uint32_t node) const {
    auto rv { tours_.at(vehicle).get_pos_for_node(node) };
    if (!rv) {
        return std::nullopt;
    }
    return rv.value();
}


[[nodiscard]] uint32_t MTSPBC::get_node_at_pos(const uint32_t vehicle, const size_t pos) const {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    return tours_.at(vehicle).get_node_at_pos(pos);
}


[[nodiscard]] bool MTSPBC::get_complete_tour(const uint32_t vehicle) const {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    return tours_.at(vehicle).get_complete();
}


[[nodiscard]] std::vector<uint32_t> MTSPBC::get_vehicle_events(const uint32_t vehicle) const {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    return tours_.at(vehicle).get_events();
}

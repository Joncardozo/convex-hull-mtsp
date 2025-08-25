#include "MTSPBC.hpp"
#include "MTSPBCInstance.hpp"
#include "MTSPBC_ds.hpp"
#include "MTSPBC_util.hpp"
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <algorithm>
#include <utility>
#include <vector>
#include <fstream>


// constructor
MTSPBC::MTSPBC(const MTSPBCInstance& instance)
: instance_(instance)
{
    total_obj_ = 0;
    n_nodes_ = 0;
    k_vehicles_ = 0;
    r_radius_ = 0;
    feasible_ = false;
}


// MSTPBC methods

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
    compute_max_distances_();
    return events_.size();
}


uint32_t collect_events_(const uint32_t& vehicle, const uint32_t& node_index) {

    return 0;
}


uint32_t MTSPBC::compute_max_distances_() {
    max_distance_events_.clear();
    uint32_t last_e { 0 };
    for (uint32_t i { 0 }; i < events_.size(); i++) {
        if (last_e == i && i != 0) {
            continue;
        }
        auto e_vehicle { events_.at(i).second };
        uint32_t curr_distance { 0 };
        for (uint32_t k { 0 }; k < k_vehicles_; k++) {
            if (k == e_vehicle || tours_.at(k).get_tour().size() < 2) {
                continue;
            }
            uint32_t tmp_distance { distance(*this, i, k) };
            if (tmp_distance > curr_distance) {
                curr_distance = tmp_distance;
            }
        }
        last_e = i;
        max_distance_events_.push_back(curr_distance);
    }
    return 0;
}


void MTSPBC::save_solution(const std::string& points_filepath, const std::string& tour_filepath) {
    std::ofstream f(filepath);
    for (uint32_t i { 0 }; i < instance_.n(); i++) {

        f << instance_.coordinate(i).pos_x << " " << instance_.coordinate(i).pos_y << std::endl;
    }
    f.close();

    std::ofstream hull_file("hull.dat");
    for (auto h : onion_hull) {
        for (uint32_t idx : h) {
            hull_file << coordinates[idx].pos_x << " " << coordinates[idx].pos_y << "\n";
        }
        hull_file << coordinates[h[0]].pos_x << " " << coordinates[h[0]].pos_y << "\n";
        hull_file << std::endl << std::endl;
    }

    // Close the hull
    hull_file.close();
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
[[nodiscard]] uint32_t MTSPBC::get_cost(const uint32_t node_A, const uint32_t node_B) const { return instance_.cost(node_A, node_B); }
[[nodiscard]] std::pair<uint32_t, uint32_t> MTSPBC::get_event(const uint32_t e_index) const {
    if (e_index > events_.size() - 1) {
        throw std::logic_error("error: event out of range");
    }
    return events_.at(e_index);
}
[[nodiscard]] uint32_t MTSPBC::get_n_events() const noexcept { return events_.size(); }
[[nodiscard]] Coord MTSPBC::get_coord(uint32_t node) const {
    if (node > instance_.n() - 1) {
        throw std::out_of_range("error: node does not exist");
    }
    return instance_.coordinate(node);
}


// Cht wrapper methods
uint32_t MTSPBC::insert_node(const uint32_t vehicle, const uint32_t node, const size_t pos) {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicles do not exist");
    }
    uint32_t old_obj { tours_.at(vehicle).get_obj() };
    uint32_t new_obj { tours_.at(vehicle).insert_node(node, pos, instance_) };
    total_obj_ += new_obj - old_obj;
    collect_events_();
    return total_obj_;
}


uint32_t MTSPBC::remove_node(const uint32_t vehicle, const size_t pos) {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    uint32_t old_obj { tours_.at(vehicle).get_obj() };
    uint32_t new_obj { tours_.at(vehicle).remove_node(pos, instance_) };
    total_obj_ -= old_obj - new_obj;
    collect_events_();
    return total_obj_;
}


uint32_t MTSPBC::push_back(const uint32_t vehicle, const uint32_t node) {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    uint32_t old_obj { tours_.at(vehicle).get_obj() };
    uint32_t new_obj { tours_.at(vehicle).push_back(node, instance_) };
    total_obj_ += new_obj - old_obj;
    collect_events_();
    return total_obj_;
}


uint32_t MTSPBC::push_front(const uint32_t vehicle, const uint32_t node) {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    uint32_t old_obj { tours_.at(vehicle).get_obj() };
    uint32_t new_obj { tours_.at(vehicle).push_front(node, instance_) };
    total_obj_ += new_obj - old_obj;
    collect_events_();
    return total_obj_;
}


uint32_t MTSPBC::pop_back(const uint32_t vehicle) {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    uint32_t old_obj { tours_.at(vehicle).get_obj() };
    uint32_t new_obj { tours_.at(vehicle).pop_back(instance_) };
    total_obj_ -= old_obj - new_obj;
    collect_events_();
    return total_obj_;
}


uint32_t MTSPBC::pop_front(const uint32_t vehicle) {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    uint32_t old_obj { tours_.at(vehicle).get_obj() };
    uint32_t new_obj { tours_.at(vehicle).pop_front(instance_) };
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
        throw std::logic_error("error: vehicle does not exist");
    }
    return tours_.at(vehicle).get_node_at_pos(pos);
}


[[nodiscard]] uint32_t MTSPBC::get_node_at_event(const uint32_t vehicle, const uint32_t e_time) const {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle does not exist");
    }
    auto index { tours_.at(vehicle).get_node_at_event(e_time) };
    if (!index) {
        throw std::logic_error("error: event not found");
    }
    return index.value();
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

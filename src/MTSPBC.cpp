#include "MTSPBC.hpp"
#include <cstdint>
#include <stdexcept>


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


uint32_t MTSPBC::get_total_obj() {
    return total_obj_;
}


std::vector<std::pair<uint32_t, uint32_t>> MTSPBC::get_events() const {
    return events_;
}


bool MTSPBC::get_feasibility() {
    return feasible_;
}


uint32_t MTSPBC::get_max_distance() {
    return max_distance_value_;
}


std::vector<uint32_t> MTSPBC::get_distances() {
    return max_distance_events_;
}


uint32_t MTSPBC::get_n_nodes() {
    return n_nodes_;
}


uint32_t MTSPBC::get_k_vehicles() {
    return k_vehicles_;
}


uint32_t MTSPBC::get_r_radius() {
    return r_radius_;
}


// Cht wrapper methods
uint32_t MTSPBC::insert_node(const uint32_t vehicle, const uint32_t node, const size_t pos) {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    total_obj_ += tours_.at(vehicle).insert_node(node, pos, dist_matrix_);
    return total_obj_;
}


uint32_t MTSPBC::remove_node(const uint32_t vehicle, const size_t pos) {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    uint32_t old_obj { tours_.at(vehicle).get_obj() };
    uint32_t new_obj { tours_.at(vehicle).remove_node(pos, dist_matrix_) };
    total_obj_ -= old_obj - new_obj;
    return total_obj_;
}


uint32_t MTSPBC::push_back(const uint32_t vehicle, const uint32_t node) {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    uint32_t old_obj { tours_.at(vehicle).get_obj() };
    uint32_t new_obj { tours_.at(vehicle).push_back(node, dist_matrix_) };
    total_obj_ += new_obj - old_obj;
    return total_obj_;
}


uint32_t MTSPBC::push_front(const uint32_t vehicle, const uint32_t node) {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    uint32_t old_obj { tours_.at(vehicle).get_obj() };
    uint32_t new_obj { tours_.at(vehicle).push_front(node, dist_matrix_) };
    total_obj_ += new_obj - old_obj;
    return total_obj_;
}


uint32_t MTSPBC::pop_back(const uint32_t vehicle) {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    uint32_t old_obj { tours_.at(vehicle).get_obj() };
    uint32_t new_obj { tours_.at(vehicle).pop_back(dist_matrix_) };
    total_obj_ -= old_obj - new_obj;
    return total_obj_;
}


uint32_t MTSPBC::pop_front(const uint32_t vehicle) {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    uint32_t old_obj { tours_.at(vehicle).get_obj() };
    uint32_t new_obj { tours_.at(vehicle).pop_front(dist_matrix_) };
    total_obj_ -= old_obj - new_obj;
    return total_obj_;
}


uint32_t MTSPBC::get_obj_vehicle(const uint32_t vehicle) const {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    return tours_.at(vehicle).get_obj();
}


std::vector<uint32_t> MTSPBC::get_tour(const uint32_t vehicle) const {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    return tours_.at(vehicle).get_tour();
}


size_t MTSPBC::get_pos_for_node(const uint32_t vehicle, const uint32_t node) {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    return tours_.at(vehicle).get_pos_for_node(node);
}


uint32_t MTSPBC::get_node_at_pos(const uint32_t vehicle, const size_t pos) {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    return tours_.at(vehicle).get_node_at_pos(pos);
}


bool MTSPBC::get_complete_tour(const uint32_t vehicle) const {
    if (k_vehicles_ - 1 < vehicle) {
        throw std::logic_error("error: vehicle do not exist");
    }
    return tours_.at(vehicle).get_complete();
}

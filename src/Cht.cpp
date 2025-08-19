#include <cstddef>
#include <cstdint>
#include "Cht.hpp"
#include <iostream>
#include <algorithm>
#include <iterator>
#include <limits>
#include <memory>
#include <vector>


// construtor
Cht::Cht(){
    obj_ = 0;
    complete_tour_ = false;
}


uint32_t Cht::insert_node(const uint32_t node, const size_t pos, const std::vector<std::vector<uint32_t>>& dist_matrix) {
    if (tour_.size() - 1 < pos) {
        std::cerr << "insert node error: no such position" << std::endl;
        return 0;
    }
    auto it_pos { tour_.begin() + pos };
    if (tour_.size() == 0 || tour_.end() == it_pos) {
        return push_back(node, dist_matrix);
    } else if (tour_.begin() == it_pos) {
        return push_front(node, dist_matrix);
    } else {
        tour_.insert(it_pos, node);
        check_complete_tour_();
        compute_events_(dist_matrix);
        if (tour_.front() == 0 && tour_.back() == 0) {
            complete_tour_ = true;
        } else complete_tour_ = false;
        return compute_obj_insert_(pos - 1, pos, pos + 1, dist_matrix);
    }
    return obj_;
}


uint32_t Cht::remove_node(const size_t pos, const std::vector<std::vector<uint32_t>>& dist_matrix) {
    if (tour_.size() - 1 < pos) {
        std::cerr << "remove node error: no such position" << std::endl;
        return obj_;
    } else if (tour_.size() == 0) {
        std::cerr << "error: cannot remove node from empty tour" << std::endl;
        return 0;
    }
    auto it_pos { tour_.begin() + pos };
    if (it_pos == tour_.begin()) {
        return pop_front(dist_matrix);
    } else if (it_pos == tour_.end()) {
        return pop_back(dist_matrix);
    } else {
        uint32_t new_obj { compute_obj_remove_(pos - 1, pos, pos + 1, dist_matrix) };
        tour_.erase(it_pos);
        check_complete_tour_();
        compute_events_(dist_matrix);
        return new_obj;
    }
}


uint32_t Cht::get_obj() const {
    return obj_;
}


std::vector<uint32_t> Cht::get_tour() const {
    return tour_;
}


size_t Cht::get_pos_for_node(const uint32_t node) {
    auto it { std::find(tour_.begin(), tour_.end(), node) };
    if (it == tour_.end()) {
        std::cerr << "node not found" << std::endl;
        return std::numeric_limits<size_t>::max();
    }
    return std::distance(tour_.begin(), it);
}


uint32_t Cht::get_node_at_pos(const size_t pos) {
    if (tour_.size() - 1 < pos) {
        std::cerr << "empty tour_" << std::endl;
        return std::numeric_limits<uint32_t>::max();
    }
    return tour_.at(pos);
}


uint32_t Cht::compute_obj_insert_(size_t pos_A, size_t pos_B, size_t pos_C, const std::vector<std::vector<uint32_t>>& dist_matrix) {
    uint32_t node_A { tour_.at(pos_A) };
    uint32_t node_B { tour_.at(pos_B) };
    uint32_t node_C { tour_.at(pos_C) };
    uint32_t broken_edge_obj { dist_matrix.at(node_A).at(node_C) };
    uint32_t add_edge_AB { dist_matrix.at(node_A).at(node_B) };
    uint32_t add_edge_BC { dist_matrix.at(node_B).at(node_C) };
    uint32_t diff_obj { add_edge_AB + add_edge_BC - broken_edge_obj };
    obj_ += diff_obj;
    return obj_;
}


uint32_t Cht::compute_obj_insert_(const bool at_end, const std::vector<std::vector<uint32_t>>& dist_matrix) {
    if (tour_.size() == 1) {
        return 0;
    }
    if (at_end) {
        uint32_t node_A { tour_.back() };
        uint32_t node_B { tour_.at(tour_.size() - 2) };
        uint32_t additional_obj { dist_matrix.at(node_A).at(node_B) };
        obj_ += additional_obj;
    } else {
        uint32_t node_A { tour_.front() };
        uint32_t node_B { tour_.at(1) };
        uint32_t additional_obj { dist_matrix.at(node_A).at(node_B) };
        obj_ += additional_obj;
    }
    return obj_;
}


uint32_t Cht::compute_obj_remove_(size_t pos_A, size_t pos_B, size_t pos_C, const std::vector<std::vector<uint32_t>>& dist_matrix) {
    if (tour_.size() < 3) {
        std::cerr << "error: cannot remove from middle" << std::endl;
        return 0;
    }
    uint32_t node_A { tour_.at(pos_A) };
    uint32_t node_B { tour_.at(pos_B) };
    uint32_t node_C { tour_.at(pos_C) };
    uint32_t direct_edge_obj { dist_matrix.at(node_A).at(node_C) };
    uint32_t broken_edge_AB { dist_matrix.at(node_A).at(node_B) };
    uint32_t broken_edge_BC { dist_matrix.at(node_B).at(node_C) };
    int32_t diff_obj { static_cast<int32_t>(direct_edge_obj - broken_edge_AB - broken_edge_BC) };
    obj_ += diff_obj;
    return obj_;
}


uint32_t Cht::compute_obj_remove_(const bool at_end, const std::vector<std::vector<uint32_t>>& dist_matrix) {
    if (tour_.size() == 0 || tour_.size() == 1) {
        obj_ = 0;
    } else if (at_end == true) {
        uint32_t node_A { tour_.back() };
        uint32_t node_B { tour_.at(tour_.size() - 2) };
        obj_ -= dist_matrix.at(node_A).at(node_B);
    } else {
        uint32_t node_A { tour_.front() };
        uint32_t node_B { tour_.at(1) };
        obj_ -= dist_matrix.at(node_A).at(node_B);
    }
    return obj_;
}


uint32_t Cht::compute_events_(const std::vector<std::vector<uint32_t>>& dist_matrix) {
    std::vector<uint32_t> new_events { 0 };
    for (auto i{ 1 }; i < tour_.size(); i++) {
        uint32_t curr_node { tour_.at(i) };
        uint32_t prev_node { tour_.at(i - 1) };
        uint32_t new_event_diff { dist_matrix.at(prev_node).at(curr_node) };
        new_events.push_back(new_event_diff + new_events.back());
    }
    events_ = new_events;
    return events_.back();
}


uint32_t Cht::push_back(const uint32_t node, const std::vector<std::vector<uint32_t>>& dist_matrix) {
    tour_.push_back(node);
    compute_events_(dist_matrix);
    check_complete_tour_();
    if (tour_.front() == 0 && tour_.back() == 0) {
        complete_tour_ = true;
    } else {
        complete_tour_ = false;
    }
    if (tour_.size() == 1) {
        return 0;
    }
    return compute_obj_insert_(true, dist_matrix);
}


uint32_t Cht::push_front(const uint32_t node, const std::vector<std::vector<uint32_t>>& dist_matrix) {
    if (tour_.size() == 0) {
        tour_.push_back(node);
        compute_events_(dist_matrix);
        return 0;
    }
    tour_.insert(tour_.begin(), node);
    compute_events_(dist_matrix);
    check_complete_tour_();
    if (tour_.front() == 0 && tour_.back() == 0) {
        complete_tour_ = true;
    } else {
        complete_tour_ = false;
    }
    return compute_obj_insert_(false, dist_matrix);
}


uint32_t Cht::pop_front(const std::vector<std::vector<uint32_t>>& dist_matrix) {
    if (tour_.size() == 0) {
        std::cerr << "error: cannot remove node from empty tour" << std::endl;
        return std::numeric_limits<uint32_t>::max();
    }
    uint32_t new_obj{ compute_obj_remove_(false, dist_matrix) };
    tour_.erase(tour_.begin());
    compute_events_(dist_matrix);
    check_complete_tour_();
    return new_obj;
}


uint32_t Cht::pop_back(const std::vector<std::vector<uint32_t>>& dist_matrix) {
    if (tour_.size() == 0) {
        std::cerr << "cannot remove node: empty tour" << std::endl;
        return 0;
    }
    uint32_t new_obj { compute_obj_remove_(true, dist_matrix) };
    tour_.pop_back();
    compute_events_(dist_matrix);
    check_complete_tour_();
    return new_obj;
}


bool Cht::check_complete_tour_() {
    if (tour_.size() < 3) {
        complete_tour_ = false;
        return false;
    }
    else if (tour_.front() != 0 || tour_.back() != 0) {
        complete_tour_ = false;
        return false;
    }
    complete_tour_ = true;
    return true;
}


size_t Cht::n_nodes() const {
    return tour_.size();
}

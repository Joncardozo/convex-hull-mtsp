/**
 * @file Cht.cpp
 * @brief Class Cht implementation
 * @details This file contains the implementation
 * of the Cht class. This class encapsulates a tour
 * with objective value, events (time a vehicle
 * visits a node), directed list of nodes decribing
 * a tour. The methods operate over the tour adding
 * and removing nodes, getting objective value,
 * checking if the tour is complete, and other tour
 * utility methods.
 */


#include "Cht.hpp"
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <vector>


/**
 * @brief Constructor of the Cht class.
 * @details Initialize an empty object.
 */
Cht::Cht(){
    obj_ = 0;
    complete_tour_ = false;
}


/**
 * @brief Inserts a node in the tour.
 * @details Inserts a given node in the tour, checking if the
 * given position is out of range first. It computes the
 * objective value of the tour, updating the objective
 * value attribute.
 * @param pos The position where a node should be inserted.
 * @param dist_matrix The cost matrix. c[i][j] is the cost
 * of travelling from [i] to [j].
 * @return Returns the updated objective value.
 */
uint32_t Cht::insert_node(const uint32_t node, const size_t pos, const std::vector<std::vector<uint32_t>>& dist_matrix) {
    if (tour_.size() - 1 < pos) {
        throw std::logic_error("insert node error: no such position");
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


/**
 * @brief Removes a node from the tour.
 * @details This method removes a node from a given position
 * and updates the objective value.
 * @param pos The position where the node should be removed.
 * @param dist_matrix The cost matrix. c[i][j] is the cost
 * of travelling from [i] to [j].
 * @return Returns the updated objective value of the tour.
 */
uint32_t Cht::remove_node(const size_t pos, const std::vector<std::vector<uint32_t>>& dist_matrix) {
    if (tour_.size() - 1 < pos) {
        throw std::logic_error("remove node error: no such position");
        return obj_;
    } else if (tour_.size() == 0) {
        throw std::logic_error("error: cannot remove node from empty tour");
        return 0;
    }
    auto it_pos { tour_.begin() + pos };
    auto begin_compare { std::distance(tour_.begin(), it_pos) };
    auto end_compare { std::distance(tour_.end() - 1, it_pos) };
    if (begin_compare == 0) {
        return pop_front(dist_matrix);
    } else if (end_compare == 0) {
        return pop_back(dist_matrix);
    } else {
        uint32_t new_obj { compute_obj_remove_(pos - 1, pos, pos + 1, dist_matrix) };
        tour_.erase(it_pos);
        check_complete_tour_();
        compute_events_(dist_matrix);
        return new_obj;
    }
}


/**
 * @brief Getter for the objective value of the tour.
 * @details This method is just a getter to the
 * objective value of the tour.
 * @return Returns the objective value of the tour.
 */
[[nodiscard]] uint32_t Cht::get_obj() const noexcept { return obj_; }


/**
 * @brief Getter for the vector of nodes describing the tour.
 * @details This method is a getter for the tour, desbribed
 * by a vector where the order of the nodes defines a directed
 * travel order.
 * @return The vector representing the tour.
 */
[[nodiscard]]std::vector<uint32_t> Cht::get_tour() const noexcept { return tour_; }


/**
 * @brief Find the index of a given node in the tour.
 * @details This method finds the first ocurrence of
 * a given node in the tour. If a node is not in tour
 * throws an exception.
 * @param node The node to be found.
 * @return The index of the found node.
 */
[[nodiscard]] size_t Cht::get_pos_for_node(const uint32_t node) const {
    auto it { std::find(tour_.begin(), tour_.end(), node) };
    if (it == tour_.end()) {
        throw std::logic_error("node not found");
    }
    return std::distance(tour_.begin(), it);
}


/**
 * @brief Gets the node in a given position.
 * @details This method gets a node in a given
 * position, checking if the position is out of
 * bounds.
 * @param pos Node position in the tour.
 * @return The node.
 */
[[nodiscard]] uint32_t Cht::get_node_at_pos(const size_t pos) const {
    if (tour_.size() - 1 < pos) {
        throw std::logic_error("empty tour_");
    }
    return tour_.at(pos);
}


/**
 * @brief Computes the objective value.
 * @details Computes the objective value of a insertion
 * that occurs in a given position inside the tour.
 * @param pos_A The position of the antecessor node.
 * @param pos_B The position of the inserted node.
 * @param pos_C The position of the successor node.
 * @param dist_matrix The cost matrix, (c[i][j]).
 * @return The updated objective value.
 */
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


/**
 * @brief Computer the objective value.
 * @details Computes the objective value in the case
 * of inserting a node at the first or last position.
 * @param at_end true if inserts at last position
 * or false if inserting at the beginning.
 * @return The updated objective value.
 */
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
        throw std::logic_error("error: cannot remove from middle");
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
        throw std::logic_error("error: cannot remove node from empty tour");
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
        throw std::logic_error("cannot remove node: empty tour");
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


[[nodiscard]] size_t Cht::n_nodes() const noexcept { return tour_.size(); }


[[nodiscard]] std::vector<uint32_t> Cht::get_events() const noexcept { return events_;}


[[nodiscard]] bool Cht::get_complete() const noexcept { return complete_tour_; }

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
#include <ios>
#include <iterator>
#include <limits>
#include <optional>
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
 * @param instance The cost matrix. c[i][j] is the cost
 * of travelling from [i] to [j].
 * @return Returns the updated objective value.
 */
uint32_t Cht::insert_node(const uint32_t node, const size_t pos, const MTSPBCInstance& instance) {
    if (tour_.size() - 1 < pos) {
        throw std::logic_error("insert node error: no such position");
        return 0;
    }
    auto it_pos { tour_.begin() + pos };
    if (tour_.size() == 0 || tour_.end() == it_pos) {
        return push_back(node, instance);
    } else if (tour_.begin() == it_pos) {
        return push_front(node, instance);
    } else {
        tour_.insert(it_pos, node);
        check_complete_tour_();
        compute_events_(pos, instance);
        if (tour_.front() == 0 && tour_.back() == 0) {
            complete_tour_ = true;
        } else complete_tour_ = false;
        return compute_obj_insert_(pos - 1, pos, pos + 1, instance);
    }
    return obj_;
}


/**
 * @brief Removes a node from the tour.
 * @details This method removes a node from a given position
 * and updates the objective value.
 * @param pos The position where the node should be removed.
 * @param instance The cost matrix. c[i][j] is the cost
 * of travelling from [i] to [j].
 * @return Returns the updated objective value of the tour.
 */
uint32_t Cht::remove_node(const size_t pos, const MTSPBCInstance& instance) {
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
        return pop_front(instance);
    } else if (end_compare == 0) {
        return pop_back(instance);
    } else {
        uint32_t new_obj { compute_obj_remove_(pos - 1, pos, pos + 1, instance) };
        tour_.erase(it_pos);
        check_complete_tour_();
        compute_events_(pos, instance);
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
[[nodiscard]] std::optional<size_t> Cht::get_pos_for_node(const uint32_t node) const {
    auto it { std::find(tour_.begin(), tour_.end(), node) };

    if (it == tour_.end()) {
        return std::nullopt;
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
 * @param instance The cost matrix, (c[i][j]).
 * @return The updated objective value.
 */
uint32_t Cht::compute_obj_insert_(size_t pos_A, size_t pos_B, size_t pos_C, const MTSPBCInstance& instance) {
    uint32_t node_A { tour_.at(pos_A) };
    uint32_t node_B { tour_.at(pos_B) };
    uint32_t node_C { tour_.at(pos_C) };
    uint32_t broken_edge_obj { instance.cost(node_A, node_C) };
    uint32_t add_edge_AB { instance.cost(node_A, node_B) };
    uint32_t add_edge_BC { instance.cost(node_B, node_C) };
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
uint32_t Cht::compute_obj_insert_(const bool at_end, const MTSPBCInstance& instance) {
    if (tour_.size() == 1) {
        return 0;
    }
    if (at_end) {
        uint32_t node_A { tour_.back() };
        uint32_t node_B { tour_.at(tour_.size() - 2) };
        uint32_t additional_obj { instance.cost(node_A, node_B) };
        obj_ += additional_obj;
    } else {
        uint32_t node_A { tour_.front() };
        uint32_t node_B { tour_.at(1) };
        uint32_t additional_obj { instance.cost(node_A, node_B) };
        obj_ += additional_obj;
    }
    return obj_;
}


uint32_t Cht::compute_obj_insert_(const uint32_t pos_i, const uint32_t pos_e, const MTSPBCInstance& instance) {
    uint32_t subtour_obj { 0 };
    uint32_t last_i { pos_i };
    for (uint32_t i { pos_i }; i <= pos_e; i++) {
        if (i == pos_i) {
            continue;
        }
        subtour_obj += instance.cost(tour_.at(last_i), tour_.at(i));
        last_i = i;
    }
    if (pos_i != 0) {
        subtour_obj += instance.cost(tour_.at(pos_i - 1), tour_.at(pos_i));
    }
    if (pos_e != tour_.size() - 1) {
        subtour_obj += instance.cost(tour_.at(pos_e), tour_.at(pos_e + 1));
    }
    obj_ += subtour_obj;
    return obj_;
}


uint32_t Cht::compute_obj_remove_(size_t pos_A, size_t pos_B, size_t pos_C, const MTSPBCInstance& instance) {
    if (tour_.size() < 3) {
        throw std::logic_error("error: cannot remove from middle");
        return 0;
    }
    uint32_t node_A { tour_.at(pos_A) };
    uint32_t node_B { tour_.at(pos_B) };
    uint32_t node_C { tour_.at(pos_C) };
    uint32_t direct_edge_obj { instance.cost(node_A, node_C) };
    uint32_t broken_edge_AB { instance.cost(node_A, node_B) };
    uint32_t broken_edge_BC { instance.cost(node_B, node_C) };
    int32_t diff_obj { static_cast<int32_t>(direct_edge_obj - broken_edge_AB - broken_edge_BC) };
    obj_ += diff_obj;
    return obj_;
}


uint32_t Cht::compute_obj_remove_(const bool at_end, const MTSPBCInstance& instance) {
    if (tour_.size() == 0 || tour_.size() == 1) {
        obj_ = 0;
    } else if (at_end == true) {
        uint32_t node_A { tour_.back() };
        uint32_t node_B { tour_.at(tour_.size() - 2) };
        obj_ -= instance.cost(node_A, node_B);
    } else {
        uint32_t node_A { tour_.front() };
        uint32_t node_B { tour_.at(1) };
        obj_ -= instance.cost(node_A, node_B);
    }
    return obj_;
}


uint32_t Cht::compute_obj_remove_(const uint32_t pos_i, const uint32_t pos_e, const MTSPBCInstance& instance) {
    uint32_t obj_sub { 0 };
    uint32_t last_i { pos_i };
    for (uint32_t i { pos_i }; i <= pos_e; i++) {
        if (i == pos_i) {
            continue;
        }
        obj_sub += instance.cost(tour_.at(i), tour_.at(last_i));
        last_i = i;
    }
    if (pos_i != 0) {
        obj_sub += instance.cost(tour_.at(pos_i - 1), tour_.at(pos_i));
    }
    if (pos_e != tour_.size() - 1) {
        obj_sub += instance.cost(tour_.at(pos_e + 1), tour_.at(pos_e));
    }
    obj_ -= obj_sub;
    return obj_;
}


uint32_t Cht::compute_events_(const MTSPBCInstance& instance) {
    std::vector<uint32_t> new_events { 0 };
    for (auto i{ 1 }; i < tour_.size(); i++) {
        uint32_t curr_node { tour_.at(i) };
        uint32_t prev_node { tour_.at(i - 1) };
        uint32_t new_event_diff { instance.cost(prev_node, curr_node) };
        new_events.push_back(new_event_diff + new_events.back());
    }
    events_ = new_events;
    return events_.back();
}


uint32_t Cht::compute_events_(const uint32_t inserted_pos, const MTSPBCInstance& instance) {
    if (inserted_pos > tour_.size() - 1) {
        throw std::logic_error("error: cannot compute events on out of range position");
    }
    std::vector<uint32_t> new_events {  };
    std::vector<uint32_t> old_events { events_ };
    for (auto i{ inserted_pos }; i < tour_.size(); i++) {
        if (inserted_pos == 0) {
            compute_events_(instance);
            return events_.back();
        }
        uint32_t curr_node { tour_.at(i) };
        uint32_t prev_node { tour_.at(i - 1) };
        uint32_t new_event_diff { instance.cost(prev_node, curr_node) };
        if (i == inserted_pos) {
            new_events.push_back(new_event_diff + events_.at(i - 1));
        }
        else {
            new_events.push_back(new_event_diff + new_events.back());
        }
    }
    events_.erase(events_.begin() + inserted_pos, events_.end());
    events_.insert(events_.end(), new_events.begin(), new_events.end());
    return events_.back();
}


uint32_t Cht::push_back(const uint32_t node, const MTSPBCInstance& instance) {
    tour_.push_back(node);
    size_t pos { tour_.size() - 1 };
    compute_events_(pos, instance);
    check_complete_tour_();
    if (tour_.front() == 0 && tour_.back() == 0) {
        complete_tour_ = true;
    } else {
        complete_tour_ = false;
    }
    if (tour_.size() == 1) {
        return 0;
    }
    return compute_obj_insert_(true, instance);
}


uint32_t Cht::push_front(const uint32_t node, const MTSPBCInstance& instance) {
    if (tour_.size() == 0) {
        tour_.push_back(node);
        compute_events_(instance);
        return 0;
    }
    tour_.insert(tour_.begin(), node);
    compute_events_(instance);
    check_complete_tour_();
    if (tour_.front() == 0 && tour_.back() == 0) {
        complete_tour_ = true;
    } else {
        complete_tour_ = false;
    }
    return compute_obj_insert_(false, instance);
}


uint32_t Cht::pop_front(const MTSPBCInstance& instance) {
    if (tour_.size() == 0) {
        throw std::logic_error("error: cannot remove node from empty tour");
        return std::numeric_limits<uint32_t>::max();
    }
    uint32_t new_obj{ compute_obj_remove_(false, instance) };
    tour_.erase(tour_.begin());
    compute_events_(instance);
    check_complete_tour_();
    return new_obj;
}


uint32_t Cht::pop_back(const MTSPBCInstance& instance) {
    if (tour_.size() == 0) {
        throw std::logic_error("cannot remove node: empty tour");
        return 0;
    }
    uint32_t new_obj { compute_obj_remove_(true, instance) };
    tour_.pop_back();
    compute_events_(tour_.size() - 1, instance);
    check_complete_tour_();
    return new_obj;
}


uint32_t Cht::insert_subtour(const MTSPBCInstance& instance, const std::vector<uint32_t>& subtour_indices, const uint32_t pos_i, const uint32_t pos_e) {
    if (pos_i > tour_.size() - 1 || pos_e > tour_.size() - 1 || pos_i >= pos_e) {
        throw std::logic_error("error: invlid range");
    }

    tour_.insert(events_.begin() + pos_e, subtour_indices.begin(), subtour_indices.end());
    compute_events_(pos_i, instance);
    compute_obj_insert_(pos_i, pos_e, instance);
    check_complete_tour_();
    return obj_;
}


uint32_t Cht::replace_subtour(const MTSPBCInstance& instance, const std::vector<uint32_t>& subtour_indices, const uint32_t pos_i, const uint32_t pos_e) {
    compute_obj_remove_(pos_i, pos_e, instance);
    tour_.erase(tour_.begin() + pos_i, tour_.begin() + pos_e);
    tour_.insert(tour_.begin() + pos_i, subtour_indices.begin(), subtour_indices.end());
    compute_obj_insert_(pos_i, pos_e, instance);
    compute_events_(pos_i, instance);
    check_complete_tour_();
    return obj_;
}


uint32_t Cht::remove_subtour(const MTSPBCInstance& instance, const uint32_t pos_i, const uint32_t pos_e) {
    compute_obj_remove_(pos_i, pos_e, instance);
    tour_.erase(tour_.begin() + pos_i, tour_.begin() + pos_e);
    compute_events_(pos_i, instance);
    check_complete_tour_();
    return obj_;
}


uint32_t Cht::reverse_subtour(const MTSPBCInstance& instance, const uint32_t pos_i, const uint32_t pos_e) {
    std::reverse(tour_.begin() + pos_i, tour_.begin() + pos_e);
    compute_events_(pos_i, instance);
    check_complete_tour_();
    return obj_;
}


uint32_t Cht::reverse_tour(const MTSPBCInstance& instance) {
    std::reverse(tour_.begin(), tour_.end());
    compute_events_(instance);
    check_complete_tour_();
    return 0;
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


[[nodiscard]] std::optional<uint32_t> Cht::get_node_at_event(const uint32_t e_time) const {
    auto e_node { std::find(events_.begin(), events_.end(), e_time) };
    if (e_node == events_.end()) {
        return std::nullopt;
    }
    return std::distance(events_.begin(), e_node);
}

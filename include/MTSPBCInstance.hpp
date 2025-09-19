#pragma once


#include "MTSPBC_ds.hpp"
#include <cstdint>
#include <string>
#include <vector>


struct InstanceData {
    std::vector<std::vector<uint32_t>> cost_matrix;
    std::vector<std::vector<std::vector<double>>> LB;
    std::vector<std::vector<std::vector<double>>> UB;
    std::vector<Coord> coordinates;
    uint32_t k_vehicles;
    uint32_t n_nodes;
    uint32_t r_radius;
};


class MTSPBCInstance {

    private:

    const std::vector<std::vector<uint32_t>> cost_matrix_;
    const std::vector<std::vector<std::vector<double>>> LB_;
    const std::vector<std::vector<std::vector<double>>> UB_;
    const std::vector<Coord> coordinates_;
    const uint32_t k_vehicles_;
    const uint32_t n_nodes_;
    const uint32_t r_radius_;

    MTSPBCInstance(const InstanceData& data);
    static InstanceData parse_instance(const std::string& filepath, const std::string& dist_filepath, const std::string& cover_filepath);

    public:

    MTSPBCInstance(const std::string& instance_filepath, const std::string& dist_filepath, const std::string& cover_filepath);

    [[nodiscard]] double get_LB(const uint32_t covered_node, const uint32_t departure_node, const uint32_t arrival_node) const;
    [[nodiscard]] double get_UB(const uint32_t covered_node, const uint32_t departure_node, const uint32_t arrival_node) const;
    [[nodiscard]] uint32_t cost(const uint32_t node_A, const uint32_t node_B) const;
    [[nodiscard]] Coord coordinate(const uint32_t node) const;
    [[nodiscard]] uint32_t k() const noexcept;
    [[nodiscard]] uint32_t n() const noexcept;
    [[nodiscard]] uint32_t r() const noexcept;

};

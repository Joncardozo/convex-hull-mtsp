#pragma once


#include "MTSPBC_ds.hpp"
#include <cstdint>
#include <string>
#include <vector>


struct InstanceData {
    std::vector<std::vector<uint32_t>> cost_matrix;
    std::vector<Coord> coordinates;
    uint32_t k_vehicles;
    uint32_t n_nodes;
    uint32_t r_radius;
};


class MTSPBCInstance {

    private:

    const std::vector<std::vector<uint32_t>> cost_matrix_;
    const std::vector<Coord> coordinates_;
    const uint32_t k_vehicles_;
    const uint32_t n_nodes_;
    const uint32_t r_radius_;

    MTSPBCInstance(const InstanceData& data);
    static InstanceData parse_instance(const std::string& filepath, const std::string& dist_filepath);


    public:


    MTSPBCInstance(const std::string& instance_filepath, const std::string& dist_filepath);

    [[nodiscard]] uint32_t cost(const uint32_t& node_A, const uint32_t node_B) const;
    [[nodiscard]] Coord coordinate(const uint32_t& node) const;
    [[nodiscard]] uint32_t k() const noexcept;
    [[nodiscard]] uint32_t n() const noexcept;
    [[nodiscard]] uint32_t r() const noexcept;

};

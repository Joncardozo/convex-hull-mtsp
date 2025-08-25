#include "MTSPBCInstance.hpp"
#include <cstdint>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <string>


InstanceData MTSPBCInstance::parse_instance(const std::string& inst_filepath, const std::string& dist_filepath) {
    InstanceData data;
    std::ifstream input_instance(inst_filepath);
    std::ifstream input_dist(dist_filepath);
    bool parameters_read {false};

    // checks if it was open
    if(!input_instance.is_open() || !input_dist.is_open()) {
        throw std::runtime_error("error: could not open file");
    }

    std::string line;                   // string para ler as linhas
    while (std::getline(input_instance, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::stringstream ss(line);     // stream para ler os valores separados por espaços

        if (!parameters_read) {
            ss >> data.k_vehicles >> data.n_nodes >> data.r_radius;
            data.n_nodes++;                  // adiciona garagem ao número de nós
            data.cost_matrix.resize(data.n_nodes, std::vector<uint32_t>(data.n_nodes));     // aloca matriz de distancias
            parameters_read = true;
        } else {
            Coord new_coord;
            ss >> new_coord.pos_x >> new_coord.pos_y;
            data.coordinates.push_back(new_coord);
        }
    }
    input_instance.close();
    parameters_read = false;
    while (std::getline(input_dist, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::stringstream ss(line);
        if (!parameters_read) {
            parameters_read = true;
        }
        else {
            uint32_t node_A {};
            uint32_t node_B {};
            uint32_t dist_AB {};
            ss >> node_A >> node_B >> dist_AB;
            data.cost_matrix.at(node_A).at(node_B) = dist_AB;
        }
    }
    return data;
}


MTSPBCInstance::MTSPBCInstance(const InstanceData& data)
: cost_matrix_(data.cost_matrix),
coordinates_(data.coordinates),
k_vehicles_(data.k_vehicles),
r_radius_(data.r_radius),
n_nodes_(data.n_nodes) {}


MTSPBCInstance::MTSPBCInstance(const std::string& instance_filepath, const std::string& dist_filepath)
: MTSPBCInstance(parse_instance(instance_filepath, dist_filepath)) {}



[[nodiscard]] uint32_t MTSPBCInstance::cost(const uint32_t& node_A, const uint32_t node_B) const {
    if ((node_A > n_nodes_ - 1) || (node_B > n_nodes_ - 1)) {
        throw std::logic_error("error: node does not exist");
    }
    return cost_matrix_.at(node_A).at(node_B);
}


[[nodiscard]] Coord MTSPBCInstance::coordinate(const uint32_t& node) const {
    if (node > n_nodes_ - 1) {
        throw std::out_of_range("error: node does not exist");
    }
    return coordinates_.at(node);
}


[[nodiscard]] uint32_t MTSPBCInstance::k() const noexcept { return k_vehicles_; }
[[nodiscard]] uint32_t MTSPBCInstance::n() const noexcept { return n_nodes_; }
[[nodiscard]] uint32_t MTSPBCInstance::r() const noexcept { return r_radius_; }

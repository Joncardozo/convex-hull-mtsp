#pragma once


#include <cstdint>
#include <cstddef>
#include <utility>


struct Edge {
    std::pair<uint32_t, uint32_t> node_A;
    std::pair<uint32_t, uint32_t> node_B;
    uint32_t A_index() { return node_A.first; }
    uint32_t A_node() { return node_A.second; }
    uint32_t B_index() { return node_B.first; }
    uint32_t B_node() { return node_B.second; }
};


typedef struct Coord  {
    double pos_x;
    double pos_y;

    Coord operator+(const Coord& other) const {
        return {
            pos_x + other.pos_x,
            pos_y + other.pos_y
        };
    }
    Coord operator-(const Coord& other) const {
        return {
            pos_x - other.pos_x,
            pos_y - other.pos_y
        };
    }
    Coord operator/(const double& other) const {
        return {
            pos_x / other,
            pos_y / other
        };
    }
    Coord operator/(const uint32_t& other) const {
        return {
            pos_x / other,
            pos_y / other
        };
    }
    Coord operator*(const double& other) const {
        return {
            pos_x * other,
            pos_y * other
        };
    }
    Coord operator*(const uint32_t& other) const {
        return {
            pos_x * other,
            pos_y * other
        };
    }
} Coord ;


typedef struct Nodes {
    size_t index;
    Coord pos;
} Nodes;

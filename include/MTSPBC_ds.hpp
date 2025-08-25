#pragma once


#include <cstdint>
#include <cstddef>


typedef struct Coord  {
    double pos_x;
    double pos_y;

    // overload minus operator
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

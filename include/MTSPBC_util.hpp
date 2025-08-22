#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>


typedef struct Coord  {
    double pos_x;
    double pos_y;
} Coord ;


typedef struct Nodes {
    size_t index;
    Coord pos;
} Nodes;


int orientation(Coord a, Coord b, Coord c);
uint32_t distance(const Nodes& a, const Nodes& b);
uint32_t distance(const Coord& a, const Coord& b);
uint32_t unassign(const std::vector<uint32_t>& nodes, std::vector<size_t>& un_nodes);
